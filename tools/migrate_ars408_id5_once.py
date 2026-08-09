#!/usr/bin/env python3
"""Parked-only, single-shot ARS408 Sensor ID 2 -> 5 NVM migration."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from cereal import car, messaging
from opendbc.can import CANPacker, CANParser


BUS = 1
CURRENT_SENSOR_ID = 2
TARGET_SENSOR_ID = 5
CURRENT_CONFIG_ADDRESS = 0x200 + (CURRENT_SENSOR_ID << 4)
CURRENT_STATE_ADDRESS = 0x201 + (CURRENT_SENSOR_ID << 4)
TARGET_STATE_ADDRESS = 0x201 + (TARGET_SENSOR_ID << 4)
LOG_DIR = Path("/data/media/0/ars408_diagnostics")
SUCCESS_MARKER = LOG_DIR / "ars408_id5_migration_complete.json"
CONFIRMATION = "MIGRATE_ID2_TO_ID5_ONCE"


def decode_state(data: bytes) -> dict:
  parser = CANParser("ARS408", [("RadarState", math.nan)], BUS)
  parser.update([(time.monotonic_ns(), [(0x201, data, BUS)])])
  return {key: value for key, value in parser.vl["RadarState"].items()}


def wait_for_frame(sock, address: int, dlc: int, timeout: float) -> bytes | None:
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    event = messaging.recv_one_or_none(sock)
    if event is None:
      continue
    for frame in event.can:
      if int(frame.src) == BUS and int(frame.address) == address and len(frame.dat) == dlc:
        return bytes(frame.dat)
  return None


def wait_for_target_state(sock, timeout: float) -> tuple[bytes, dict] | None:
  """Ignore unrelated vehicle frames and the radar's brief old-ID transition state."""
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    raw = wait_for_frame(sock, TARGET_STATE_ADDRESS, 8, min(0.5, deadline - time.monotonic()))
    if raw is None:
      continue
    state = decode_state(raw)
    if (int(state["RadarState_SensorID"]) == TARGET_SENSOR_ID and
        int(state["RadarState_NVMReadStatus"]) == 1 and
        int(state["RadarState_OutputTypeCfg"]) == 1 and
        int(state["RadarState_SendQualityCfg"]) == 1 and
        int(state["RadarState_SendExtInfoCfg"]) == 0 and
        int(state["RadarState_MaxDistanceCfg"]) == 250):
      return raw, state
  return None


def assert_parked() -> dict:
  sm = messaging.SubMaster(["carState", "pandaStates", "selfdriveState"])
  deadline = time.monotonic() + 5.0
  last_observation = None
  while time.monotonic() < deadline:
    sm.update(500)
    if not sm.valid["carState"] or not sm.valid["pandaStates"] or not sm.valid["selfdriveState"] or not sm["pandaStates"]:
      continue
    state = sm["carState"]
    pandas = sm["pandaStates"]
    parked = state.gearShifter == car.CarState.GearShifter.park
    # This vehicle reports a stationary floor of 0.0694 m/s. Park plus the
    # documented sub-0.1 m/s floor is the reliable stopped condition.
    stopped = abs(float(state.vEgo)) < 0.1
    selfdrive_disabled = not bool(sm["selfdriveState"].enabled)
    last_observation = {
      "gear": str(state.gearShifter),
      "vEgo": float(state.vEgo),
      "controlsAllowed": [bool(panda.controlsAllowed) for panda in pandas],
      "selfdrive_enabled": bool(sm["selfdriveState"].enabled),
      "parked": parked,
      "stopped": stopped,
      "selfdrive_disabled": selfdrive_disabled,
    }
    if parked and stopped and selfdrive_disabled:
      return last_observation
  raise RuntimeError("Refusing migration: vehicle must be stopped in Park with selfdrive disabled; "
                     f"last_observation={last_observation}")


def make_configuration() -> bytes:
  values = {
    "RadarCfg_RCS_Threshold_Valid": 1,
    "RadarCfg_RCS_Threshold": 0,
    "RadarCfg_StoreInNVM_valid": 1,
    "RadarCfg_StoreInNVM": 1,
    "RadarCfg_SortIndex_valid": 1,
    "RadarCfg_SortIndex": 1,
    "RadarCfg_SendExtInfo_valid": 1,
    "RadarCfg_SendExtInfo": 0,
    "RadarCfg_CtrlRelay_valid": 1,
    "RadarCfg_CtrlRelay": 0,
    "RadarCfg_SendQuality_valid": 1,
    "RadarCfg_SendQuality": 1,
    "RadarCfg_MaxDistance_valid": 1,
    "RadarCfg_MaxDistance": 250,
    "RadarCfg_RadarPower_valid": 1,
    "RadarCfg_RadarPower": 0,
    "RadarCfg_OutputType_valid": 1,
    "RadarCfg_OutputType": 1,
    "RadarCfg_SensorID_valid": 1,
    "RadarCfg_SensorID": TARGET_SENSOR_ID,
  }
  _address, data, _bus = CANPacker("ARS408").make_can_msg("RadarConfiguration", BUS, values)
  return bytes(data)


def decode_configuration(data: bytes) -> dict:
  parser = CANParser("ARS408", [("RadarConfiguration", math.nan)], BUS)
  parser.update([(time.monotonic_ns(), [(0x200, data, BUS)])])
  return {key: value for key, value in parser.vl["RadarConfiguration"].items()}


def assert_configuration(data: bytes) -> dict:
  decoded = decode_configuration(data)
  expected = {
    "RadarCfg_SensorID_valid": 1,
    "RadarCfg_SensorID": TARGET_SENSOR_ID,
    "RadarCfg_OutputType_valid": 1,
    "RadarCfg_OutputType": 1,
    "RadarCfg_MaxDistance_valid": 1,
    "RadarCfg_MaxDistance": 250,
    "RadarCfg_SendQuality_valid": 1,
    "RadarCfg_SendQuality": 1,
    "RadarCfg_SendExtInfo_valid": 1,
    "RadarCfg_SendExtInfo": 0,
    "RadarCfg_CtrlRelay_valid": 1,
    "RadarCfg_CtrlRelay": 0,
    "RadarCfg_RadarPower_valid": 1,
    "RadarCfg_RadarPower": 0,
    "RadarCfg_SortIndex_valid": 1,
    "RadarCfg_SortIndex": 1,
    "RadarCfg_RCS_Threshold_Valid": 1,
    "RadarCfg_RCS_Threshold": 0,
    "RadarCfg_StoreInNVM_valid": 1,
    "RadarCfg_StoreInNVM": 1,
  }
  mismatches = {
    key: {"expected": value, "decoded": decoded.get(key)}
    for key, value in expected.items()
    if decoded.get(key) != value
  }
  if mismatches:
    raise RuntimeError(f"Refusing migration: packed configuration mismatch {mismatches}")
  return decoded


def send_once(data: bytes) -> None:
  pub = messaging.PubMaster(["sendcan"])
  # A freshly-created PUB socket can drop its first message before pandad's
  # SUB socket has connected. This tool is deliberately single-shot, so wait
  # for the existing sendcan reader instead of relying on a repeated stream.
  if not pub.wait_for_readers_to_update("sendcan", timeout=5):
    raise RuntimeError("Refusing migration: pandad sendcan reader did not connect")
  msg = messaging.new_message("sendcan", 1)
  msg.sendcan[0].address = CURRENT_CONFIG_ADDRESS
  msg.sendcan[0].dat = data
  msg.sendcan[0].src = BUS
  pub.send("sendcan", msg)


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--execute", action="store_true")
  parser.add_argument("--confirm", default="")
  args = parser.parse_args()

  if SUCCESS_MARKER.exists():
    raise SystemExit(f"Refusing migration: success marker already exists at {SUCCESS_MARKER}")

  parked_state = assert_parked()
  sock = messaging.sub_sock("can", timeout=500)
  current_state = None
  current_deadline = time.monotonic() + 4.0
  while time.monotonic() < current_deadline:
    current_raw = wait_for_frame(sock, CURRENT_STATE_ADDRESS, 8, min(0.5, current_deadline - time.monotonic()))
    if current_raw is None:
      continue
    candidate = decode_state(current_raw)
    if int(candidate["RadarState_SensorID"]) == CURRENT_SENSOR_ID:
      current_state = candidate
      break
  if current_state is None:
    raise RuntimeError(f"Refusing migration: current Sensor ID {CURRENT_SENSOR_ID} RadarState not observed")

  data = make_configuration()
  decoded_configuration = assert_configuration(data)
  report = {
    "test": "ARS408 one-time Sensor ID migration",
    "created_utc": datetime.now(timezone.utc).isoformat(),
    "parked_state": parked_state,
    "current_config_address": f"0x{CURRENT_CONFIG_ADDRESS:03X}",
    "target_state_address": f"0x{TARGET_STATE_ADDRESS:03X}",
    "configuration_data": data.hex(),
    "decoded_configuration": decoded_configuration,
    "before": current_state,
    "executed": False,
  }
  print(json.dumps(report, indent=2, sort_keys=True))

  if not args.execute:
    print("DRY RUN ONLY: no CAN frame sent")
    return 0
  if args.confirm != CONFIRMATION:
    raise RuntimeError(f"Refusing migration: --confirm must equal {CONFIRMATION}")

  send_once(data)
  report["executed"] = True
  report["sent_utc"] = datetime.now(timezone.utc).isoformat()
  target_result = wait_for_target_state(sock, 10.0)
  if target_result is None:
    report["verified"] = False
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    failure = LOG_DIR / "ars408_id3_migration_unverified.json"
    failure.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    raise RuntimeError(f"Migration frame sent once, but target RadarState not observed; see {failure}")

  _target_raw, target_state = target_result
  report["after"] = target_state
  report["verified"] = (
    int(target_state["RadarState_SensorID"]) == TARGET_SENSOR_ID and
    int(target_state["RadarState_NVMReadStatus"]) == 1 and
    int(target_state["RadarState_OutputTypeCfg"]) == 1 and
    int(target_state["RadarState_SendQualityCfg"]) == 1 and
    int(target_state["RadarState_SendExtInfoCfg"]) == 0 and
    int(target_state["RadarState_MaxDistanceCfg"]) == 250
  )
  if not report["verified"]:
    raise RuntimeError(f"Target RadarState failed verification: {target_state}")

  LOG_DIR.mkdir(parents=True, exist_ok=True)
  SUCCESS_MARKER.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
  print(f"VERIFIED: {SUCCESS_MARKER}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
