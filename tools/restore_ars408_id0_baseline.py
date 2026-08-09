#!/usr/bin/env python3
"""Parked-only ARS408 SensorID 0 baseline restore with reply verification.

The ARS408 protocol has no factory-reset command. This tool restores a known
unfiltered baseline: SensorID 0, object output, quality enabled, 250 m range,
standard power/sensitivity, range sorting, and every object/cluster filter
disabled. It must run with pandad stopped so it can exclusively claim Panda.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from opendbc.can import CANParser
from opendbc.car.structs import CarParams
from opendbc.car.tesla.ars408_can import ARS408_BUS, ARS408_SENSOR_ID, ARS408CAN
from panda import Panda


CONFIRMATION = "RESTORE_ARS408_ID0_UNFILTERED_BASELINE_ONCE"
LOG_DIR = Path("/data/media/0/ars408_diagnostics")
SUCCESS_MARKER = LOG_DIR / "ars408_id0_baseline_restore.json"
SOURCE_SENSOR_ID = 5
TARGET_SENSOR_ID = 0
SOURCE_CONFIG_ADDRESS = 0x200 + (SOURCE_SENSOR_ID << 4)
TARGET_RADAR_STATE_ADDRESS = 0x201
TARGET_FILTER_CONFIG_ADDRESS = 0x202
TARGET_FILTER_STATE_ADDRESS = 0x204


def receive_until(panda: Panda, deadline: float):
  while time.monotonic() < deadline:
    for address, data, bus in panda.can_recv():
      if bus == ARS408_BUS:
        yield int(address), bytes(data)


def decode(message: str, address: int, data: bytes) -> dict:
  parser = CANParser("ARS408", [(message, math.nan)], ARS408_BUS)
  parser.update([(time.monotonic_ns(), [(address, data, ARS408_BUS)])])
  return dict(parser.vl[message])


def sample_current_bus(panda: Panda, duration: float = 3.0) -> dict:
  allowed_base = {0x201, 0x203, 0x204, 0x402, 0x408, 0x600, 0x60A, 0x60B, 0x60C, 0x60D, 0x60E,
                  0x700, 0x701, 0x702}
  allowed = ({address + (SOURCE_SENSOR_ID << 4) for address in allowed_base} |
             {address + (TARGET_SENSOR_ID << 4) for address in allowed_base} | {0x8})
  counts = Counter()
  current_state = None
  unexpected = Counter()
  for address, data in receive_until(panda, time.monotonic() + duration):
    counts[(address, len(data))] += 1
    if address not in allowed:
      unexpected[(address, len(data))] += 1
    if address in (0x201 + (SOURCE_SENSOR_ID << 4), TARGET_RADAR_STATE_ADDRESS) and len(data) == 8:
      current_state = decode("RadarState", 0x201, data)
  if unexpected:
    raise RuntimeError(f"Refusing restore: non-radar bus 1 traffic observed: {dict(unexpected)}")
  if current_state is None or int(current_state["RadarState_SensorID"]) not in (SOURCE_SENSOR_ID, TARGET_SENSOR_ID):
    raise RuntimeError("Refusing restore: SensorID 5 or 0 RadarState was not observed")
  return {
    "counts": {f"0x{a:03X}/dlc{dlc}": count for (a, dlc), count in sorted(counts.items())},
    "radar_state": current_state,
  }


def wait_for_target_state(panda: Panda, timeout: float = 4.0) -> dict:
  for address, data in receive_until(panda, time.monotonic() + timeout):
    if address != TARGET_RADAR_STATE_ADDRESS or len(data) != 8:
      continue
    state = decode("RadarState", 0x201, data)
    expected = {
      "RadarState_SensorID": TARGET_SENSOR_ID,
      "RadarState_MaxDistanceCfg": 250,
      "RadarState_OutputTypeCfg": 1,
      "RadarState_SendQualityCfg": 1,
      "RadarState_SendExtInfoCfg": 0,
      "RadarState_SortIndex": 1,
      "RadarState_RadarPowerCfg": 0,
      "RadarState_RCS_Threshold": 0,
      "RadarState_CtrlRelayCfg": 0,
    }
    mismatches = {key: {"expected": value, "actual": state.get(key)}
                  for key, value in expected.items() if state.get(key) != value}
    if mismatches:
      raise RuntimeError(f"Restored RadarState mismatch: {mismatches}")
    return state
  raise RuntimeError("No verified SensorID 0 RadarState after RadarCfg")


def wait_for_disabled_filter(panda: Panda, filter_type: int, index: int, timeout: float = 1.5) -> dict:
  for address, data in receive_until(panda, time.monotonic() + timeout):
    if address != TARGET_FILTER_STATE_ADDRESS or len(data) != 5:
      continue
    state = decode("FilterState_Cfg", 0x204, data)
    if int(state["FilterState_Type"]) != filter_type or int(state["FilterState_Index"]) != index:
      continue
    if int(state["FilterState_Active"]) != 0:
      raise RuntimeError(f"Filter remained active: type={filter_type} index={index} state={state}")
    return state
  raise RuntimeError(f"No disabled FilterState reply: type={filter_type} index={index}")


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--execute", action="store_true")
  parser.add_argument("--confirm", default="")
  args = parser.parse_args()

  if ARS408_SENSOR_ID != TARGET_SENSOR_ID:
    raise RuntimeError(f"Device code must target SensorID {TARGET_SENSOR_ID}, got {ARS408_SENSOR_ID}")
  if Path("/data/params/d/IsOffroad").read_text().strip() != "1":
    raise RuntimeError("Refusing restore: device must be offroad")
  if SUCCESS_MARKER.exists():
    raise RuntimeError(f"Refusing restore: success marker already exists at {SUCCESS_MARKER}")
  if not args.execute:
    print("DRY RUN: would migrate SensorID 5 to 0 and disable all object/cluster filters")
    return 0
  if args.confirm != CONFIRMATION:
    raise RuntimeError(f"Refusing restore: --confirm must equal {CONFIRMATION}")

  can = ARS408CAN()
  target_address, radar_data, target_bus = can.create_radar_configuration(store_in_nvm=True)
  if target_address != 0x200 or target_bus != ARS408_BUS or len(radar_data) != 8:
    raise RuntimeError("Generated RadarCfg address/bus/DLC mismatch")
  cfg = decode("RadarConfiguration", 0x200, bytes(radar_data))
  expected_cfg = {
    "RadarCfg_SensorID": 0, "RadarCfg_MaxDistance": 250, "RadarCfg_OutputType": 1,
    "RadarCfg_SendQuality": 1, "RadarCfg_SendExtInfo": 0, "RadarCfg_SortIndex": 1,
    "RadarCfg_RadarPower": 0, "RadarCfg_RCS_Threshold": 0, "RadarCfg_CtrlRelay": 0,
    "RadarCfg_StoreInNVM": 1,
  }
  mismatches = {key: {"expected": value, "actual": cfg.get(key)}
                for key, value in expected_cfg.items() if cfg.get(key) != value}
  if mismatches:
    raise RuntimeError(f"Generated RadarCfg mismatch: {mismatches}")

  filter_messages = []
  # Continental table 4: cluster filters support indices 0..5; object
  # filters support indices 0..14. Reserved/unsupported mux values do not reply.
  for filter_type, indices in ((0, range(6)), (1, range(15))):
    for index in indices:
      data = bytes(((0x80 if filter_type else 0x00) | 0x02 | (index << 3), 0, 0, 0, 0))
      filter_messages.append((filter_type, index, data))

  report = {
    "test": "ARS408 SensorID 0 unfiltered baseline restore",
    "created_utc": datetime.now(timezone.utc).isoformat(),
    "source_sensor_id": SOURCE_SENSOR_ID,
    "target_sensor_id": TARGET_SENSOR_ID,
    "radar_cfg_payload": bytes(radar_data).hex(),
    "filters_to_disable": len(filter_messages),
    "executed": False,
  }

  with Panda(cli=False) as panda:
    health = panda.health()
    if health["ignition_line"] or health["ignition_can"] or health["controls_allowed"]:
      raise RuntimeError(f"Refusing restore: ignition/controls must be off: {health}")
    panda.can_clear(0xFFFF)
    report["before"] = sample_current_bus(panda)
    verified = []
    try:
      panda.set_safety_mode(CarParams.SafetyModel.allOutput)
      current_sensor_id = int(report["before"]["radar_state"]["RadarState_SensorID"])
      if current_sensor_id == SOURCE_SENSOR_ID:
        panda.can_send(SOURCE_CONFIG_ADDRESS, radar_data, ARS408_BUS)
        report["after_radar_state"] = wait_for_target_state(panda)
      else:
        report["after_radar_state"] = report["before"]["radar_state"]
        report["sensor_id_already_restored"] = True
      for filter_type, index, data in filter_messages:
        last_error = None
        for _attempt in range(3):
          panda.can_send(TARGET_FILTER_CONFIG_ADDRESS, data, ARS408_BUS)
          try:
            state = wait_for_disabled_filter(panda, filter_type, index)
            break
          except RuntimeError as error:
            last_error = error
        else:
          raise last_error
        verified.append({"type": filter_type, "index": index, "state": state})
        time.sleep(0.15)
      report["executed"] = True
    finally:
      panda.set_safety_mode(CarParams.SafetyModel.noOutput)

    health_after = panda.health()
    if int(health_after["safety_mode"]) != int(CarParams.SafetyModel.noOutput):
      raise RuntimeError(f"Panda did not return to noOutput: {health_after}")
    report["after_health"] = health_after
    report["verified_filters"] = verified

  LOG_DIR.mkdir(parents=True, exist_ok=True)
  SUCCESS_MARKER.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
  print(json.dumps(report, indent=2, sort_keys=True))
  print(f"VERIFIED: {SUCCESS_MARKER}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
