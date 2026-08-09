#!/usr/bin/env python3
"""Parked-only, one-shot ARS408 Extended and object-filter commissioning.

This tool must be run with pandad stopped so it can exclusively claim Panda.
It enables Extended output in NVM, caps the object list at 32 entries, and
disables every other object pass-through filter without changing SensorID.
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
from opendbc.car.tesla.ars408_can import (
  ARS408_BUS, ARS408_FILTER_CONFIG_ADDRESS, ARS408_MAX_OBJECTS, ARS408_RADAR_CONFIG_ADDRESS,
  ARS408_SEND_EXTENDED, ARS408_SENSOR_ID, ARS408CAN,
)
from panda import Panda


CONFIRMATION = "COMMISSION_ARS408_ID0_EXTENDED_32_OBJECTS_ONCE"
LOG_DIR = Path("/data/media/0/ars408_diagnostics")
SUCCESS_MARKER = LOG_DIR / "ars408_extended_32_objects_complete.json"
RADAR_STATE_ADDRESS = 0x201 + (ARS408_SENSOR_ID << 4)
FILTER_STATE_ADDRESS = 0x204 + (ARS408_SENSOR_ID << 4)

# Every ARS408 output address that may legitimately appear on dedicated bus 1.
EXPECTED_RADAR_OUTPUTS = {
  0x201, 0x203, 0x204, 0x402, 0x408, 0x600, 0x60A, 0x60B, 0x60C, 0x60D, 0x60E,
  0x700, 0x701, 0x702,
}
EXPECTED_RADAR_OUTPUTS = {address + (ARS408_SENSOR_ID << 4) for address in EXPECTED_RADAR_OUTPUTS}
EXPECTED_RADAR_OUTPUTS.add(0x8)


def decode_radar_state(data: bytes) -> dict:
  parser = CANParser("ARS408", [("RadarState", math.nan)], ARS408_BUS)
  parser.update([(time.monotonic_ns(), [(0x201, data, ARS408_BUS)])])
  return dict(parser.vl["RadarState"])


def decode_filter_state(data: bytes) -> dict:
  parser = CANParser("ARS408", [("FilterState_Cfg", math.nan)], ARS408_BUS)
  parser.update([(time.monotonic_ns(), [(0x204, data, ARS408_BUS)])])
  return dict(parser.vl["FilterState_Cfg"])


def receive_until(panda: Panda, deadline: float):
  while time.monotonic() < deadline:
    for address, data, bus in panda.can_recv():
      if bus == ARS408_BUS:
        yield int(address), bytes(data)


def verify_dedicated_radar_bus(panda: Panda, duration: float = 3.0) -> tuple[dict, dict]:
  counts = Counter()
  radar_state = None
  unexpected = Counter()
  for address, data in receive_until(panda, time.monotonic() + duration):
    counts[(address, len(data))] += 1
    if address not in EXPECTED_RADAR_OUTPUTS:
      unexpected[(address, len(data))] += 1
    if address == RADAR_STATE_ADDRESS and len(data) == 8:
      radar_state = decode_radar_state(data)

  if unexpected:
    raise RuntimeError(f"Refusing commissioning: non-radar bus 1 traffic observed: {dict(unexpected)}")
  if radar_state is None:
    raise RuntimeError(f"Refusing commissioning: Sensor ID {ARS408_SENSOR_ID} RadarState was not observed")

  expected_state = {
    "RadarState_SensorID": ARS408_SENSOR_ID,
    "RadarState_OutputTypeCfg": 1,
    "RadarState_SendQualityCfg": 1,
    "RadarState_CtrlRelayCfg": 0,
    "RadarState_SortIndex": 1,
    "RadarState_RCS_Threshold": 0,
    "RadarState_RadarPowerCfg": 0,
    "RadarState_MaxDistanceCfg": 250,
    "RadarState_NVMReadStatus": 1,
  }
  mismatches = {key: {"expected": value, "actual": radar_state.get(key)}
                for key, value in expected_state.items() if radar_state.get(key) != value}
  if mismatches:
    raise RuntimeError(f"Refusing commissioning: RadarState mismatch: {mismatches}")

  formatted_counts = {f"0x{address:03X}/dlc{dlc}": count for (address, dlc), count in sorted(counts.items())}
  return radar_state, formatted_counts


def wait_for_extended_state(panda: Panda, timeout: float = 4.0) -> dict:
  for address, data in receive_until(panda, time.monotonic() + timeout):
    if address != RADAR_STATE_ADDRESS or len(data) != 8:
      continue
    state = decode_radar_state(data)
    if (int(state["RadarState_SensorID"]) == ARS408_SENSOR_ID and
        int(state["RadarState_SendExtInfoCfg"]) == int(ARS408_SEND_EXTENDED) and
        int(state["RadarState_SendQualityCfg"]) == 1 and
        int(state["RadarState_OutputTypeCfg"]) == 1 and
        int(state["RadarState_MaxDistanceCfg"]) == 250 and
        int(state["RadarState_NVMReadStatus"]) == 1):
      return state
  raise RuntimeError("No verified RadarState with Extended enabled")


def wait_for_filter_state(panda: Panda, index: int, active: int, timeout: float = 1.5) -> dict:
  for address, data in receive_until(panda, time.monotonic() + timeout):
    if address != FILTER_STATE_ADDRESS or len(data) != 5:
      continue
    state = decode_filter_state(data)
    if int(state["FilterState_Type"]) != 1 or int(state["FilterState_Index"]) != index:
      continue
    if int(state["FilterState_Active"]) != active:
      raise RuntimeError(f"Filter index {index} active-state mismatch: {state}")
    if index == 0 and int(state["FilterState_Max_NofObj"]) != ARS408_MAX_OBJECTS:
      raise RuntimeError(f"Filter index 0 object-count mismatch: {state}")
    return state
  raise RuntimeError(f"No verified FilterState_Cfg reply for object filter index {index}")


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--execute", action="store_true")
  parser.add_argument("--confirm", default="")
  args = parser.parse_args()

  if Path("/data/params/d/IsOffroad").read_text().strip() != "1":
    raise RuntimeError("Refusing commissioning: device must be offroad")
  if SUCCESS_MARKER.exists():
    raise RuntimeError(f"Refusing commissioning: success marker already exists at {SUCCESS_MARKER}")
  if not args.execute:
    print("DRY RUN: would persist Extended=1, configure object count 32, and disable object filters 1..14")
    return 0
  if args.confirm != CONFIRMATION:
    raise RuntimeError(f"Refusing commissioning: --confirm must equal {CONFIRMATION}")

  can = ARS408CAN()
  radar_config = can.create_radar_configuration(store_in_nvm=True)
  if (radar_config[0] != ARS408_RADAR_CONFIG_ADDRESS or radar_config[2] != ARS408_BUS or
      len(radar_config[1]) != 8 or not ARS408_SEND_EXTENDED):
    raise RuntimeError("Refusing commissioning: generated RadarCfg address/bus/DLC or Extended setting is invalid")
  messages = (can.create_object_count_filter(),) + can.create_disabled_object_filters()
  expected_payloads = [bytes.fromhex("8600000020")] + [bytes((0x82 | (index << 3), 0, 0, 0, 0)) for index in range(1, 15)]
  if [bytes(data) for address, data, bus in messages] != expected_payloads:
    raise RuntimeError("Refusing commissioning: generated FilterCfg payloads do not match reviewed bytes")
  if any(address != ARS408_FILTER_CONFIG_ADDRESS or bus != ARS408_BUS or len(data) != 5
         for address, data, bus in messages):
    raise RuntimeError("Refusing commissioning: generated FilterCfg address/bus/DLC mismatch")

  report = {
    "test": "ARS408 one-time Extended and 32-object commissioning",
    "created_utc": datetime.now(timezone.utc).isoformat(),
    "sensor_id": ARS408_SENSOR_ID,
    "bus": ARS408_BUS,
    "filter_address": f"0x{ARS408_FILTER_CONFIG_ADDRESS:03X}",
    "radar_config_address": f"0x{ARS408_RADAR_CONFIG_ADDRESS:03X}",
    "radar_config_payload": bytes(radar_config[1]).hex(),
    "send_extended": ARS408_SEND_EXTENDED,
    "max_objects": ARS408_MAX_OBJECTS,
    "payloads": [data.hex() for data in expected_payloads],
    "executed": False,
  }

  with Panda(cli=False) as panda:
    health = panda.health()
    if health["ignition_line"] or health["ignition_can"] or health["controls_allowed"]:
      raise RuntimeError(f"Refusing commissioning: ignition/controls must be off: {health}")
    panda.can_clear(0xFFFF)
    radar_state, bus_counts = verify_dedicated_radar_bus(panda)
    report["before_radar_state"] = radar_state
    report["before_bus_counts"] = bus_counts

    verified_filters = []
    try:
      panda.set_safety_mode(CarParams.SafetyModel.allOutput)
      panda.can_send(*radar_config)
      report["after_radar_state"] = wait_for_extended_state(panda)
      for index, (address, data, bus) in enumerate(messages):
        panda.can_send(address, data, bus)
        state = wait_for_filter_state(panda, index=index, active=1 if index == 0 else 0)
        verified_filters.append({"index": index, "state": state})
        time.sleep(0.1)
      report["executed"] = True
    finally:
      panda.set_safety_mode(CarParams.SafetyModel.noOutput)

    health_after = panda.health()
    if int(health_after["safety_mode"]) != int(CarParams.SafetyModel.noOutput):
      raise RuntimeError(f"Panda did not return to noOutput: {health_after}")
    report["after_health"] = health_after
    report["verified_filters"] = verified_filters

  LOG_DIR.mkdir(parents=True, exist_ok=True)
  SUCCESS_MARKER.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
  print(json.dumps(report, indent=2, sort_keys=True))
  print(f"VERIFIED: {SUCCESS_MARKER}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
