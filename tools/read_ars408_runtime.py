#!/usr/bin/env python3
"""Print a compact, read-only snapshot of the live ARS408 control path."""

import json
import math
import time

from cereal import messaging
from opendbc.can import CANParser
from opendbc.car.tesla.ars408_can import ARS408_BUS, ARS408_SENSOR_ID


RADAR_STATE_ADDRESS = 0x201 + (ARS408_SENSOR_ID << 4)


SERVICES = ["carState", "liveTracks", "radarState", "pandaStates"]
sm = messaging.SubMaster(SERVICES)
deadline = time.monotonic() + 5.0
while time.monotonic() < deadline and not all(sm.valid[service] for service in SERVICES):
  sm.update(500)

ars408_state = None
can_sock = messaging.sub_sock("can", timeout=500)
deadline = time.monotonic() + 3.0
while time.monotonic() < deadline and ars408_state is None:
  event = messaging.recv_one_or_none(can_sock)
  if event is None:
    continue
  for frame in event.can:
    if int(frame.src) == ARS408_BUS and int(frame.address) == RADAR_STATE_ADDRESS and len(frame.dat) == 8:
      parser = CANParser("ARS408", [("RadarState", math.nan)], ARS408_BUS)
      parser.update([(time.monotonic_ns(), [(0x201, bytes(frame.dat), ARS408_BUS)])])
      ars408_state = {"raw": bytes(frame.dat).hex(), "decoded": dict(parser.vl["RadarState"])}
      break

pandas = sm["pandaStates"]
report = {
  "valid": {service: bool(sm.valid[service]) for service in SERVICES},
  "alive": {service: bool(sm.alive[service]) for service in SERVICES},
  "car": {
    "vEgo": float(sm["carState"].vEgo),
    "steeringAngleDeg": float(sm["carState"].steeringAngleDeg),
    "gearShifter": str(sm["carState"].gearShifter),
    "standstill": bool(sm["carState"].standstill),
  },
  "ars408State": ars408_state,
  "liveTracks": sm["liveTracks"].to_dict(),
  "radarState": sm["radarState"].to_dict(),
  "panda": [{
    "safetyModel": str(state.safetyModel),
    "controlsAllowed": bool(state.controlsAllowed),
    "safetyTxBlocked": int(state.safetyTxBlocked),
    "bus1": state.canState1.to_dict(),
  } for state in pandas],
}
print(json.dumps(report, indent=2, sort_keys=True))
