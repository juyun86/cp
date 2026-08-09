#!/usr/bin/env python3
"""Read live vehicle state and fail unless the car is safely parked."""

import json

from cereal import messaging


def main() -> int:
  sm = messaging.SubMaster(["carState", "pandaStates"])
  for _ in range(50):
    sm.update(100)
    if sm.alive["carState"] and sm.valid["carState"] and sm.alive["pandaStates"] and sm.valid["pandaStates"]:
      break

  car_state = sm["carState"]
  pandas = list(sm["pandaStates"])
  report = {
    "carStateAlive": bool(sm.alive["carState"]),
    "carStateValid": bool(sm.valid["carState"]),
    "pandaStatesAlive": bool(sm.alive["pandaStates"]),
    "pandaStatesValid": bool(sm.valid["pandaStates"]),
    "gear": str(car_state.gearShifter),
    "vEgo": float(car_state.vEgo),
    "standstill": bool(car_state.standstill),
    "controlsAllowed": [bool(panda.controlsAllowed) for panda in pandas],
    "ignitionLine": [bool(panda.ignitionLine) for panda in pandas],
    "ignitionCan": [bool(panda.ignitionCan) for panda in pandas],
  }
  print(json.dumps(report, indent=2, sort_keys=True))

  # Tesla's standstill field follows the cruise state, not the physical Park
  # state, and DI_speed has a small zero offset on this vehicle. Park plus a
  # sub-0.2 m/s reading is the independently observable stationary condition.
  safe = (report["carStateAlive"] and report["carStateValid"] and
          report["pandaStatesAlive"] and report["pandaStatesValid"] and
          report["gear"] == "park" and abs(report["vEgo"]) < 0.2)
  if not safe:
    raise RuntimeError(f"Vehicle is not verified parked: {report}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
