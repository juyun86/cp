#!/usr/bin/env python3
"""Read-only ARS408 namespace collision scanner for the Tesla vehicle bus."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from cereal import messaging


BASE_MESSAGES = {
  0x200: "RadarCfg",
  0x201: "RadarState",
  0x202: "FilterCfg",
  0x203: "FilterStateHeader",
  0x204: "FilterStateCfg",
  0x205: "PolygonFilterCfg",
  0x206: "PolygonFilterState",
  0x300: "SpeedInformation",
  0x301: "YawRateInformation",
  0x400: "CollisionCfg",
  0x401: "CollisionRegionCfg",
  0x402: "CollisionRegionState",
  0x408: "CollisionState",
  0x600: "ClusterStatus",
  0x60A: "ObjectStatus",
  0x60B: "ObjectGeneral",
  0x60C: "ObjectQuality",
  0x60D: "ObjectExtended",
  0x60E: "ObjectWarning",
  0x700: "Version",
  0x701: "ClusterGeneral",
  0x702: "ClusterQuality",
}


def namespace(sensor_id: int) -> dict[int, str]:
  offset = sensor_id << 4
  return {address + offset: name for address, name in BASE_MESSAGES.items()}


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--duration", type=float, default=90.0)
  parser.add_argument("--bus", type=int, default=1)
  parser.add_argument("--label", default="powered_stationary")
  parser.add_argument("--output-dir", type=Path, default=Path("/data/media/0/ars408_diagnostics"))
  args = parser.parse_args()

  if Path("/data/params/d/IsOffroad").read_text().strip() == "1":
    raise SystemExit("Refusing scan: IsOffroad=1, boardd CAN stream is not active")

  all_namespaces = {sensor_id: namespace(sensor_id) for sensor_id in range(8)}
  watched = set().union(*(messages.keys() for messages in all_namespaces.values()))
  counts: Counter[tuple[int, int, int]] = Counter()
  payloads: dict[tuple[int, int, int], Counter[str]] = defaultdict(Counter)
  first_seen: dict[tuple[int, int, int], float] = {}
  last_seen: dict[tuple[int, int, int], float] = {}

  sock = messaging.sub_sock("can", timeout=1000)
  started_utc = datetime.now(timezone.utc)
  started_mono = time.monotonic()
  deadline = started_mono + args.duration

  while time.monotonic() < deadline:
    event = messaging.recv_one_or_none(sock)
    if event is None:
      continue
    now = time.monotonic() - started_mono
    for frame in event.can:
      address = int(frame.address)
      bus = int(frame.src)
      if bus != args.bus or address not in watched:
        continue
      data = bytes(frame.dat)
      key = (bus, address, len(data))
      counts[key] += 1
      payloads[key][data.hex()] += 1
      first_seen.setdefault(key, now)
      last_seen[key] = now

  elapsed = time.monotonic() - started_mono
  frames = []
  for (bus, address, dlc), count in sorted(counts.items()):
    frames.append({
      "bus": bus,
      "address": f"0x{address:03X}",
      "dlc": dlc,
      "count": count,
      "frequency_hz": round(count / elapsed, 3),
      "first_seen_s": round(first_seen[(bus, address, dlc)], 3),
      "last_seen_s": round(last_seen[(bus, address, dlc)], 3),
      "payloads": dict(payloads[(bus, address, dlc)].most_common(64)),
    })

  candidates = {}
  for sensor_id, messages in all_namespaces.items():
    occupied = []
    for address, name in messages.items():
      matches = [frame for frame in frames if int(frame["address"], 16) == address]
      if matches:
        occupied.append({"address": f"0x{address:03X}", "name": name, "observations": matches})
    candidates[str(sensor_id)] = {
      "collision_free_in_this_capture": not occupied,
      "occupied": occupied,
    }

  result = {
    "test": "ARS408 read-only namespace collision scan",
    "label": args.label,
    "started_utc": started_utc.isoformat(),
    "duration_s": round(elapsed, 3),
    "bus": args.bus,
    "frames": frames,
    "candidates": candidates,
  }

  args.output_dir.mkdir(parents=True, exist_ok=True)
  stamp = started_utc.strftime("%Y%m%dT%H%M%SZ")
  output = args.output_dir / f"ars408_id_scan_{stamp}_{args.label}.json"
  output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
  print(output)
  for sensor_id, candidate in candidates.items():
    occupied_ids = [entry["address"] for entry in candidate["occupied"]]
    print(f"sensor_id={sensor_id} collision_free={not occupied_ids} occupied={occupied_ids}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
