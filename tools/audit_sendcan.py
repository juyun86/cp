#!/usr/bin/env python3
from collections import Counter
from pprint import pprint
import time

from cereal import messaging


def panda_state():
  event = messaging.recv_one_retry(messaging.sub_sock("pandaStates", timeout=3000))
  return [state.to_dict() for state in event.pandaStates]


sock = messaging.sub_sock("sendcan", timeout=500)
before = panda_state()
counts = Counter()
deadline = time.monotonic() + 10.0
while time.monotonic() < deadline:
  event = messaging.recv_one_or_none(sock)
  if event is None:
    continue
  for frame in event.sendcan:
    counts[(int(frame.address), int(frame.src), len(frame.dat))] += 1
after = panda_state()
pprint({
  "requests": [
    {"address": f"0x{address:03X}", "bus": bus, "dlc": dlc, "count": count}
    for (address, bus, dlc), count in sorted(counts.items())
  ],
  "before": before,
  "after": after,
})
