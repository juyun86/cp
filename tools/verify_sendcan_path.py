#!/usr/bin/env python3
from pprint import pprint
import time

from cereal import messaging


def panda_state():
  event = messaging.recv_one_retry(messaging.sub_sock("pandaStates", timeout=3000))
  return [state.to_dict() for state in event.pandaStates]


before = panda_state()
can_sock = messaging.sub_sock("can", timeout=100)
pub = messaging.PubMaster(["sendcan"])
msg = messaging.new_message("sendcan", 1)
msg.sendcan[0].address = 0x330
msg.sendcan[0].dat = b"\x00\x00"
msg.sendcan[0].src = 1
pub.send("sendcan", msg)

observed = []
deadline = time.monotonic() + 1.0
while time.monotonic() < deadline:
  event = messaging.recv_one_or_none(can_sock)
  if event is None:
    continue
  for frame in event.can:
    if int(frame.address) == 0x330:
      observed.append({"src": int(frame.src), "data": bytes(frame.dat).hex()})

after = panda_state()
pprint({"before": before, "observed_0x330": observed, "after": after})
