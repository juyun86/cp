#!/usr/bin/env python3
from pprint import pprint

from cereal import messaging


event = messaging.recv_one_retry(messaging.sub_sock("pandaStates", timeout=3000))
pprint([state.to_dict() for state in event.pandaStates])
