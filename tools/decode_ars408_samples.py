#!/usr/bin/env python3
import math
import time

from opendbc.can import CANParser


def decode(message, address, payload):
  parser = CANParser("ARS408", [(message, math.nan)], 1)
  parser.update([(time.monotonic_ns(), [(address, bytes.fromhex(payload), 1)])])
  print(message, payload, parser.vl[message])


decode("FilterCfg", 0x202, "8a00b109b4040000")
decode("FilterCfg", 0x202, "8a00b209b4040000")
decode("FilterCfg", 0x202, "8a00b309b5040000")
decode("RadarState", 0x201, "c01f400013940000")
decode("RadarState", 0x201, "c01f400013d40000")
decode("RadarState", 0x201, "60555515545111f8")
decode("RadarState", 0x201, "6105555501050039")
decode("FilterCfg", 0x202, "5904000081")
decode("FilterCfg", 0x202, "f423a14b00")
decode("FilterCfg", 0x202, "f423a24b00")
decode("FilterCfg", 0x202, "0380630000")
decode("FilterCfg", 0x202, "060016b108")
