from collections import Counter
import logging
import math

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.tesla.ars408_can import (
  ARS408_BUS, ARS408_MAX_DISTANCE, ARS408_SEND_EXTENDED, ARS408_SENSOR_ID,
)


ARS408_DBC = "ARS408"
ARS408_ADDRESS_OFFSET = ARS408_SENSOR_ID << 4

# The DBC describes SensorID 0. Incoming frames from the runtime-selected
# sensor namespace are validated at their shifted addresses, then mapped back
# to the SensorID 0 addresses used for DBC decoding.
ARS408_STATUS = 0x60A + ARS408_ADDRESS_OFFSET
ARS408_GENERAL = 0x60B + ARS408_ADDRESS_OFFSET
ARS408_QUALITY = 0x60C + ARS408_ADDRESS_OFFSET
ARS408_EXTENDED = 0x60D + ARS408_ADDRESS_OFFSET
ARS408_RADAR_STATE = 0x201 + ARS408_ADDRESS_OFFSET
ARS408_MESSAGES = {
  ARS408_STATUS: (0x60A, "Obj_0_Status", 4),
  ARS408_GENERAL: (0x60B, "Obj_1_General", 8),
  ARS408_QUALITY: (0x60C, "Obj_2_Quality", 7),
  ARS408_EXTENDED: (0x60D, "Obj_3_Extended", 8),
}
ARS408_PROTOCOL_MAX_OBJECTS = 100
ARS408_OBJECT_CORRIDOR = 5.5
ARS408_MIN_PROBABILITY = 3       # >= 75%
ARS408_MIN_TRACKED_PROBABILITY = 2  # >= 50%, Toyota-style hysteresis for an existing track
ARS408_MIN_STATIC_PROBABILITY = 5  # >= 99%
ARS408_TRACK_GRACE_CYCLES = 2
LOG_EVERY_CYCLES = 20
ZERO_OBJECT_WARNING_CYCLES = 20
DIAGNOSTIC_LOG_EVERY_CYCLES = 200
MEASUREMENT_COUNTER_STALL_WARNING_CYCLES = 20
ARS408_STARTUP_GRACE_UPDATES = 300
ARS408_STATUS_TIMEOUT_UPDATES = 100
ARS408_STATE_TIMEOUT_UPDATES = 300
ARS408_ERROR_PUBLISH_INTERVAL = 5
# RadarState is nominally 1 Hz. Production never reconfigures the sensor, so a
# short grace period is enough to distinguish startup from persistent mismatch.
CONFIG_GRACE_STATE_FRAMES = 3

log = logging.getLogger(__name__)


def object_rejection_reason(obj, previously_tracked=False):
  measurement_state = int(obj["Obj_MeasState"])
  probability = int(obj["Obj_ProbOfExist"])
  d_rel = float(obj["Obj_DistLong"])
  y_rel = -float(obj["Obj_DistLat"])
  v_rel = float(obj["Obj_VrelLong"])
  yv_rel = float(obj["Obj_VrelLat"])
  dynamic_property = int(obj["Obj_DynProp"])

  min_probability = ARS408_MIN_TRACKED_PROBABILITY if previously_tracked else ARS408_MIN_PROBABILITY
  if measurement_state in (0, 4):
    return "invalid_measurement_state"
  if probability < min_probability:
    return "low_probability"
  # A predicted-only target must first have been observed by the interface.
  if measurement_state == 3 and not previously_tracked:
    return "predicted_new_target"
  if not (0.0 <= d_rel <= ARS408_MAX_DISTANCE and abs(y_rel) <= 100.0 and
          -100.0 <= v_rel <= 100.0 and abs(yv_rel) <= 60.0):
    return "out_of_bounds"

  # ARS408 reports roadside infrastructure as stationary/candidate/crossing
  # stationary. Keep stopped targets (7), and only keep never-moving static
  # targets when they are high-confidence and within the current/adjacent
  # three-lane corridor.
  if dynamic_property in (1, 3, 5):
    if probability < ARS408_MIN_STATIC_PROBABILITY:
      return "static_low_probability"
    if abs(y_rel) > ARS408_OBJECT_CORRIDOR:
      return "static_outside_corridor"
  return None


def object_is_usable(obj, previously_tracked=False):
  return object_rejection_reason(obj, previously_tracked) is None


def get_radar_can_parser(CP):
  messages = [
    ("Obj_0_Status", 14),
    ("Obj_1_General", math.nan),
    ("Obj_2_Quality", math.nan),
    ("Obj_3_Extended", math.nan),
    ("RadarState", math.nan),
  ]
  return CANParser(ARS408_DBC, messages, ARS408_BUS)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.rcp = None if CP.radarUnavailable else get_radar_can_parser(CP)
    self.trigger_msg = ARS408_STATUS
    self.cycle_started = False
    self.expected_objects = 0
    self.cycle_frames = []
    self.part_ids = {address: set() for address in (ARS408_GENERAL, ARS408_QUALITY, ARS408_EXTENDED)}
    self.part_counts = {address: 0 for address in self.part_ids}
    self.cycle_invalid = False
    self.incomplete_cycles = 0
    self.last_logged_incomplete = 0
    self.last_radar_state = None
    self.last_fault_signature = None
    self.radar_state_frames = 0
    self.track_miss_counts = {}
    self.total_cycles = 0
    self.zero_object_cycles = 0
    self.last_measurement_counter = None
    self.measurement_counter_stalls = 0
    self.last_can_valid = None
    self.update_count = 0
    self.last_status_update = None
    self.last_radar_state_update = None
    self.last_missing_can_signature = None
    self.diagnostic_cycle_count = 0
    self.diagnostic_raw_objects = 0
    self.diagnostic_accepted_objects = 0
    self.diagnostic_rejections = Counter()

  def _start_cycle(self):
    status = self.rcp.vl["Obj_0_Status"]
    self.expected_objects = int(status["Obj_NofObjects"])
    measurement_counter = int(status["Obj_MeasCounter"])
    self.total_cycles += 1

    if self.last_measurement_counter == measurement_counter:
      self.measurement_counter_stalls += 1
      if self.measurement_counter_stalls == MEASUREMENT_COUNTER_STALL_WARNING_CYCLES or \
         self.measurement_counter_stalls % DIAGNOSTIC_LOG_EVERY_CYCLES == 0:
        log.warning("ARS408 measurement counter stalled: counter=%d consecutive=%d cycles=%d can_valid=%d",
                    measurement_counter, self.measurement_counter_stalls, self.total_cycles, int(self.rcp.can_valid))
    else:
      if self.measurement_counter_stalls >= MEASUREMENT_COUNTER_STALL_WARNING_CYCLES:
        log.warning("ARS408 measurement counter recovered: previous_counter=%d stalled_cycles=%d new_counter=%d",
                    self.last_measurement_counter, self.measurement_counter_stalls, measurement_counter)
      self.measurement_counter_stalls = 0
    self.last_measurement_counter = measurement_counter

    if self.expected_objects == 0:
      self.zero_object_cycles += 1
      if self.zero_object_cycles == ZERO_OBJECT_WARNING_CYCLES or \
         (self.zero_object_cycles > ZERO_OBJECT_WARNING_CYCLES and
          (self.zero_object_cycles - ZERO_OBJECT_WARNING_CYCLES) % DIAGNOSTIC_LOG_EVERY_CYCLES == 0):
        state = self.last_radar_state or {}
        log.warning("ARS408 zero object output: consecutive=%d cycles=%d meas_counter=%d can_valid=%d " +
                    "sensor_id=%d output_type=%d motion_rx=%d nvm_read=%d nvm_write=%d",
                    self.zero_object_cycles, self.total_cycles, measurement_counter, int(self.rcp.can_valid),
                    int(state.get("RadarState_SensorID", -1)), int(state.get("RadarState_OutputTypeCfg", -1)),
                    int(state.get("RadarState_MotionRxState", -1)), int(state.get("RadarState_NVMReadStatus", -1)),
                    int(state.get("RadarState_NVMwriteStatus", -1)))
    else:
      if self.zero_object_cycles >= ZERO_OBJECT_WARNING_CYCLES:
        log.warning("ARS408 object output recovered: zero_cycles=%d current_objects=%d meas_counter=%d cycles=%d",
                    self.zero_object_cycles, self.expected_objects, measurement_counter, self.total_cycles)
      self.zero_object_cycles = 0
    self.cycle_frames = []
    self.part_ids = {address: set() for address in (ARS408_GENERAL, ARS408_QUALITY, ARS408_EXTENDED)}
    self.part_counts = {address: 0 for address in self.part_ids}
    self.cycle_invalid = self.expected_objects > ARS408_PROTOCOL_MAX_OBJECTS or int(status["Obj_InterfaceVersion"]) != 1
    self.cycle_started = True

  def _add_detail_frame(self, address, frame, object_id):
    self.cycle_frames.append(frame)
    self.part_counts[address] += 1
    self.part_ids[address].add(object_id)

  def _cycle_complete(self):
    if self.cycle_invalid:
      return False

    expected = self.expected_objects
    general_ids = self.part_ids[ARS408_GENERAL]
    quality_ids = self.part_ids[ARS408_QUALITY]
    return len(general_ids) == expected and \
           len(quality_ids) == expected and \
           self.part_counts[ARS408_GENERAL] == expected and \
           self.part_counts[ARS408_QUALITY] == expected and \
           general_ids == quality_ids

  def _extended_complete(self):
    expected = self.expected_objects
    extended_ids = self.part_ids[ARS408_EXTENDED]
    return len(extended_ids) == expected and \
           self.part_counts[ARS408_EXTENDED] == expected and \
           extended_ids == self.part_ids[ARS408_GENERAL]

  def _incomplete_result(self):
    self.incomplete_cycles += 1
    if self.incomplete_cycles == 1 or self.incomplete_cycles - self.last_logged_incomplete >= LOG_EVERY_CYCLES:
      general_ids = self.part_ids[ARS408_GENERAL]
      quality_ids = self.part_ids[ARS408_QUALITY]
      log.warning("ARS408 incomplete cycle: expected=%d general=%d quality=%d missing_general=%s missing_quality=%s consecutive=%d",
                  self.expected_objects, self.part_counts[ARS408_GENERAL], self.part_counts[ARS408_QUALITY],
                  sorted(quality_ids - general_ids), sorted(general_ids - quality_ids), self.incomplete_cycles)
      self.last_logged_incomplete = self.incomplete_cycles

    # A dropped object-list frame is not proof of a broken CAN connection.
    # Publish the last complete set so radard remains alive and diagnoses do
    # not become the generic "check connections" alert.
    ret = structs.RadarData()
    ret.points = list(self.pts.values())
    if not self.rcp.can_valid:
      ret.errors.canError = True
    self._apply_radar_state_errors(ret)
    return ret

  def _update_radar_state(self, timestamp, data, src):
    self.rcp.update([(timestamp, [(0x201, data, src)])])
    self.last_radar_state_update = self.update_count
    state = self.rcp.vl["RadarState"]
    self.last_radar_state = dict(state)
    self.radar_state_frames += 1

  def _apply_radar_state_errors(self, ret):
    state = self.last_radar_state
    if state is None:
      return

    temporary_fault = any(int(state[name]) for name in (
      "RadarState_Interference", "RadarState_Temperature_Error", "RadarState_Temporary_Error"))
    hard_fault = any(int(state[name]) for name in (
      "RadarState_Voltage_Error", "RadarState_Persistent_Error"))
    wrong_config = int(state["RadarState_SensorID"]) != ARS408_SENSOR_ID or \
                   int(state["RadarState_OutputTypeCfg"]) != 1 or \
                   not int(state["RadarState_SendQualityCfg"]) or \
                   int(state["RadarState_SendExtInfoCfg"]) != int(ARS408_SEND_EXTENDED) or \
                   int(state["RadarState_CtrlRelayCfg"]) != 0 or \
                   int(state["RadarState_SortIndex"]) != 1 or \
                   int(state["RadarState_RCS_Threshold"]) != 0 or \
                   int(state["RadarState_RadarPowerCfg"]) != 0 or \
                   int(state["RadarState_MaxDistanceCfg"]) != ARS408_MAX_DISTANCE

    ret.errors.radarUnavailableTemporary = temporary_fault
    ret.errors.radarFault = hard_fault
    # The controller configures the radar throughout its first ten seconds.
    # Reporting the power-on/default state before those attempts complete
    # invalidates liveTracks and causes an unnecessary takeover request.
    ret.errors.wrongConfig = wrong_config and self.radar_state_frames >= CONFIG_GRACE_STATE_FRAMES
    fault_signature = tuple(int(state[name]) for name in (
      "RadarState_Interference", "RadarState_Temperature_Error", "RadarState_Temporary_Error",
      "RadarState_Voltage_Error", "RadarState_Persistent_Error", "RadarState_SensorID",
      "RadarState_OutputTypeCfg", "RadarState_SendQualityCfg", "RadarState_SendExtInfoCfg",
      "RadarState_MotionRxState", "RadarState_NVMReadStatus", "RadarState_NVMwriteStatus",
      "RadarState_MaxDistanceCfg", "RadarState_RadarPowerCfg", "RadarState_SortIndex", "RadarState_RCS_Threshold",
    )) + (int(wrong_config), int(self.radar_state_frames < CONFIG_GRACE_STATE_FRAMES))
    if fault_signature != self.last_fault_signature:
      log.warning("ARS408 state diagnostic: interference=%d temperature=%d temporary=%d voltage=%d persistent=%d " +
                  "sensor_id=%d output_type=%d quality=%d extended=%d motion_rx=%d nvm_read=%d nvm_write=%d " +
                  "max_distance=%d power=%d sort=%d rcs_threshold=%d wrong_config=%d config_grace=%d",
                  int(state["RadarState_Interference"]), int(state["RadarState_Temperature_Error"]),
                  int(state["RadarState_Temporary_Error"]), int(state["RadarState_Voltage_Error"]),
                  int(state["RadarState_Persistent_Error"]), int(state["RadarState_SensorID"]),
                  int(state["RadarState_OutputTypeCfg"]), int(state["RadarState_SendQualityCfg"]),
                  int(state["RadarState_SendExtInfoCfg"]), int(state["RadarState_MotionRxState"]),
                  int(state["RadarState_NVMReadStatus"]), int(state["RadarState_NVMwriteStatus"]),
                  int(state["RadarState_MaxDistanceCfg"]), int(state["RadarState_RadarPowerCfg"]),
                  int(state["RadarState_SortIndex"]), int(state["RadarState_RCS_Threshold"]), int(wrong_config),
                  int(self.radar_state_frames < CONFIG_GRACE_STATE_FRAMES))
      self.last_fault_signature = fault_signature

  def _missing_can_signature(self):
    if self.update_count <= ARS408_STARTUP_GRACE_UPDATES:
      return None

    status_missing = self.last_status_update is None or \
                     self.update_count - self.last_status_update > ARS408_STATUS_TIMEOUT_UPDATES
    state_missing = self.last_radar_state_update is None or \
                    self.update_count - self.last_radar_state_update > ARS408_STATE_TIMEOUT_UPDATES
    if status_missing or state_missing:
      return status_missing, state_missing
    return None

  def _apply_missing_can_error(self, ret):
    missing_signature = self._missing_can_signature()
    if missing_signature != self.last_missing_can_signature:
      if missing_signature is None:
        log.info("ARS408 required CAN messages recovered")
      else:
        log.error("ARS408 required CAN messages missing: object_status=%d radar_state=%d update=%d",
                  int(missing_signature[0]), int(missing_signature[1]), self.update_count)
      self.last_missing_can_signature = missing_signature

    if missing_signature is not None:
      ret.errors.canError = True
    return missing_signature is not None

  def _decode_cycle(self, timestamp):
    self.rcp.update([(timestamp, self.cycle_frames)])
    core_decode_fields = {
      ARS408_GENERAL: (
        "Obj_ID", "Obj_DistLong", "Obj_DistLat", "Obj_VrelLong", "Obj_VrelLat", "Obj_RCS", "Obj_DynProp"),
      ARS408_QUALITY: ("Obj_ID", "Obj_ProbOfExist", "Obj_MeasState"),
    }

    decoded_parts = {}
    for address, fields in core_decode_fields.items():
      message_name = ARS408_MESSAGES[address][1]
      values = self.rcp.vl_all[message_name]
      columns = [values[field] for field in fields]
      if len({len(column) for column in columns}) != 1:
        return None

      rows = {}
      for row in zip(*columns, strict=True):
        rows[int(row[0])] = dict(zip(fields[1:], row[1:], strict=True))
      decoded_parts[address] = rows

    # A shared CAN bus can lose one lower-priority Quality frame without
    # invalidating every other target. Only merge IDs present in both core
    # messages; the caller applies a short grace period to missing IDs.
    objects = {}
    core_ids = set(decoded_parts[ARS408_GENERAL]) & set(decoded_parts[ARS408_QUALITY])
    for object_id in core_ids:
      objects[object_id] = decoded_parts[ARS408_GENERAL][object_id] | decoded_parts[ARS408_QUALITY][object_id]

    # Extended object frames have the lowest CAN priority in the ARS408
    # object list. Under high target load the radar can begin its next cycle
    # before every Extended frame is transmitted. General and Quality remain
    # sufficient for safe lead tracking, so only merge Extended data when the
    # complete set belongs to this cycle.
    if self._extended_complete():
      fields = ("Obj_ID", "Obj_ArelLong", "Obj_Class")
      values = self.rcp.vl_all[ARS408_MESSAGES[ARS408_EXTENDED][1]]
      columns = [values[field] for field in fields]
      if all(len(column) == self.expected_objects for column in columns):
        for row in zip(*columns, strict=True):
          obj = objects.get(int(row[0]))
          if obj is not None:
            obj.update(dict(zip(fields[1:], row[1:], strict=True)))

    return objects

  def _build_result(self, timestamp):
    if self.cycle_invalid:
      return self._incomplete_result()

    cycle_objects = self._decode_cycle(timestamp)
    if cycle_objects is None:
      return self._incomplete_result()

    if not self._cycle_complete():
      self.incomplete_cycles += 1
      if self.incomplete_cycles == 1 or self.incomplete_cycles - self.last_logged_incomplete >= LOG_EVERY_CYCLES:
        log.warning("ARS408 partial cycle salvaged: expected=%d general=%d quality=%d usable_pairs=%d consecutive=%d",
                    self.expected_objects, self.part_counts[ARS408_GENERAL], self.part_counts[ARS408_QUALITY],
                    len(cycle_objects), self.incomplete_cycles)
        self.last_logged_incomplete = self.incomplete_cycles
    else:
      self.incomplete_cycles = 0
      self.last_logged_incomplete = 0
    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True
    can_valid = bool(self.rcp.can_valid)
    if self.last_can_valid is None or can_valid != self.last_can_valid:
      log.log(logging.INFO if can_valid else logging.WARNING,
              "ARS408 CAN parser state changed: can_valid=%d cycles=%d incomplete_consecutive=%d",
              int(can_valid), self.total_cycles, self.incomplete_cycles)
      self.last_can_valid = can_valid
    self._apply_radar_state_errors(ret)

    current_targets = set()
    cycle_rejections = Counter()
    for object_id, obj in cycle_objects.items():
      d_rel = float(obj["Obj_DistLong"])
      y_rel = -float(obj["Obj_DistLat"])
      v_rel = float(obj["Obj_VrelLong"])
      yv_rel = float(obj["Obj_VrelLat"])

      rejection_reason = object_rejection_reason(obj, object_id in self.pts)
      if rejection_reason is not None:
        cycle_rejections[rejection_reason] += 1
        continue

      current_targets.add(object_id)
      self.track_miss_counts[object_id] = 0
      if object_id not in self.pts:
        self.pts[object_id] = structs.RadarData.RadarPoint()
        self.pts[object_id].trackId = object_id

      point = self.pts[object_id]
      point.dRel = d_rel
      point.yRel = y_rel
      point.vRel = v_rel
      point.vLead = point.vRel + self.v_ego
      # Missing Extended data must never reuse acceleration from an older
      # cycle. A neutral value lets the downstream tracker estimate motion
      # from successive General frames without invalidating the radar stream.
      point.aRel = float(obj.get("Obj_ArelLong", 0.0))
      point.yvRel = yv_rel
      point.measured = int(obj["Obj_MeasState"]) in (1, 2, 5)

    for object_id in list(self.pts):
      if object_id not in current_targets:
        missed_cycles = self.track_miss_counts.get(object_id, 0) + 1
        self.track_miss_counts[object_id] = missed_cycles
        if missed_cycles <= ARS408_TRACK_GRACE_CYCLES:
          # Mirror Toyota's valid-count hysteresis: a one-frame confidence or
          # list-size fluctuation must not destroy and recreate a real track.
          self.pts[object_id].measured = False
        else:
          del self.pts[object_id]
          self.track_miss_counts.pop(object_id, None)

    ret.points = list(self.pts.values())
    self.diagnostic_cycle_count += 1
    self.diagnostic_raw_objects += len(cycle_objects)
    self.diagnostic_accepted_objects += len(current_targets)
    self.diagnostic_rejections.update(cycle_rejections)
    if self.diagnostic_cycle_count >= DIAGNOSTIC_LOG_EVERY_CYCLES:
      log.info("ARS408 cycle summary: cycles=%d expected_last=%d paired_raw=%d accepted=%d tracked=%d " +
               "rejected=%s incomplete_consecutive=%d can_valid=%d",
               self.diagnostic_cycle_count, self.expected_objects, self.diagnostic_raw_objects,
               self.diagnostic_accepted_objects, len(self.pts), dict(self.diagnostic_rejections),
               self.incomplete_cycles, int(self.rcp.can_valid))
      self.diagnostic_cycle_count = 0
      self.diagnostic_raw_objects = 0
      self.diagnostic_accepted_objects = 0
      self.diagnostic_rejections.clear()
    return ret

  def update(self, can_packets):
    if self.rcp is None:
      return super().update(None)

    self.update_count += 1
    result = None
    for timestamp, frames in can_packets:
      for address, data, src in frames:
        if src == ARS408_BUS and address == ARS408_RADAR_STATE and len(data) == 8:
          self._update_radar_state(timestamp, data, src)
          continue
        if src != ARS408_BUS or address not in ARS408_MESSAGES:
          continue

        base_address, _message_name, expected_dlc = ARS408_MESSAGES[address]
        if len(data) != expected_dlc:
          if self.cycle_started and address != ARS408_EXTENDED:
            self.cycle_invalid = True
          continue

        # CANParser uses the unshifted DBC addresses. Only frames that passed
        # the runtime-selected SensorID address and exact DLC checks reach the decoder.
        frame = (base_address, data, src)
        if address == ARS408_STATUS:
          self.last_status_update = self.update_count
          if self.cycle_started:
            cycle_result = self._build_result(timestamp)
            if cycle_result is not None:
              result = cycle_result
          self.rcp.update([(timestamp, [frame])])
          self._start_cycle()
          continue

        if not self.cycle_started:
          continue

        self._add_detail_frame(address, frame, data[0])

    if result is not None:
      self._apply_missing_can_error(result)
      return result

    missing_result = structs.RadarData()
    if self._apply_missing_can_error(missing_result) and self.update_count % ARS408_ERROR_PUBLISH_INTERVAL == 0:
      return missing_result
    return None
