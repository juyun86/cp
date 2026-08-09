from collections import Counter
import math

from opendbc.car import structs
from opendbc.car.interfaces import MyTrack
from opendbc.car.tesla.radar_interface import (
  ARS408_EXTENDED, ARS408_GENERAL, ARS408_MAX_DISTANCE, ARS408_QUALITY, ARS408_SEND_EXTENDED,
  ARS408_SENSOR_ID, ARS408_STARTUP_GRACE_UPDATES, ARS408_TRACK_GRACE_CYCLES,
  CONFIG_GRACE_STATE_FRAMES, RadarInterface,
  object_is_usable, object_rejection_reason,
)


def object_data(**overrides):
  obj = {
    "Obj_MeasState": 2,
    "Obj_ProbOfExist": 5,
    "Obj_DistLong": 40.0,
    "Obj_DistLat": 0.0,
    "Obj_VrelLong": 0.0,
    "Obj_VrelLat": 0.0,
    "Obj_DynProp": 0,
  }
  obj.update(overrides)
  return obj


def test_rejects_low_probability_and_new_predicted_targets():
  assert not object_is_usable(object_data(Obj_ProbOfExist=2))
  assert object_is_usable(object_data(Obj_ProbOfExist=2), previously_tracked=True)
  assert not object_is_usable(object_data(Obj_MeasState=3), previously_tracked=False)
  assert object_is_usable(object_data(Obj_MeasState=3), previously_tracked=True)


def test_filters_roadside_static_objects_but_keeps_adjacent_lane_and_stopped_targets():
  assert not object_is_usable(object_data(Obj_DynProp=1, Obj_DistLat=8.0, Obj_ProbOfExist=7))
  assert object_is_usable(object_data(Obj_DynProp=1, Obj_DistLat=3.7, Obj_ProbOfExist=5))
  assert object_is_usable(object_data(Obj_DynProp=7, Obj_DistLat=3.7, Obj_ProbOfExist=3))


def test_object_rejection_reasons_are_diagnostic_and_stable():
  assert object_rejection_reason(object_data(Obj_MeasState=0)) == "invalid_measurement_state"
  assert object_rejection_reason(object_data(Obj_ProbOfExist=2)) == "low_probability"
  assert object_rejection_reason(object_data(Obj_MeasState=3)) == "predicted_new_target"
  assert object_rejection_reason(object_data(Obj_DistLong=-1.0)) == "out_of_bounds"
  assert object_rejection_reason(object_data(Obj_DynProp=1, Obj_ProbOfExist=4)) == "static_low_probability"
  assert object_rejection_reason(object_data(Obj_DynProp=1, Obj_DistLat=8.0)) == "static_outside_corridor"
  assert object_rejection_reason(object_data()) is None


def test_incomplete_object_cycle_is_not_reported_as_can_disconnect():
  radar = RadarInterface.__new__(RadarInterface)
  radar.incomplete_cycles = 0
  radar.last_logged_incomplete = 0
  radar.expected_objects = 2
  radar.part_counts = {ARS408_GENERAL: 1, ARS408_QUALITY: 0}
  radar.part_ids = {ARS408_GENERAL: {1}, ARS408_QUALITY: set()}
  radar.pts = {}
  radar.last_radar_state = None
  radar.rcp = type("FakeParser", (), {"can_valid": True})()

  result = radar._incomplete_result()

  assert not result.errors.canError
  assert radar.incomplete_cycles == 1


def test_disabled_radar_publishes_valid_empty_data():
  CP = structs.CarParams()
  CP.radarUnavailable = True
  radar = RadarInterface(CP)

  results = [radar.update([]) for _ in range(5)]

  assert all(result is None for result in results[:-1])
  assert list(results[-1].points) == []
  assert not results[-1].errors.canError


def test_enabled_radar_requires_ars408_can_after_startup_grace():
  CP = structs.CarParams()
  CP.radarUnavailable = False
  radar = RadarInterface(CP)

  result = None
  for _ in range(ARS408_STARTUP_GRACE_UPDATES + 5):
    result = radar.update([])

  assert result is not None
  assert result.errors.canError


def test_wrong_config_is_reported_after_boot_grace():
  radar = RadarInterface.__new__(RadarInterface)
  radar.last_radar_state = {
    "RadarState_Interference": 0,
    "RadarState_Temperature_Error": 0,
    "RadarState_Temporary_Error": 0,
    "RadarState_Voltage_Error": 0,
    "RadarState_Persistent_Error": 0,
    "RadarState_SensorID": (ARS408_SENSOR_ID + 1) & 0x7,
    "RadarState_OutputTypeCfg": 1,
    "RadarState_SendQualityCfg": 1,
    "RadarState_SendExtInfoCfg": int(ARS408_SEND_EXTENDED),
    "RadarState_CtrlRelayCfg": 0,
    "RadarState_SortIndex": 1,
    "RadarState_RCS_Threshold": 0,
    "RadarState_RadarPowerCfg": 0,
    "RadarState_MaxDistanceCfg": ARS408_MAX_DISTANCE,
    "RadarState_MotionRxState": 0,
    "RadarState_NVMReadStatus": 1,
    "RadarState_NVMwriteStatus": 0,
  }
  radar.last_fault_signature = None

  result = structs.RadarData()
  radar.radar_state_frames = CONFIG_GRACE_STATE_FRAMES - 1
  radar._apply_radar_state_errors(result)
  assert not result.errors.wrongConfig

  result = structs.RadarData()
  radar.radar_state_frames = CONFIG_GRACE_STATE_FRAMES
  radar._apply_radar_state_errors(result)
  assert result.errors.wrongConfig


def make_result_radar(points=None):
  radar = RadarInterface.__new__(RadarInterface)
  radar.expected_objects = 0
  radar.cycle_invalid = False
  radar.part_counts = {ARS408_GENERAL: 0, ARS408_QUALITY: 0}
  radar.part_ids = {ARS408_GENERAL: set(), ARS408_QUALITY: set()}
  radar.pts = points or {}
  radar.track_miss_counts = {track_id: 0 for track_id in radar.pts}
  radar.incomplete_cycles = 0
  radar.last_logged_incomplete = 0
  radar.last_radar_state = None
  radar.total_cycles = 0
  radar.v_ego = 0.0
  radar.last_can_valid = None
  radar.diagnostic_cycle_count = 0
  radar.diagnostic_raw_objects = 0
  radar.diagnostic_accepted_objects = 0
  radar.diagnostic_rejections = Counter()
  radar.rcp = type("FakeParser", (), {"can_valid": True})()
  radar._decode_cycle = lambda _timestamp: {}
  return radar


def test_zero_object_cycles_publish_empty_data_without_can_error():
  radar = make_result_radar()

  result = radar._build_result(1_000_000_000)

  assert list(result.points) == []
  assert not result.errors.canError


def test_existing_track_survives_brief_empty_cycle_then_expires():
  point = structs.RadarData.RadarPoint()
  point.trackId = 7
  point.dRel = 35.0
  point.measured = True
  radar = make_result_radar({7: point})

  for _ in range(ARS408_TRACK_GRACE_CYCLES):
    result = radar._build_result(1_000_000_000)
    assert [pt.trackId for pt in result.points] == [7]
    assert not result.points[0].measured

  result = radar._build_result(1_000_000_000)
  assert list(result.points) == []


def test_partial_cycle_keeps_objects_with_both_general_and_quality_frames():
  radar = RadarInterface.__new__(RadarInterface)
  radar.expected_objects = 2
  radar.cycle_frames = []
  radar.part_ids = {ARS408_GENERAL: {1, 2}, ARS408_QUALITY: {1}, ARS408_EXTENDED: set()}
  radar.part_counts = {ARS408_GENERAL: 2, ARS408_QUALITY: 1, ARS408_EXTENDED: 0}
  radar.rcp = type("FakeParser", (), {
    "update": lambda *_args: None,
    "vl_all": {
      "Obj_1_General": {
        "Obj_ID": [1, 2], "Obj_DistLong": [30.0, 60.0], "Obj_DistLat": [0.0, 3.5],
        "Obj_VrelLong": [0.0, 0.0], "Obj_VrelLat": [0.0, 0.0], "Obj_RCS": [10.0, 10.0],
        "Obj_DynProp": [0, 0],
      },
      "Obj_2_Quality": {
        "Obj_ID": [1], "Obj_ProbOfExist": [5], "Obj_MeasState": [2],
      },
      "Obj_3_Extended": {
        "Obj_ID": [], "Obj_ArelLong": [], "Obj_ArelLat": [], "Obj_Class": [],
        "Obj_OrientationAngle": [], "Obj_Length": [], "Obj_Width": [],
      },
    },
  })()

  objects = radar._decode_cycle(1_000_000_000)

  assert set(objects) == {1}
  assert objects[1]["Obj_DistLong"] == 30.0
  assert objects[1]["Obj_ProbOfExist"] == 5


def test_partial_extended_cycle_merges_each_available_object_independently():
  radar = RadarInterface.__new__(RadarInterface)
  radar.expected_objects = 2
  radar.cycle_frames = []
  radar.rcp = type("FakeParser", (), {
    "update": lambda *_args: None,
    "vl_all": {
      "Obj_1_General": {
        "Obj_ID": [1, 2], "Obj_DistLong": [30.0, 60.0], "Obj_DistLat": [0.0, 3.5],
        "Obj_VrelLong": [-1.0, 2.0], "Obj_VrelLat": [0.1, -0.2], "Obj_RCS": [10.0, 8.0],
        "Obj_DynProp": [0, 2],
      },
      "Obj_2_Quality": {
        "Obj_ID": [1, 2], "Obj_ProbOfExist": [5, 6], "Obj_MeasState": [2, 2],
      },
      "Obj_3_Extended": {
        "Obj_ID": [1], "Obj_ArelLong": [-0.75], "Obj_ArelLat": [0.25], "Obj_Class": [1],
        "Obj_OrientationAngle": [2.0], "Obj_Length": [4.6], "Obj_Width": [1.9],
      },
    },
  })()

  objects = radar._decode_cycle(1_000_000_000)

  assert objects[1]["Obj_ArelLong"] == -0.75
  assert objects[1]["Obj_Class"] == 1
  assert "Obj_ArelLong" not in objects[2]
  assert "Obj_Class" not in objects[2]


def test_build_result_exposes_extended_acceleration_and_classification():
  radar = make_result_radar()
  radar.expected_objects = 1
  radar.part_counts = {ARS408_GENERAL: 1, ARS408_QUALITY: 1, ARS408_EXTENDED: 1}
  radar.part_ids = {ARS408_GENERAL: {9}, ARS408_QUALITY: {9}, ARS408_EXTENDED: {9}}
  radar._decode_cycle = lambda _timestamp: {9: object_data(
    Obj_ID=9, Obj_DistLong=42.0, Obj_DistLat=-1.2, Obj_VrelLong=-3.0, Obj_VrelLat=0.4,
    Obj_RCS=12.0, Obj_ArelLong=-0.8, Obj_ArelLat=0.2, Obj_Class=2,
    Obj_OrientationAngle=-3.0, Obj_Length=8.0, Obj_Width=2.5,
  )}

  result = radar._build_result(1_000_000_000)
  point = result.points[0]

  assert point.trackId == 9
  assert math.isclose(point.aRel, -0.8, abs_tol=1e-6)
  assert math.isclose(point.aRelLat, 0.2, abs_tol=1e-6)
  assert point.objectClass == 2
  assert point.classValid
  assert point.length == 8.0
  assert point.width == 2.5
  assert point.rcs == 12.0


def test_missing_extended_frame_uses_nan_acceleration_and_invalid_class():
  radar = make_result_radar()
  radar.expected_objects = 1
  radar.part_counts = {ARS408_GENERAL: 1, ARS408_QUALITY: 1, ARS408_EXTENDED: 0}
  radar.part_ids = {ARS408_GENERAL: {4}, ARS408_QUALITY: {4}, ARS408_EXTENDED: set()}
  radar._decode_cycle = lambda _timestamp: {4: object_data(Obj_RCS=7.0)}

  point = radar._build_result(1_000_000_000).points[0]

  assert math.isnan(point.aRel)
  assert not point.classValid
  assert point.objectClass == 7


def test_generic_tracker_uses_radar_relative_acceleration_when_available():
  point = structs.RadarData.RadarPoint()
  point.measured = True
  point.dRel = 30.0
  point.yRel = 0.0
  point.vRel = -2.0
  point.yvRel = 0.0
  point.vLead = 18.0
  point.aRel = -1.25
  track = MyTrack(3, point, 0.05)

  track.update(point, a_ego=0.5)
  track.update(point, a_ego=0.5)

  assert math.isclose(track.aLead, -0.75)
