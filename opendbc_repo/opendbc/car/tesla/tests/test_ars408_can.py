import math

from opendbc.can import CANParser
from opendbc.car.tesla.ars408_can import (
  ARS408_BUS, ARS408_FILTER_CLEAR_PASSES, ARS408_FILTER_CLEAR_STEP, ARS408_FILTER_CONFIG_ADDRESS,
  ARS408_MAX_DISTANCE, ARS408_MAX_OBJECTS, ARS408_OBJECT_FILTER_INDICES_TO_DISABLE,
  ARS408_RADAR_CONFIG_ADDRESS,
  ARS408_RADAR_CONFIG_ADDRESSES, ARS408_SEND_EXTENDED, ARS408_SENSOR_ID, ARS408_SPEED_ADDRESS, ARS408_YAW_RATE_ADDRESS,
  ARS408CAN,
)


def test_startup_configuration_is_safe_for_shared_tesla_can():
  messages = [ARS408CAN().create_radar_configuration(address) for address in ARS408_RADAR_CONFIG_ADDRESSES]

  assert ARS408_RADAR_CONFIG_ADDRESSES == (0x200,)
  for address, data, bus in messages:
    assert address in ARS408_RADAR_CONFIG_ADDRESSES
    assert bus == ARS408_BUS == 1
    assert len(data) == 8


def test_decoder_and_configuration_share_sensor_id():
  assert ARS408_SENSOR_ID == 0


def test_configuration_uses_250_m_range_and_enables_extended_output():
  address, data, bus = ARS408CAN().create_radar_configuration(ARS408_RADAR_CONFIG_ADDRESS)
  parser = CANParser("ARS408", [("RadarConfiguration", math.nan)], bus)
  parser.update([(1_000_000_000, [(0x200, data, bus)])])

  assert address == 0x200
  assert ARS408_MAX_DISTANCE == 250
  assert parser.vl["RadarConfiguration"]["RadarCfg_MaxDistance"] == 250
  assert parser.vl["RadarConfiguration"]["RadarCfg_SensorID"] == ARS408_SENSOR_ID == 0
  assert ARS408_SEND_EXTENDED
  assert parser.vl["RadarConfiguration"]["RadarCfg_SendExtInfo"] == 1
  assert parser.vl["RadarConfiguration"]["RadarCfg_SendQuality"] == 1
  assert parser.vl["RadarConfiguration"]["RadarCfg_StoreInNVM"] == 0


def test_commissioning_configuration_can_request_one_nvm_write():
  _address, data, bus = ARS408CAN().create_radar_configuration(store_in_nvm=True)
  parser = CANParser("ARS408", [("RadarConfiguration", math.nan)], bus)
  parser.update([(1_000_000_000, [(0x200, data, bus)])])

  assert parser.vl["RadarConfiguration"]["RadarCfg_StoreInNVM_valid"] == 1
  assert parser.vl["RadarConfiguration"]["RadarCfg_StoreInNVM"] == 1


def test_motion_inputs_use_id0_namespace_and_dbc_scaling():
  can = ARS408CAN()
  speed_address, speed_data, bus = can.create_speed_information(27.78, 1)
  yaw_address, yaw_data, _bus = can.create_yaw_rate_information(-12.34)
  parser = CANParser("ARS408", [("SpeedInformation", math.nan), ("YawRateInformation", math.nan)], bus)
  parser.update([(1_000_000_000, [(0x300, speed_data, bus), (0x301, yaw_data, bus)])])

  assert speed_address == ARS408_SPEED_ADDRESS == 0x300
  assert yaw_address == ARS408_YAW_RATE_ADDRESS == 0x301
  assert len(speed_data) == len(yaw_data) == 2
  assert parser.vl["SpeedInformation"]["RadarDevice_SpeedDirection"] == 1
  assert math.isclose(parser.vl["SpeedInformation"]["RadarDevice_Speed"], 27.78, abs_tol=0.02)
  assert math.isclose(parser.vl["YawRateInformation"]["RadarDevice_YawRate"], -12.34, abs_tol=0.01)


def test_object_count_filter_caps_bus_load():
  address, data, bus = ARS408CAN().create_object_count_filter()
  parser = CANParser("ARS408", [("FilterCfg", math.nan)], bus)
  parser.update([(1_000_000_000, [(0x202, data, bus)])])

  assert address == ARS408_FILTER_CONFIG_ADDRESS == 0x202
  assert data == bytes.fromhex("8600000020")
  assert parser.vl["FilterCfg"]["FilterCfg_Valid"] == 1
  assert parser.vl["FilterCfg"]["FilterCfg_Active"] == 1
  assert parser.vl["FilterCfg"]["FilterCfg_Type"] == 1
  assert parser.vl["FilterCfg"]["FilterCfg_Index"] == 0
  assert parser.vl["FilterCfg"]["FilterCfg_Max_NofObj"] == ARS408_MAX_OBJECTS == 32


def test_non_count_object_filters_are_disabled_with_exact_payloads():
  messages = ARS408CAN().create_disabled_object_filters()
  assert ARS408_OBJECT_FILTER_INDICES_TO_DISABLE == tuple(range(1, 15))
  assert len(messages) == 14
  assert ARS408_FILTER_CLEAR_STEP == 20
  assert ARS408_FILTER_CLEAR_PASSES == 2

  for index, (address, data, bus) in zip(ARS408_OBJECT_FILTER_INDICES_TO_DISABLE, messages, strict=True):
    parser = CANParser("ARS408", [("FilterCfg", math.nan)], bus)
    parser.update([(1_000_000_000, [(0x202, data, bus)])])

    assert address == ARS408_FILTER_CONFIG_ADDRESS == 0x202
    assert data == bytes((0x82 | (index << 3), 0, 0, 0, 0))
    assert parser.vl["FilterCfg"]["FilterCfg_Valid"] == 1
    assert parser.vl["FilterCfg"]["FilterCfg_Active"] == 0
    assert parser.vl["FilterCfg"]["FilterCfg_Type"] == 1
    assert parser.vl["FilterCfg"]["FilterCfg_Index"] == index
