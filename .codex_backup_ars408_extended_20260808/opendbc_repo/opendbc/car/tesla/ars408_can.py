from opendbc.can import CANPacker


ARS408_BUS = 1
ARS408_SENSOR_ID = 0
ARS408_ADDRESS_OFFSET = ARS408_SENSOR_ID << 4
ARS408_RADAR_CONFIG_ADDRESS = 0x200 + ARS408_ADDRESS_OFFSET
# Sensor ID 0 is commissioned once in NVM. Production never transmits radar,
# filter, polygon, or collision configuration frames on the shared vehicle bus.
ARS408_RADAR_CONFIG_ADDRESSES = (ARS408_RADAR_CONFIG_ADDRESS,)
ARS408_FILTER_CONFIG_ADDRESS = 0x202 + ARS408_ADDRESS_OFFSET
ARS408_SPEED_ADDRESS = 0x300 + ARS408_ADDRESS_OFFSET
ARS408_YAW_RATE_ADDRESS = 0x301 + ARS408_ADDRESS_OFFSET
ARS408_MAX_DISTANCE = 250
ARS408_MAX_OBJECTS = 32
ARS408_SEND_EXTENDED = False
ARS408_OBJECT_FILTER_INDICES_TO_DISABLE = tuple(range(1, 15))
ARS408_FILTER_CLEAR_STEP = 20  # 200 ms at the 100 Hz card rate
ARS408_FILTER_CLEAR_PASSES = 2

class ARS408CAN:
  """Creates ARS408 motion frames and reviewed maintenance frames."""

  def __init__(self):
    self.packer = CANPacker("ARS408")

  @staticmethod
  def _replace_address(message, address):
    _base_address, data, bus = message
    return address, data, bus

  def create_radar_configuration(self, address=ARS408_RADAR_CONFIG_ADDRESS, store_in_nvm=False):
    assert address in ARS408_RADAR_CONFIG_ADDRESSES
    values = {
      "RadarCfg_RCS_Threshold_Valid": 1,
      "RadarCfg_RCS_Threshold": 0,       # standard sensitivity
      "RadarCfg_StoreInNVM_valid": int(store_in_nvm),
      "RadarCfg_StoreInNVM": int(store_in_nvm),
      "RadarCfg_SortIndex_valid": 1,
      "RadarCfg_SortIndex": 1,          # nearest objects first
      "RadarCfg_SendExtInfo_valid": 1,
      # General + Quality contain every field used for lead tracking. Turning
      # Extended off removes one frame per object from the shared Tesla bus.
      "RadarCfg_SendExtInfo": int(ARS408_SEND_EXTENDED),
      "RadarCfg_CtrlRelay_valid": 1,
      "RadarCfg_CtrlRelay": 0,
      "RadarCfg_SendQuality_valid": 1,
      "RadarCfg_SendQuality": 1,
      "RadarCfg_MaxDistance_valid": 1,
      "RadarCfg_MaxDistance": ARS408_MAX_DISTANCE,
      "RadarCfg_RadarPower_valid": 1,
      "RadarCfg_RadarPower": 0,          # standard Tx power
      "RadarCfg_OutputType_valid": 1,
      "RadarCfg_OutputType": 1,          # object list, never cluster list
      "RadarCfg_SensorID_valid": 1,
      "RadarCfg_SensorID": ARS408_SENSOR_ID,
    }
    message = self.packer.make_can_msg("RadarConfiguration", ARS408_BUS, values)
    return self._replace_address(message, address)

  def create_speed_information(self, speed_mps, direction):
    """Create the platform-speed input expected by the commissioned sensor ID."""
    values = {
      "RadarDevice_SpeedDirection": int(direction),
      "RadarDevice_Speed": min(max(abs(float(speed_mps)), 0.0), 163.8),
    }
    message = self.packer.make_can_msg("SpeedInformation", ARS408_BUS, values)
    return self._replace_address(message, ARS408_SPEED_ADDRESS)

  def create_yaw_rate_information(self, yaw_rate_deg_s):
    """Create the platform yaw-rate input expected by the commissioned sensor ID."""
    values = {
      "RadarDevice_YawRate": min(max(float(yaw_rate_deg_s), -327.68), 327.67),
    }
    message = self.packer.make_can_msg("YawRateInformation", ARS408_BUS, values)
    return self._replace_address(message, ARS408_YAW_RATE_ADDRESS)

  def create_object_count_filter(self):
    """Cap object-list load without filtering by lane, RCS, or probability."""
    values = {
      "FilterCfg_Type": 1,       # object filter
      "FilterCfg_Index": 0,      # number of objects
      "FilterCfg_Active": 1,
      "FilterCfg_Valid": 1,
      "FilterCfg_Min_NofObj": 0,
      "FilterCfg_Max_NofObj": ARS408_MAX_OBJECTS,
    }
    message = self.packer.make_can_msg("FilterCfg", ARS408_BUS, values)
    return self._replace_address(message, ARS408_FILTER_CONFIG_ADDRESS)

  def create_disabled_object_filter(self, index):
    """Deactivate one non-count object filter; values are ignored when inactive."""
    assert index in ARS408_OBJECT_FILTER_INDICES_TO_DISABLE
    # Motorola layout: Type=object, Index=index, Active=0, Valid=1.
    # Keep the ignored min/max bytes at zero to make the safety allowlist exact.
    data = bytes((0x82 | (index << 3), 0, 0, 0, 0))
    return ARS408_FILTER_CONFIG_ADDRESS, data, ARS408_BUS

  def create_disabled_object_filters(self):
    """Remove all pass-through criteria except the separately configured count cap."""
    return tuple(
      self.create_disabled_object_filter(index)
      for index in ARS408_OBJECT_FILTER_INDICES_TO_DISABLE
    )
