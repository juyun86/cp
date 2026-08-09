from openpilot.selfdrive.carrot.amap_navi import AmapNaviServ, SharedData


def make_service():
  service = AmapNaviServ.__new__(AmapNaviServ)
  service.shared_data = SharedData()
  return service


def test_stock_bsd_becomes_available_only_after_both_can_channels_arrive():
  service = make_service()
  assert not service.stock_bsd_available()

  service.shared_data.left_blindspot = False
  assert not service.stock_bsd_available()

  service.shared_data.right_blindspot = False
  assert service.stock_bsd_available()


def test_stock_bsd_is_merged_with_external_blind_zone_inputs():
  service = make_service()
  service.shared_data.left_blindspot = True
  service.shared_data.right_blindspot = False

  assert service.left_blindspot()
  assert not service.right_blindspot()

  service.shared_data.right_blind = True
  assert service.right_blindspot()
