import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.carrot.carrot_functions import (
  CarrotPlanner,
  TRAFFIC_STOP_CONFIRM_TIME,
  TRAFFIC_STOP_RELEASE_TIME,
  TrafficState,
  XState,
)


class PassThroughFilter:
  def process(self, value):
    return value


def make_planner():
  planner = CarrotPlanner.__new__(CarrotPlanner)
  planner.vFilter = PassThroughFilter()
  planner.comfortBrake = 2.4
  planner.trafficState = TrafficState.off
  planner.xState = XState.e2eCruise
  planner.stopSignCount = 0
  planner.noStopSignCount = 0
  planner.startSignCount = 0
  return planner


def update_stop_model(planner, *, model_x=80.0, model_v=0.0, v_ego=15.0):
  velocity = np.array([v_ego, model_v])
  planner.check_model_stopping(100.0, velocity, v_ego, 0.0, model_x, np.zeros(2), 1000.0)


def test_single_model_frame_does_not_trigger_signal_stop():
  planner = make_planner()

  update_stop_model(planner)

  assert planner.trafficState == TrafficState.off
  assert planner.stopSignCount == 1


def test_signal_stop_requires_sustained_model_prediction():
  planner = make_planner()
  required_frames = round(TRAFFIC_STOP_CONFIRM_TIME / DT_MDL)

  for _ in range(required_frames - 1):
    update_stop_model(planner)
    assert planner.trafficState == TrafficState.off

  update_stop_model(planner)
  assert planner.trafficState == TrafficState.red


def test_implausibly_close_stop_is_rejected():
  planner = make_planner()
  required_frames = round(TRAFFIC_STOP_CONFIRM_TIME / DT_MDL)

  for _ in range(required_frames + 2):
    update_stop_model(planner, model_x=50.0, v_ego=20.0)

  assert planner.trafficState == TrafficState.off
  assert planner.stopSignCount == 0


def test_confirmed_stop_has_release_hysteresis():
  planner = make_planner()
  required_frames = round(TRAFFIC_STOP_CONFIRM_TIME / DT_MDL)
  release_frames = round(TRAFFIC_STOP_RELEASE_TIME / DT_MDL)

  for _ in range(required_frames):
    update_stop_model(planner)
  assert planner.trafficState == TrafficState.red

  for _ in range(release_frames - 1):
    update_stop_model(planner, model_v=15.0)
    assert planner.trafficState == TrafficState.red

  update_stop_model(planner, model_v=15.0)
  assert planner.trafficState == TrafficState.green


if __name__ == "__main__":
  test_single_model_frame_does_not_trigger_signal_stop()
  test_signal_stop_requires_sustained_model_prediction()
  test_implausibly_close_stop_is_rejected()
  test_confirmed_stop_has_release_hysteresis()
  print("traffic stop tests passed")
