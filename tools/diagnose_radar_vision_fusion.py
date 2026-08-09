#!/usr/bin/env python3
import argparse
import json
import time

from cereal import messaging


RADAR_TO_CAMERA = 1.52


def main():
  parser = argparse.ArgumentParser(description="Sample live vision/radar lead fusion without sending control messages")
  parser.add_argument("--seconds", type=float, default=10.0)
  parser.add_argument("--summary", action="store_true")
  args = parser.parse_args()

  sm = messaging.SubMaster(["modelV2", "radarState", "carState"], poll="radarState")
  deadline = time.monotonic() + args.seconds
  total = status_frames = radar_frames = transitions = track_transitions = large_distance_jumps = 0
  previous_radar = None
  previous_track_id = None
  previous_lead_distance = None
  max_lead_distance_jump = 0.0
  samples = []
  transition_events = []

  while time.monotonic() < deadline:
    sm.update(1000)
    if not sm.updated["radarState"] or not sm.seen["modelV2"]:
      continue

    total += 1
    lead = sm["radarState"].leadOne
    vision = sm["modelV2"].leadsV3[0]
    if lead.status:
      status_frames += 1
    if lead.status and lead.radar:
      radar_frames += 1
    current_radar = bool(lead.status and lead.radar)
    if previous_radar is not None and current_radar != previous_radar:
      transitions += 1
    previous_radar = current_radar
    current_track_id = int(lead.radarTrackId) if current_radar else None
    track_changed = previous_track_id is not None and current_track_id is not None and current_track_id != previous_track_id
    if track_changed:
      track_transitions += 1
    current_lead_distance = float(lead.dRel) if lead.status else None
    if previous_lead_distance is not None and current_lead_distance is not None:
      distance_jump = abs(current_lead_distance - previous_lead_distance)
      max_lead_distance_jump = max(max_lead_distance_jump, distance_jump)
      if distance_jump > 3.0:
        large_distance_jumps += 1
    previous_lead_distance = current_lead_distance

    if track_changed and len(transition_events) < 20:
      candidates = []
      seen_transition_ids = set()
      for group in (sm["radarState"].leadsLeft, sm["radarState"].leadsCenter, sm["radarState"].leadsRight):
        for track in group:
          track_id = int(track.radarTrackId)
          if track_id in seen_transition_ids or track.dRel > 30.0:
            continue
          seen_transition_ids.add(track_id)
          candidates.append({"id": track_id, "d": round(float(track.dRel), 2),
                             "y": round(float(track.yRel), 2), "score": round(float(track.score), 6)})
      transition_events.append({
        "from": previous_track_id,
        "to": current_track_id,
        "visionD": round(float(vision.x[0]) - RADAR_TO_CAMERA, 2),
        "leadD": round(float(lead.dRel), 2),
        "candidates": sorted(candidates, key=lambda track: track["d"]),
      })
    previous_track_id = current_track_id

    if len(samples) < 12 and (total == 1 or total % 10 == 0):
      nearby_tracks = []
      seen_ids = set()
      for group in (sm["radarState"].leadsLeft, sm["radarState"].leadsCenter, sm["radarState"].leadsRight):
        for track in group:
          track_id = int(track.radarTrackId)
          if track_id in seen_ids or track.dRel > 40.0:
            continue
          seen_ids.add(track_id)
          nearby_tracks.append({
            "id": track_id,
            "d": round(float(track.dRel), 2),
            "y": round(float(track.yRel), 2),
            "v": round(float(track.vLead), 2),
            "score": round(float(track.score), 6),
            "class": int(track.objectClass),
            "classValid": bool(track.classValid),
            "probability": int(track.probability),
            "dynamicProperty": int(track.dynamicProperty),
            "rcs": round(float(track.rcs), 1),
          })
      samples.append({
        "vEgo": round(float(sm["carState"].vEgo), 2),
        "visionProb": round(float(vision.prob), 3),
        "visionD": round(float(vision.x[0]) - RADAR_TO_CAMERA, 2),
        "leadStatus": bool(lead.status),
        "leadRadar": bool(lead.radar),
        "trackId": int(lead.radarTrackId),
        "leadD": round(float(lead.dRel), 2),
        "leadY": round(float(lead.yRel), 2),
        "leadV": round(float(lead.vLead), 2),
        "nearbyTracks": sorted(nearby_tracks, key=lambda track: track["d"]),
      })

  result = {
    "seconds": args.seconds,
    "radarStateFrames": total,
    "leadStatusFrames": status_frames,
    "fusedRadarFrames": radar_frames,
    "fusionRateWhenLeadPresent": round(radar_frames / status_frames, 3) if status_frames else 0.0,
    "radarVisionTransitions": transitions,
    "radarTrackTransitions": track_transitions,
    "largeLeadDistanceJumps": large_distance_jumps,
    "maxLeadDistanceJump": round(max_lead_distance_jump, 2),
  }
  if args.summary:
    result["transitionEvents"] = transition_events
  else:
    result["samples"] = samples
    result["transitionEvents"] = transition_events
  print(json.dumps(result, indent=2))


if __name__ == "__main__":
  main()
