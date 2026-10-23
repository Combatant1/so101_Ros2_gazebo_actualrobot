#!/usr/bin/env python3
"""Turn a LeRobot calibration JSON into the per-joint <param> lines for so101_ros2_control.xacro.

  python3 tools/calib_to_xacro.py ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/<id>.json

Convention (see README, "Calibration"):
  arm joints : zero_tick = (range_min + range_max) / 2     (LeRobot "degrees" convention)
  gripper    : zero_tick = range_min                       (fully closed = 0 rad)
min_tick / max_tick are the recorded range; `direction` is left at 1 - flip it in the xacro if a
joint turns the wrong way (check with read_only:=true first).
"""
import json
import sys

ORDER = ["gripper", "wrist_roll", "wrist_flex", "elbow_flex", "shoulder_lift", "shoulder_pan"]


def fmt(x):
    return str(int(x)) if float(x).is_integer() else str(x)


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    cal = json.load(open(sys.argv[1]))
    missing = [j for j in ORDER if j not in cal]
    if missing:
        sys.exit(f"Calibration file has no entry for: {', '.join(missing)}")
    for name in ORDER:
        c = cal[name]
        lo, hi = c["range_min"], c["range_max"]
        zero = lo if name == "gripper" else (lo + hi) / 2.0
        print(f'<!-- {name} -->')
        print(f'<param name="id">{c["id"]}</param>')
        print(f'<param name="zero_tick">{fmt(zero)}</param>')
        print(f'<param name="min_tick">{lo}</param>')
        print(f'<param name="max_tick">{hi}</param>')
        print('<param name="direction">1</param>\n')


if __name__ == "__main__":
    main()
