"""Check the ambulance footwell, hand contact and release in a normal-speed driver pull."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from analyze_driver_pull import analyze


def check(path):
    trace = json.loads(path.read_bytes())
    assert trace['car_model'].lower().endswith('ambulance10.i3d'), 'wrong vehicle'
    result = analyze(trace)
    assert result['end'] == 'completed' and result.get('grip_s', 0) > .6, 'incomplete pull'
    assert result['max_tick_s'] < .025, 'tick spacing too large to assess jitter'
    held = next(row['t'] for row in trace['samples'] if row['coat'] is not None)
    # Floor support is expected. A shin crossing the sloped bulkhead in the
    # original capture penetrated it by more than the radius of the shin.
    shins = [c['depth_m'] for row in trace['samples'] if row['t'] < held
             for c in row['cabin_contacts'] if c['part'] in (10, 13)]
    assert max(shins, default=0) < .025, 'shin embedded in the cabin before grasp'
    assert min(result['final_tyre_clearance_m']) > 0, 'torso or foot remained inside the wheel line'
    assert result['peak_gap_m'] < .05 and .95 <= result['tracking_ratio'] <= 1.05, 'hand contact slipped'
    assert result['release_rise_m'] < .1, 'pelvis launched at release'
    head = result['head_motion']['held']['drawn_nose_m']
    assert head['n'] >= .9 * (result['held_ticks'] - 2) and head['p95'] < .01, 'drawn head jerked'
    print(f"PASS {path.name}: nearest clearance {min(result['final_tyre_clearance_m']):.3f} m")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', nargs='+', type=Path)
    for capture in parser.parse_args().capture:
        check(capture)
