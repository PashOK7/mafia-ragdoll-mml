"""Check the inward reach and horizontal release in stationary Falconer captures."""
import argparse
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from analyze_driver_pull import analyze, hand, side


def check(path):
    trace = json.loads(path.read_bytes())
    assert trace['car_model'].lower().endswith('blackha11.i3d')
    result = analyze(trace)
    assert result['end'] == 'completed' and result['max_tick_s'] < .025
    rows = trace['samples']
    held = [r for r in rows if r['coat'] is not None]
    sign = math.copysign(1, trace['outer_tyre_x_m'])
    deepest = min((r for r in rows if r['left'] is not None),
                  key=lambda r: sign * side(r, hand(r)))
    initial = side(rows[0], rows[0]['head'])
    early = max(sign * (side(r, r['head']) - initial) for r in rows if r['t'] <= deepest['t'])
    assert early < .15 and held[0]['t'] >= deepest['t'], 'upper body lurched out during the reach'
    assert result['grip_s'] > .6 and result['peak_gap_m'] < .05, 'lost contact with the driver'
    release = held[-1]
    later = next(r for r in rows if r['t'] >= release['t'] + .079)
    def travel(r):
        return math.hypot(*(r['pelvis'][i] - release['pelvis'][i] for i in (0, 2)))
    speed = travel(later) / (later['t'] - release['t'])
    # Turning over on the pavement can move the pelvis along the car. Measure
    # how far it was thrown out through the door separately from that roll.
    start_side = side(release, release['pelvis'])
    outward = max(sign * (side(r, r['pelvis']) - start_side) for r in rows if r['t'] >= release['t'])
    assert speed < 2 and outward < .45, 'driver was flung out through the door'
    assert min(result['final_tyre_clearance_m']) > 0, 'torso or foot stayed inside the tyre line'
    print(f'PASS {path.name}: early head motion {early:.3f} m, horizontal release {speed:.2f} m/s')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', nargs='+', type=Path)
    for capture in parser.parse_args().capture:
        check(capture)
