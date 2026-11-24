"""Verify that both hands reach the chosen body surface before the driver is pulled."""
import argparse
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from analyze_driver_pull import analyze


def check(path):
    trace = json.loads(path.read_bytes())
    result = analyze(trace)
    assert result['end'] == 'completed' and result.get('grip_s', 0) > .6, 'incomplete pull'
    grip = result['grasp']
    assert grip['drawn_ticks'] >= .9 * result['held_ticks'], 'missing drawn hands'
    assert grip['before_grasp_m'] is not None and grip['before_grasp_m'] < .035, 'late reach'
    assert grip['first_grasp_m'] < .035, 'body moved before the hands arrived'
    assert grip['p95_m'] < .035 and grip['max_m'] < .055, 'hand lost contact with the body'
    rows = trace['samples']
    held = [i for i, row in enumerate(rows) if row['coat'] is not None]
    if 'grip_part' in rows[held[0]]:
        assert grip['part'] in ('chest', 'chestlow', 'abdomen', 'pelvis', 'l_thigh', 'r_thigh'), 'unsafe grip site'
        assert all(rows[i]['grip_part'] == grip['part'] for i in held), 'grip switched under load'
        assert all(math.dist(*rows[i]['grip_contacts']) >= .064 for i in held), 'no room for both hands'
        for previous, current in zip(held, held[1:]):
            a, b = rows[previous], rows[current]
            dt = b['t'] - a['t']
            assert math.dist(a['coat'], b['coat']) <= 3 * dt + .002, 'contact point jumped during the pull'
        i = held[0]
        before, after = rows[i-1:i+1]
        speed = max(math.dist(before[p], after[p]) for p in ('chest','pelvis')) / (after['t']-before['t'])
        assert speed < 2, 'torso jumped on the first held frame'
    first = next(i for i, row in enumerate(rows) if row.get('ik_weight', 0) > 0)
    row = rows[first]
    assert row['ik_weight'] < .1, 'arms snapped to full reach'
    assert max(math.dist(w, row[k]) for w, k in zip(row['drawn_wrists'], ('left', 'right'))) < .05, 'jump on first IK frame'
    print(f"PASS {path.name}: largest hand separation {grip['max_m']:.3f} m")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', nargs='+', type=Path)
    for capture in parser.parse_args().capture:
        check(capture)
