"""Verify that both wrists reach the physical chest before the driver is pulled."""
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
    grip = result['chest_grip']
    assert grip['drawn_ticks'] >= .9 * result['held_ticks'], 'missing drawn wrists'
    assert grip['before_grasp_m'] is not None and grip['before_grasp_m'] < .035, 'late reach'
    assert grip['first_grasp_m'] < .035, 'body moved before the hands arrived'
    assert grip['p95_m'] < .035 and grip['max_m'] < .055, 'wrist lost contact with chest'
    rows = trace['samples']
    first = next(i for i, row in enumerate(rows) if row.get('ik_weight', 0) > 0)
    row = rows[first]
    assert row['ik_weight'] < .1, 'arms snapped to full reach'
    assert max(math.dist(w, row[k]) for w, k in zip(row['drawn_wrists'], ('left', 'right'))) < .05, 'jump on first IK frame'
    print(f"PASS {path.name}: largest wrist separation {grip['max_m']:.3f} m")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', nargs='+', type=Path)
    for capture in parser.parse_args().capture:
        check(capture)
