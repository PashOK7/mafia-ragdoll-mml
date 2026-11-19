"""Check normal-speed driver-pull.json captures from the Bolt V8 Touring regression."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from analyze_driver_pull import analyze


def check(path):
    trace = json.loads(path.read_bytes())
    assert trace['car_model'].replace('\\', '/').lower().endswith('/forvto11.i3d'), 'wrong vehicle'
    result = analyze(trace)
    assert result['end'] == 'completed' and result.get('grip_s', 0) > .6, 'incomplete pull'
    assert result['max_tick_s'] <= .025, 'sampling rate too low to measure shake'
    assert .95 <= result['tracking_ratio'] <= 1.05, 'driver lagged behind hands'
    assert result['peak_gap_m'] < .05, 'grip slipped over 5 cm after take-up'
    assert min(result['final_tyre_clearance_m']) > 0, 'torso or feet stayed inside tyre line'
    assert result['release_rise_m'] < .10, 'body launched upwards after release'
    assert result['release_travel_m'] < 2, 'body travelled too far after release'
    assert max(result['p95_three_tick_residual_m']) < .01, 'trunk or feet jerked during extraction'
    assert result['pelvis_settle_path_m'] < .01, 'body still moving at the end'
    head = result['head_motion']['held']
    assert head['neck_deg']['p95'] < 5, 'neck jerked during the grip'
    assert head['drawn_nose_m']['n'] >= .9 * (result['held_ticks'] - 2), 'missing drawn pose samples'
    assert head['drawn_nose_m']['p95'] < .01, 'drawn face jerked during the grip'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', nargs='+', type=Path)
    for capture in parser.parse_args().capture:
        check(capture)
        print('PASS', capture.name)
