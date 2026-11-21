"""Measure a driver-pull.json capture in metres and simulation seconds. Python standard library only."""
import argparse
import json
import math
from pathlib import Path


def subtract(a, b):
    return tuple(x - y for x, y in zip(a, b))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def hand(row):
    return tuple((a + b) * .5 for a, b in zip(row['left'], row['right']))


def grip_hands(row):
    if 'drawn_wrists' in row:
        left, right = row['drawn_wrists']
        return tuple((a + b) * .5 for a, b in zip(left, right))
    return hand(row)


def side(row, point):
    return dot(subtract(point, row['car']), row['side'])


def percentile95(values):
    return sorted(values)[math.ceil(.95 * len(values)) - 1]


def head_motion(rows):
    """Three-tick residuals separate rapid neck reversals from a smooth turn."""
    angles, physical, drawn = [], [], []
    for a, b, c in zip(rows, rows[1:], rows[2:]):
        if not all(s.get('neck_q') for s in (a, b, c)):
            continue
        u = (b['t'] - a['t']) / (c['t'] - a['t'])
        qa, qb, qc = (s['neck_q'] for s in (a, b, c))
        # A quaternion and its negative describe the same orientation.
        sign = 1 if dot(qa, qc) >= 0 else -1
        interp = [(1-u)*x + u*sign*y for x,y in zip(qa, qc)]
        cosine = abs(dot(interp, qb)) / (math.hypot(*interp)*math.hypot(*qb))
        angles.append(math.degrees(2 * math.acos(min(1., cosine))))
        for field, values in [('head', physical), ('drawn_nose', drawn)]:
            if all(s.get(field) is not None for s in (a, b, c)):
                midpoint = [(1-u)*x + u*y for x,y in zip(a[field], c[field])]
                values.append(math.dist(b[field], midpoint))
    return {name: {'peak': max(values), 'p95': percentile95(values), 'n': len(values)}
            for name, values in [('neck_deg', angles), ('head_m', physical), ('drawn_nose_m', drawn)] if values}


def cabin_resistance(rows, edge):
    """Count sustained cabin contacts facing against the pull while the grip lags."""
    names = ('pelvis', 'abdomen', 'lower chest', 'chest', 'head',
             'right upper arm', 'right forearm', 'left upper arm', 'left forearm',
             'right thigh', 'right shin', 'right foot', 'left thigh', 'left shin', 'left foot')
    counts = {}
    for row in rows:
        if row['gap'] < .05 or not row['side'] or not edge:
            continue
        sign = math.copysign(1, edge)
        for c in row['cabin_contacts']:
            if sign * dot(c['normal'], row['side']) > -.25:
                continue
            part = c['part']
            key = (names[part] if 0 <= part < len(names) else str(part), c['mesh'])
            counts[key] = counts.get(key, 0) + 1
    return [{'part': part, 'mesh': mesh, 'ticks': count}
            for (part, mesh), count in sorted(counts.items(), key=lambda item: -item[1])]


def analyze(trace):
    rows = trace['samples']
    if any(b['t'] <= a['t'] for a, b in zip(rows, rows[1:])):
        raise ValueError('unordered simulation ticks')
    held = [s for s in rows if all(s[k] is not None for k in ('coat', 'left', 'right'))]
    result = {'ticks': len(rows), 'end': trace['end'], 'held_ticks': len(held)}
    if not held:
        return result
    start, end = held[0]['t'], held[-1]['t']
    settled = [s for s in held if s['t'] >= start + .22]
    after = [s for s in rows if s['t'] > end]
    if not settled or not after:
        return result

    # Compare the same material point on the coat with the hands over 80 ms windows.
    # Integrate speed along the hand path. A slow body under-reports this ratio.
    hand_travel = coat_progress = 0.
    base = settled[0]
    for s in settled[1:]:
        if s['t'] - base['t'] < .08:
            continue
        h = subtract(grip_hands(s), grip_hands(base))
        if math.hypot(*h) > .015:
            hand_travel += math.hypot(*h)
            coat_progress += dot(subtract(s['coat'], base['coat']), h) / math.hypot(*h)
        base = s

    def peak_speed(group, field):
        return max((math.dist(a[field], b[field]) / (b['t'] - a['t'])
                    for a, b in zip(group, group[1:])), default=0)

    # High-frequency motion relative to the hands: distance from the interpolated
    # neighbouring residuals. Smooth translation or a turn by Tommy cancels out.
    jitter = []
    for a, b, c in zip(settled, settled[1:], settled[2:]):
        ra, rb, rc = [subtract(s['coat'], grip_hands(s)) for s in (a, b, c)]
        u = (b['t'] - a['t']) / (c['t'] - a['t'])
        jitter.append(math.dist(rb, [x + u * (y - x) for x, y in zip(ra, rc)]))

    # Test for rapid back-and-forth motion of the trunk and legs independently of the grip.
    # A three-tick residual is in metres at the logged tick rate. Include the free landing.
    wobble = [[], [], [], []]
    for a, b, c in zip(rows, rows[1:], rows[2:]):
        u = (b['t'] - a['t']) / (c['t'] - a['t'])
        points = [(s['chest'], s['pelvis'], *s['feet']) for s in (a, b, c)]
        for i, (prev, mid, nxt) in enumerate(zip(*points)):
            wobble[i].append(math.dist(mid, [x + u*(y-x) for x, y in zip(prev, nxt)]))

    edge = trace['outer_tyre_x_m']
    if all('cabin_contacts' in row for row in settled):
        result['opposing_cabin_contacts_while_lagging'] = cabin_resistance(settled, edge)
    def wrist_gap(row):
        points = row.get('grip_contacts', row.get('chest_contacts'))
        return max(math.dist(w, c) for w, c in zip(row['drawn_wrists'], points))
    drawn = [s for s in held if 'drawn_wrists' in s]
    if drawn:
        before = [s for s in rows if s['t'] < start and 'drawn_wrists' in s]
        result['grasp'] = {
            'part': drawn[0].get('grip_part', 'chest'),
            'drawn_ticks': len(drawn),
            'before_grasp_m': wrist_gap(before[-1]) if before else None,
            'first_grasp_m': wrist_gap(drawn[0]),
            'p95_m': percentile95([wrist_gap(s) for s in drawn]),
            'max_m': max(wrist_gap(s) for s in drawn),
        }
        if result['grasp']['part'] == 'chest':
            result['chest_grip'] = result['grasp']
    final = rows[-1]
    tail = [s for s in rows if s['t'] >= final['t'] - .25]
    result.update(grip_s=end-start, max_tick_s=max(b['t']-a['t'] for a,b in zip(rows, rows[1:])),
                  peak_three_tick_residual_m=[max(part, default=0.) for part in wobble],
                  p95_three_tick_residual_m=[percentile95(part) if part else 0. for part in wobble],
                  head_motion={'reach': head_motion([s for s in rows if s['t'] < start]),
                               'held': head_motion(held), 'after': head_motion(after)},
                  peak_gap_m=max(s['gap'] for s in settled),
                  tracking_ratio=coat_progress/hand_travel if hand_travel else None,
                  peak_relative_jitter_m=max(jitter, default=0), peak_grab_speed_m_s=peak_speed(held, 'chest'),
                  peak_release_speed_m_s=peak_speed([held[-1]] + after, 'pelvis'),
                  release_travel_m=max(math.dist(s['pelvis'], held[-1]['pelvis']) for s in after),
                  release_rise_m=max(0, max(s['pelvis'][1] for s in after)-held[-1]['pelvis'][1]),
                  pelvis_settle_m=math.dist(final['pelvis'], tail[0]['pelvis']),
                  pelvis_settle_path_m=sum(math.dist(a['pelvis'], b['pelvis']) for a,b in zip(tail, tail[1:])))
    if edge and final['car'] and rows[0]['car']:
        sign = math.copysign(1, edge)
        result.update(pelvis_outward_m=sign*(side(final, final['pelvis']) - side(rows[0], rows[0]['pelvis'])),
                      final_tyre_clearance_m=[sign*(side(final, p) - edge) for p in
                                              (final['chest'], final['pelvis'], *final['feet'])])
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', nargs='+', type=Path)
    for path in parser.parse_args().capture:
        print(path.name, json.dumps(analyze(json.loads(path.read_text())), allow_nan=False))
