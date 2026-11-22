"""Explode a native traffic car with a dead driver in a running city-small freeride lab."""
import math
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'MafiaModLoader/tests/game'))
from mml_game import Game
from check_car_explosion import until


def read(game, at, fmt):
    data = game.call('mem.read', addr=hex(at), size=struct.calcsize(fmt))['hex']
    return struct.unpack(fmt, bytes.fromhex(data))


def traffic_driver(game):
    at = game.player()['pos']
    first, end = read(game, 0x63D848 if game.build == '1.2' else 0x6561A0, '<II')
    if not 0 < end - first <= 4096:
        return None
    candidates = []
    for record in read(game, first, f'<{(end - first) // 4}I'):
        if not record or not read(game, record, 'B')[0]:
            continue
        car, _, _, _, model = read(game, record + 12, '<5I')
        if not car or not model or read(game, car + (0x2131 if game.build == '1.2' else 0x2071), 'B')[0]:
            continue
        frame, = read(game, car + 0x68, '<I')
        if not frame:
            continue
        pos = game.call('frame', addr=hex(frame))['pos']
        distance = math.dist(at, pos)
        if 5 < distance < 50:
            candidates.append((distance, car, model))
    return min(candidates, default=None)


def car_frame(game, car):
    root, = read(game, car + 0x68, '<I')
    return game.call('frame', addr=hex(root))


def local(frame, point):
    displacement = [v - c for v, c in zip(point, frame['pos'])]
    return [sum(v * d for v, d in zip(axis, displacement)) for axis in frame['axes']]


def check(game):
    _, car, model = until(lambda: traffic_driver(game), bool)
    game.call('native.call', at=hex(0x4BADD0 if game.build == '1.2' else 0x506C00), conv='fastcall',
              ret='void', args=[car, model])

    def by_model():
        return next((o for o in game.call('ragdoll.state')['occupants'] if int(o['model'], 16) == model), None)

    before = until(by_model, lambda o: o and o['age'] >= .5 and o['in_cabin'])
    assert before['kind'] == 'traffic_driver'
    old_frame = car_frame(game, car)
    old_local = local(old_frame, before['pelvis'])
    seq = game.latest()
    result = game.call('native.call', at=hex(0x421D60 if game.build == '1.2' else 0x468BC0),
                       conv='thiscall', ret='uint', args=[0], **{'this': hex(car)})
    assert result['stack']['ok']
    events = game.call('events.since', seq=seq, types='CAR_EXPLODED,MOD_FAULTED')['events']
    assert any(e['type'] == 'CAR_EXPLODED' and int(e['data']['actor'], 16) == car for e in events), events
    assert all(e['type'] != 'MOD_FAULTED' for e in events), events

    def by_serial():
        return next((o for o in game.call('ragdoll.state')['occupants']
                     if o['body_serial'] == before['body_serial']), None)

    after = until(by_serial, lambda o: o and o['kind'] == 'wreck_driver')
    wreck = int(after['carrier'], 16)
    assert wreck and wreck != car and after['in_cabin'], after
    assert read(game, wreck + 0x65, 'B')[0] == 1
    wreck_frame = car_frame(game, wreck)
    jump = math.dist(old_local, local(wreck_frame, after['pelvis']))
    assert jump < .8, (jump, before, after)
    saved = game.call('model.file', frame=after['model'])
    assert saved['file'].lower().endswith(r'\ohorelec01.i3d'), saved
    assert game.call('frame', addr=after['model'])['parent'] == wreck_frame['addr']
    assert int(after['model'], 16) != model, after
    later = until(by_serial, lambda o: o and o['age'] >= before['age'] + 2)
    assert later['in_cabin'] and later['carrier'] == after['carrier'] and 0 <= later['drawn_neck_error'] < .02, later
    drift = math.dist(old_local, local(car_frame(game, wreck), later['pelvis']))
    assert drift < .8, (drift, before, later)
    prop = next(p for p in game.call('props.list')['props'] if p['prop'] == later['model'])
    assert prop['on'] and prop['parent'] == wreck_frame['addr'], prop
    print(game.build, 'traffic driver', before['body_serial'], 'kept after explosion',
          'wreck', after['carrier'], 'pelvis shift in cabin', round(jump, 3), 'then', round(drift, 3),
          'drawn neck error', later['drawn_neck_error'], flush=True)


if __name__ == '__main__':
    if sys.argv[1:] not in (['1.0'], ['1.2']):
        raise SystemExit('Pass one configured lab build (1.0 or 1.2).')
    game = Game(sys.argv[1])
    try:
        assert game.call('ragdoll.state')['enabled']
        check(game)
    finally:
        game.close()
