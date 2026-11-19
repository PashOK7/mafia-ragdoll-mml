"""Exercise the native transport death paths in a running city freeride lab.

Run python tests/check_transport_occupants.py 1.2 (or 1.0) with ragdoll loaded.
This kills four native occupants in the selected lab. The player can stay still.
"""
import math
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'MafiaModLoader/tests/game'))
from mml_game import Game


def check(game):
    state = game.call('game')
    assert state['mission_state'] == 'running'
    assert game.call('ragdoll.state')['enabled']
    v12 = state['build'] == '1.2'

    def read(at, size):
        return bytes.fromhex(game.call('mem.read', addr=hex(at), size=size)['hex'])

    def word(at):
        return struct.unpack('<I', read(at, 4))[0]

    def roster(at):
        begin, end = struct.unpack('<II', read(at, 8))
        assert 0 <= end - begin <= 4096
        return [p for (p,) in struct.iter_unpack('<I', read(begin, end - begin))] if end > begin else []

    targets = {}
    for record in roster(0x63D848 if v12 else 0x6561A0):
        if not record or not read(record, 1)[0]:
            continue
        car, model = word(record + 12), word(record + 28)
        if not car or not model or read(car + (0x2131 if v12 else 0x2071), 1)[0]:
            continue
        params = dict(at=hex(0x4BADD0 if v12 else 0x506C00), conv='fastcall', ret='void')
        # Hitting the vehicle frame is not a kill of the occupant.
        game.call('native.call', **params, args=[car, word(car + 0x68)])
        assert not any(int(o['model'], 16) == model for o in game.call('ragdoll.state')['occupants'])
        game.call('native.call', **params, args=[car, model])
        assert read(car + (0x2131 if v12 else 0x2071), 1)[0]
        targets['traffic_driver'] = model
        break

    player = game.player()['pos']
    rails = sorted((a for a in game.call('actors') if a['type'] == 8),
                   key=lambda a: math.dist(a['pos'], player))
    for rail in rails:
        actor = int(rail['actor'], 16)
        for person in roster(actor + 0x254):
            state, model = struct.unpack('<II', read(person, 8))
            if state not in (9, 19) or not model:
                continue
            carriage = word(person + 24)
            if person == actor + 0x2B0:
                role = 'rail_driver'
            elif not word(actor + 0x1A8):
                role = 'tram_passenger'
            elif carriage == 1:
                role = 'train_passenger'
            else:
                continue
            if role in targets:
                continue
            game.call('native.call', at=hex(0x484230 if v12 else 0x4D5160), conv='thiscall',
                      this=person, args=[actor], ret='void')
            targets[role] = model
        if len(targets) == 4:
            break
    assert len(targets) == 4, targets

    # Keep observing past the native death clip. Its terminal states differ for
    # passengers and the driver. Ask for a post-draw bone sample at each poll.
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        occupants = {int(o['model'], 16): o for o in game.call('ragdoll.state')['occupants']}
        if all(m in occupants and occupants[m]['age'] >= 3 and
               0 <= occupants[m]['drawn_neck_error'] < .15 for m in targets.values()):
            break
        time.sleep(.1)
    for role, model in targets.items():
        body = occupants[model]
        assert body['age'] >= 3 and all(math.isfinite(v) for v in body['head'] + body['pelvis']), body
        assert body['kind'] == ('rail_passenger' if role.endswith('passenger') else role), body
        assert body['carriage'] == int(role == 'train_passenger'), body
        assert 0 <= body['drawn_neck_error'] < .15, body
        print(game.build, role, body['age'], body['drawn_neck_error'])

    def float_bits(value):
        return struct.unpack('<I', struct.pack('<f', value))[0]

    engine = word(int(game.call('game')['mission'], 16) + 0x24)
    shooter = int(game.call('game')['player'], 16)
    def fire(origin, direction):
        game.call('native.call', at=hex(0x5A84A0 if v12 else 0x5E3DF0), conv='thiscall',
                  this=engine, ret='uint', args=[shooter, *map(float_bits, origin),
                  *(float_bits(v * 160) for v in direction), float_bits(28), 0, 0, 1])
        time.sleep(.32)
        return game.call('ragdoll.state')

    scale = game.call('state.time')['scale']
    try:
        game.call('state.time', scale=.15)
        for role, model in targets.items():
            if role == 'traffic_driver':
                continue
            for through_floor in (True, False):
                before = game.call('ragdoll.state')
                rider = next(o for o in before['occupants'] if int(o['model'], 16) == model)
                carrier = int(rider['carrier'], 16)
                frame = word(word(carrier + 0xC4)) if rider['carriage'] else word(carrier + 0x68)
                address = game.call('native.call', at=hex(0x47ACD0 if v12 else 0x4F46C0),
                                    conv='fastcall', args=[frame], ret='uint')['ret']
                world = struct.unpack('<16f', read(address, 64))
                head = rider['head']
                local = [v - w for v, w in zip(head, world[12:15])]
                if through_floor:
                    direction = world[4:7]
                    distance = sum(v * w for v, w in zip(local, direction)) + .3
                else:
                    side = 1 if sum(v * w for v, w in zip(local, world[:3])) > 0 else -1
                    direction = [side * v for v in world[:3]]
                    distance = .42
                after = fire([v - distance * d for v, d in zip(head, direction)], direction)
                impacts = after['corpse_hits'] - before['corpse_hits']
                assert impacts == int(not through_floor), (role, through_floor, impacts, after['last_bullet'])
                if impacts:
                    hit = after['last_bullet']
                    assert int(hit['actor'], 16) == model and hit['part'] >= 0 and hit['local_dv'] > 0, hit
                    print(game.build, role, 'floor blocked, exposed hit', hit['part'], hit['local_dv'])
    finally:
        game.call('state.time', scale=scale)


if __name__ == '__main__':
    if sys.argv[1:] not in (['1.0'], ['1.2']):
        raise SystemExit('Pass one configured lab build (1.0 or 1.2).')
    lab = Game(sys.argv[1])
    try:
        check(lab)
    finally:
        lab.close()
