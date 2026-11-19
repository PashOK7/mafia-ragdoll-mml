"""Shoot a native tram passenger off the edge and check the street below.

Run against a windowed city freeride lab with ragdoll loaded. Moves the lab
player near a tram and places one passenger at the edge before the native kill.
"""
import math
import struct
import sys
import time

from check_transport_occupants import Game


def check(game):
    state = game.call('game')
    v12 = state['build'] == '1.2'
    assert state['mission_state'] == 'running' and game.call('ragdoll.state')['enabled']

    def read(at, size):
        return bytes.fromhex(game.call('mem.read', addr=hex(at), size=size)['hex'])

    def word(at):
        return struct.unpack('<I', read(at, 4))[0]

    def native(at, convention, args, owner=None):
        params = {'this': owner} if owner else {}
        return game.call('native.call', at=hex(at), conv=convention, args=args, ret='uint', **params)['ret']

    def matrix(frame):
        address = native(0x47ACD0 if v12 else 0x4F46C0, 'fastcall', [frame])
        return struct.unpack('<16f', read(address, 64))

    def position(w, x, y, z):
        return [w[12+i] + x*w[i] + y*w[4+i] + z*w[8+i] for i in range(3)]

    player = game.player()['pos']
    for car in sorted((a for a in game.call('actors') if a['type'] == 8),
                      key=lambda a: math.dist(player, a['pos'])):
        carrier = int(car['actor'], 16)
        if word(carrier + 0x1A8):
            continue
        begin, end = struct.unpack('<II', read(carrier + 0x254, 8))
        if not 0 < end - begin <= 4096:
            continue
        riders = [p for (p,) in struct.iter_unpack('<I', read(begin, end - begin))
                  if p != carrier + 0x2B0 and word(p) in (9, 19)]
        if not riders:
            continue
        frame = word(carrier + 0x68)
        w = matrix(frame)
        street = game.call('world.ground', at=position(w, 6, 2, 0))
        if street['hit'] and street['normal'][1] > .9 and abs(street['height'] - w[13]) < 3:
            break
    else:
        raise AssertionError('No occupied tram above a street')
    spawn = street['pos']
    game.call('world.teleport', to=[spawn[0], spawn[1] + .35, spawn[2]])
    time.sleep(.6)
    person = riders[0]
    model = word(person + 4)
    engine = word(int(state['mission'], 16) + 0x24)
    shooter = int(state['player'], 16)

    def rider():
        return next(o for o in game.call('ragdoll.state')['occupants'] if int(o['model'], 16) == model)

    scale = game.call('state.time')['scale']
    try:
        game.call('state.time', scale=0)
        w = matrix(frame)
        # Seated variants face either side. Place the hips at the lip in both poses.
        lip = position(w, 1.52, 2.30, 3.15)
        vtable = word(model)
        thighs = [matrix(native(word(vtable + 0x38), 'stdcall', [model, {'str': name}, 65535]))
                  for name in ('l_thigh', 'r_thigh')]
        root = matrix(model)
        target = [lip[i] + root[12+i] - .5*(thighs[0][12+i] + thighs[1][12+i]) for i in range(3)]
        native(word(vtable + 4), 'stdcall', [model, {'buf': 12, 'hex': struct.pack('<3f', *target).hex()}])
        native(word(vtable + 0x18), 'stdcall', [model])
        native(0x484230 if v12 else 0x4D5160, 'thiscall', [carrier], person)
        assert rider()['in_cabin']
        game.call('state.time', scale=.22)
        hits = 0
        for _ in range(30):
            body = rider()
            if not body['in_cabin']:
                break
            w = matrix(frame)
            direction = [w[i] + .18*w[4+i] for i in range(3)]
            start = [p - .32*d for p, d in zip(body['pelvis'], direction)]
            count = game.call('ragdoll.state')['corpse_hits']
            native(0x5A84A0 if v12 else 0x5E3DF0, 'thiscall',
                   [shooter, *({'float': v} for v in start),
                    *({'float': d*160} for d in direction), {'float': 110}, 0, 0, 1], engine)
            time.sleep(.06)
            result = game.call('ragdoll.state')
            if result['corpse_hits'] > count and int(result['last_bullet']['actor'], 16) == model:
                hits += 1
        assert hits > 0 and not rider()['in_cabin'], (hits, rider())
        game.call('state.time', scale=1)
        for delay in (3, 2):
            time.sleep(delay)
            body = rider()
            x, y, z = body['pelvis']
            floor = game.call('world.ground', at=[x, lip[1] + 2, z])
            assert floor['hit'] and not body['in_cabin'], body
            clearance = y - floor['height']
            assert .02 < clearance < 1.3, (body, floor)
        print(game.build, car['model'], hits, 'hits, pelvis above street', round(clearance, 3), 'm')
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
