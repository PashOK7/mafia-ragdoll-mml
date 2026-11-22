"""Drive a native car into the stock dog, or drop a native dog into city water."""
import argparse
import math
import re
import struct
import time

from check_transport_occupants import Game


def native(game, address, owner, *args):
    result = game.call('native.call', at=hex(address), conv='thiscall', this=owner, args=list(args), ret='uint')
    assert result['stack']['ok'], result
    return result['ret']


def word(game, address):
    raw = game.call('mem.read', addr=hex(address), size=4)['hex']
    return struct.unpack('<I', bytes.fromhex(raw))[0]


def wait_for(test, timeout=6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = test()
        if result:
            return result
        time.sleep(.025)
    raise AssertionError(f'condition did not arrive in {timeout} s')


def make_dog(game, model, position):
    build = game.build
    factory, add = {'1.0': (0x5FED70, 0x5E3220), '1.2': (0x53F7D0, 0x5A77C0)}[build]
    mission = int(game.call('game')['mission'], 16)
    engine = word(game, mission+0x24)
    prop = int(game.call('prop.create', model=model, at=position)['prop'], 16)
    actor = native(game, factory, mission, 0x15)
    vtable = word(game, actor)
    assert native(game, word(game, vtable+0x48), actor, prop) & 0xff
    # The bridge prop owns its model throughout the fixture. The actor is temporary.
    game.call('mem.write', addr=hex(actor+0x65), hex='01')
    game.call('mem.write', addr=hex(actor+0x7c), hex=struct.pack('<f', 60).hex())
    native(game, add, engine, actor)
    return actor


def drive(game):
    assert game.call('game')['mission_name'] == 'freeride'
    x, z = -1981, 75
    ground = game.call('world.ground', at=[x, 0, z])['height']
    car = game.tool('spawn', car='Bolt-Thrower', at=[x, ground+.23, z], dir=[0, 0, 1])['actor']
    game.call('world.seat', actor='player', car=car, seat=0)
    point = [x+.65, ground+.03, z+4.5]
    actor = hex(make_dog(game,'pes03.i3d',point))

    def body():
        return next((d for d in game.call('ragdoll.dogs') if int(d['actor'],16) == int(actor,16)), None)

    bound=wait_for(body)
    assert bound['bound'], bound

    def roll():
        return game.call('vehicle.body', car=car, velocity=[0,0,9])

    vehicle=roll()
    corpse = wait_for(lambda: (d if (d := body())['active'] and d['dead'] else None))
    assert math.dist(corpse['centre'], point) < 3, (corpse, point)
    print(game.build, 'native drive contact at', point, 'with initial velocity', vehicle['velocity'], corpse, flush=True)

    # If the original impact was on the bumper, point the right-hand wheels at the settled body.
    for offset in (.68, -.68):
        if body()['wheel_contacts']:
            break
        game.tool('input', steps=[{'action': 'Handbrake', 'ms': 600}])
        dog = wait_for(lambda: d if (d := body())['max_speed'] < .18 else None)
        x, y, z = dog['centre']
        surface = game.call('world.ground', at=[x-offset, y+1, z-7])
        game.call('world.teleport', actor=car, to=[x-offset, surface['height']+.23, z-7], dir=[0, 0, 1])
        roll()
        try:
            wait_for(lambda: d if (d := body())['wheel_contacts'] else None, timeout=5)
        except AssertionError:
            continue
    assert body()['wheel_contacts'] > 0, (body(), game.call('car'))
    print(game.build, 'native drivetrain produced tyre contacts on the physical dog', body(), flush=True)


def water(game, model):
    info = game.call('game')
    assert info['mission_name'] == 'freeride', info
    dry = [-1598.48, -6.2, -30.8]
    game.call('world.teleport', to=dry)
    for x in range(-1586, -1557, 7):
        at = [x, -4, -31]
        surface = game.call('world.water', at=at)
        if surface['water'] and not surface['covered'] and (surface['depth'] < 0 or surface['depth'] > 1.2):
            break
    else:
        raise AssertionError('No open water by the Little Italy embankment')

    build = game.build
    hit = 0x47BDD0 if build=='1.0' else 0x4352A0
    drop = [at[0], surface['height']+.75, at[2]]
    vec = lambda v: {'buf': 12, 'hex': struct.pack('<3f', *v).hex()}
    game.call('state.time', scale=0)
    try:
        actor = make_dog(game,model,drop)
        actual = game.pos(hex(actor))
        if math.dist(actual,drop) > .1:
            game.call('world.teleport', actor=hex(actor), to=drop)
            actual = game.pos(hex(actor))
        assert math.dist(actual[::2],drop[::2]) < .15 and abs(actual[1]-surface['height']) < 1.75, (drop,actual)
        native(game, hit, actor, 0, vec([0, 0, 0]), vec(drop), 0, {'float': 120}, 0, 0, 0)
    finally:
        game.call('state.time', scale=1)

    def dog():
        return next((d for d in game.call('ragdoll.dogs') if int(d['actor'], 16) == actor), None)

    wet = []
    deadline = time.monotonic() + 9
    while time.monotonic() < deadline:
        state = dog()
        jolt = game.call('ragdoll.jolt')
        count = re.search(r'(\d+) in water', jolt)
        if state and state['active'] and count and int(count[1]) > 0:
            wet.append((state, int(count[1])))
            if wet[-1][0]['age'] - wet[0][0]['age'] >= 4:
                break
        time.sleep(.08)
    assert wet and wet[-1][0]['age']-wet[0][0]['age'] >= 4, (dog(), wet[-2:])
    assert wet[-1][0]['dead'] and wet[-1][0]['centre'][1] > surface['height']-.45, (surface, wet[-1])
    print(build, model, 'remained at the native water surface', surface['height'], 'wet parts', wet[-1][1], wet[-1][0], flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('build', choices=['1.0', '1.2'])
    parser.add_argument('scenario', choices=['drive', 'water'])
    parser.add_argument('--model', choices=['pes02.i3d','pes03.i3d'], default='pes03.i3d')
    args = parser.parse_args()
    lab = Game(args.build)
    try:
        if args.scenario == 'drive':
            drive(lab)
        else:
            water(lab,args.model)
    finally:
        lab.close()
