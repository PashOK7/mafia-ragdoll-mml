"""Exercise live and already simulated occupants in a fresh city freeride lab.

The wrecks stay in the disposable mission. The bridge cannot remove a seated corpse.
"""
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'MafiaModLoader/tests/game'))
from mml_game import Game


VISIBLE_BONES = ('pelvis', 'neck', 'r_hand', 'l_hand')


def until(read, ready):
    deadline = time.monotonic() + 10
    value = read()
    while not ready(value) and time.monotonic() < deadline:
        time.sleep(.04)
        value = read()
    assert ready(value), value
    return value


def bodies(game):
    return {body['actor']: body for body in game.call('ragdoll.state')['ragdolls']}


def vehicle(game, actor):
    field = game.call('mem.read', addr=hex(int(actor, 16) + 0x98), size=4)['hex']
    return int.from_bytes(bytes.fromhex(field), 'little')


def bones(game, actor):
    return {b['name']: b['frame'] for b in game.call('human.skeleton', actor=actor)['bones']
            if b['name'] in VISIBLE_BONES}


def check(game, car_name, z, already_dead=None, death_age=1.0):
    x = -1981
    observer = game.call('world.ground', at=[x, 3, z - 12])
    ground = game.call('world.ground', at=[x, 3, z])
    assert ground['hit'] and observer['hit']
    game.tool('teleport', to=[x, observer['height'] + .2, z - 12])
    car = game.tool('spawn', car=car_name, at=[x, ground['height'] + .25, z], dir=[0, 0, 1])['actor']
    riders = []
    for seat, skin in enumerate(('civil15', 'civil19')):
        actor = game.tool('spawn', human=skin, at=[x + 2 + seat, ground['height'] + .1, z])['actor']
        riders.append(actor)
        game.call('world.seat', actor=actor, car=car, seat=seat)

    time.sleep(.35)
    states = {a['actor']: a for a in game.call('actors')}
    assert all(states[a]['alive'] and vehicle(game, a) == int(car, 16) for a in riders)
    if already_dead is not None:
        corpse = riders[already_dead]
        assert not game.call('human.hit', actor=corpse, damage=900, type=0, dir=[0, 0, 1], part=3)['alive']
        before = until(lambda: bodies(game).get(corpse), lambda b: b and b['age'] >= death_age)

    old_bones = {a: bones(game, a) for a in riders}
    seq = game.latest()
    game.tool('script', code='car_explosion 0', act={'0': car})
    after = until(lambda: bodies(game), lambda b: all(a in b and b[a]['age'] > .10 for a in riders))
    if already_dead is not None:
        retained = after[corpse]
        assert retained['body_serial'] == before['body_serial'] and retained['age'] >= before['age'], (before, retained)

    wreck = vehicle(game, riders[0])
    assert wreck and wreck != int(car, 16)
    now = {a['actor']: a for a in game.call('actors')}
    wreck_actor = next(a for a in now.values() if int(a['actor'], 16) == wreck)
    for actor in riders:
        frame = now[actor]['frame']
        assert not now[actor]['alive'] and frame == states[actor]['frame'] and vehicle(game, actor) == wreck
        assert game.call('model.file', frame=frame)['file'].lower().endswith(r'\ohorelec01.i3d')
        assert game.call('frame', addr=frame)['parent'] == wreck_actor['frame']
        assert old_bones[actor] != bones(game, actor)

    after = until(lambda: bodies(game), lambda b: all(a in b and b[a]['age'] > 3 for a in riders))
    mesh = game.call('actor.meshes', actor=wreck_actor['actor'])['list']
    cabin = game.call('ragdoll.vehicle', car=wreck_actor['actor'])
    assert ' solid panels' in cabin and int(cabin.split('cabin: ', 1)[1].split()[0]) > 0, cabin
    for actor in riders:
        body = after[actor]
        assert all(math.isfinite(v) for v in body['pelvis'] + body['head'])
        x, y, _ = body['head']
        assert min(m['lo'][0] for m in mesh) - .10 <= x <= max(m['hi'][0] for m in mesh) + .10
        assert y < max(m['hi'][1] for m in mesh)
        report = game.call('ragdoll.inspect', actor=actor)
        assert f'seated in car {wreck:08x}' in report.lower(), report
        errors = json.loads(next(line.split(': ', 1)[1] for line in report.splitlines() if line.startswith('render errors')))
        assert max(errors[b][0] for b in VISIBLE_BONES) < 20 and max(errors[b][1] for b in VISIBLE_BONES) < 5, errors
        print(game.build, car_name, actor, 'body', body['body_serial'], 'skin error cm/deg',
              {b: errors[b] for b in VISIBLE_BONES}, flush=True)

    events = game.call('events.since', seq=seq, types='EXPLOSION,CAR_EXPLODED,MOD_FAULTED')['events']
    assert all(e['type'] != 'MOD_FAULTED' for e in events), events
    assert [e['type'] for e in events if e['data'].get('actor') == car] == ['EXPLOSION', 'CAR_EXPLODED']


if __name__ == '__main__':
    if sys.argv[1:] not in (['1.0'], ['1.2']):
        raise SystemExit('Pass one configured lab build (1.0 or 1.2).')
    game = Game(sys.argv[1])
    try:
        assert game.call('game')['mission_state'] == 'running'
        assert game.call('ragdoll.state')['enabled']
        check(game, 'Bolt-Thrower', 83)
        check(game, 'Bolt-Thrower', 122, already_dead=0)
        check(game, 'Masseur Taxi', 104, already_dead=1, death_age=.20)
    finally:
        game.close()
