"""Check human death while running, switching clips and leaving a native car."""
import math
import re
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'MafiaModLoader/tests/game'))
from mml_game import Game


def wait_for(read, ready, timeout=8):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        val = read()
        if ready(val):
            return val
        time.sleep(.02)
    raise AssertionError(val)


def start_motion(g, actor):
    line = next(line for line in str(g.tool('log_tail', lines=100)).splitlines()
                if f'ragdoll started on person {actor[2:]}' in line)
    match = re.search(r'pelvis ([-.\d ]+) m/s; root ([-.\d ]+) m/s', line)
    assert match, line
    pelvis = tuple(map(float, match[1].split()))
    root = tuple(map(float, match[2].split()))
    return pelvis, root


def animation(g, actor):
    return g.call('human.anim_state', actor=actor)


def shot(g, actor, damage, direction):
    trunk = next(b['world_pos'] for b in g.call('human.skeleton', actor=actor)['bones'] if b['name'] == 'back3')
    return g.call('human.hit', actor=actor, type=0, damage=damage, dir=direction, pos=trunk, part=3)


def exit_death(g, z, passenger_burst):
    model = 'Wright Coupe Gangster' if passenger_burst else 'Bolt-Thrower'
    car = g.tool('spawn', car=model, at=[-1978, -4, z], dir=[0, 0, 1])['actor']
    actor = g.spawn_human('FREEgang02', [-1981, -4, z])
    g.call('world.seat', actor=actor, car=car, seat=int(passenger_burst))
    g.sleep(450)
    g.call('state.time', scale=.4)
    g.call('human.order', actor=actor, code='enemy_usecar\nend')
    before = wait_for(lambda: animation(g, actor),
                      lambda a: 284 <= a['playing'] <= 291 and a['time_ms'] >= (1850 if passenger_burst else 1250))
    g.call('human.health', actor=actor, set=70 if passenger_burst else 20)
    direction = [0, 0, 1]
    if passenger_burst:
        contact = next(b['world_pos'] for b in g.call('human.skeleton', actor=actor)['bones'] if b['name'] == 'back3')
        for _ in range(3):
            assert g.call('human.hit', actor=actor, type=0, damage=12, dir=direction, pos=contact, part=5)['alive']
        g.sleep(65)
    after = animation(g, actor)
    if passenger_burst:
        assert after['playing'] != before['playing'], (before, after)
    assert not shot(g, actor, 100, direction)['alive']
    pelvis, root = start_motion(g, actor)
    print(g.build, 'doorway', before['playing'], before['time_ms'], 'to', after['playing'],
          'pelvis', pelvis, 'root', root, flush=True)
    assert math.hypot(*root) < .02 and abs(pelvis[1]) < 1.6, (pelvis, root)
    g.call('state.time', scale=1)
    g.sleep(500)


def running_death(g):
    assert g.on_foot()
    x, y, z = g.pos('player')
    actor = g.spawn_human('Paulie', [x - 1.5, y, z - 7])
    g.call('human.go', actor=actor, to=[x - 1.5, y, z - 17], run=True)
    wait_for(lambda: g.pos(actor), lambda p: p[2] < z - 8.5)
    g.call('human.health', actor=actor, set=20)
    assert not shot(g, actor, 80, [0, 0, 0])['alive']
    pelvis, root = start_motion(g, actor)
    print(g.build, 'running', pelvis, root, flush=True)
    assert root[2] < -2 and pelvis[2] < -1, (pelvis, root)


def falling_death(g):
    ground = g.call('world.ground', at=[-1978, 0, 52])['height']
    actor = g.spawn_human('Paulie', [-1978, ground, 52])
    g.call('human.health', actor=actor, set=20)
    g.sleep(280)
    g.call('world.teleport', actor=actor, to=[-1978, ground + 5, 52])
    g.call('state.time', scale=0)
    try:
        # Seed the native airborne fields while the engine clock is held. A
        # teleported AI with no movement order can otherwise snap to ground.
        for offset, data in ((0x40C, struct.pack('<i', 3)), (0x418, struct.pack('<f', 4))):
            g.call('mem.write', addr=hex(int(actor, 16) + offset), hex=data.hex())
        assert animation(g, actor)['state'] == 3
        assert not shot(g, actor, 80, [0, 0, 0])['alive']
        pelvis, root = start_motion(g, actor)
    finally:
        g.call('state.time', scale=1)
    print(g.build, 'falling', pelvis, root, flush=True)
    assert -5 < root[1] <= -4 and math.isclose(root[1], pelvis[1], abs_tol=.02), (pelvis, root)


def death_in_moving_car(g):
    car = g.tool('spawn', car='Bolt-Thrower', at=[-1978, -4, 72], dir=[0, 0, 1])['actor']
    actor = g.spawn_human('civil15', [-1981, -4, 72])
    g.call('world.seat', actor=actor, car=car, seat=0)
    g.call('human.health', actor=actor, set=20)
    g.sleep(550)
    g.call('vehicle.move', car=car, velocity=[0, 0, 7], ms=1800)
    wait_for(lambda: g.call('vehicle.body', car=car)['velocity'], lambda v: v[2] > 6)
    assert not shot(g, actor, 80, [0, 0, 0])['alive']
    pelvis, root = start_motion(g, actor)
    print(g.build, 'moving car', pelvis, root, flush=True)
    assert root[2] > 6 and pelvis[2] > 5, (pelvis, root)
    g.call('vehicle.move', car=car, ms=0)


def check(g):
    g.call('human.invulnerable', on=True)
    try:
        exit_death(g, 7, False)
        exit_death(g, 19, True)
        running_death(g)
        falling_death(g)
        death_in_moving_car(g)
    finally:
        g.call('state.time', scale=1)
        g.call('human.invulnerable', on=False)


if __name__ == '__main__':
    if sys.argv[1:] not in (['1.0'], ['1.2']):
        raise SystemExit('Pass one running full-city Freeride lab.')
    game = Game(sys.argv[1])
    try:
        check(game)
    finally:
        game.close()
