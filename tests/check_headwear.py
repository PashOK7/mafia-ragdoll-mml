"""Check per-wearer visibility and falling props in a running freeride lab."""
import math
import sys

from check_transport_occupants import Game
from check_scalp_meshes import check as check_scalps


def check_truck_carry(game):
    # The truck hits the physical body during its first fall. Measure the actual
    # prop along the truck's path, including travel after it becomes independent.
    previous = game.player()['vehicle']['actor']
    ground = game.call('world.ground', at=[-1981, 0, 75])['height']
    truck = game.call('world.spawn', car=61, at=[-1981, ground+.12, 75], dir=[0,0,1])['actor']
    game.made.append(truck)
    try:
        game.call('world.seat', actor='player', car=truck, seat=0)
        game.later('world.seat', actor='player', car=previous, seat=0)
        game.sleep(250)
        start = game.call('vehicle.body', car=truck)['pos'][2]+5.8
        actor = game.spawn_human('civil15', [-1981, ground+.04, start])
        game.call('human.anim', actor=actor, play='stand')
        game.sleep(160)
        def hats():
            return {p['prop']: p['pos'][2] for p in game.call('props.list')['props']
                    if p['model'].lower() == 'rdhat_civil15_lined.i3d'}
        before = set(hats())
        game.call('vehicle.move', car=truck, velocity=[0,0,18], ms=1050)
        game.call('human.hit', actor=actor, type=0, damage=1000, dir=[0,0,0])
        flight = []
        for _ in range(45):
            positions = hats()
            for prop in positions.keys()-before:
                flight.append(positions[prop]-start)
            if len(flight) >= 12:
                break
            game.sleep(24)
        assert flight and flight[0] > 1 and max(flight)-flight[0] > .6, flight
        print(game.build, 'truck impact: hat first seen', round(flight[0],2),
              'm forward, then travelled another', round(max(flight)-flight[0],2), 'm')
    finally:
        game.cleanup()


def check(game):
    check_scalps(game)
    start = game.pos('player')

    def hats():
        return {p['prop']: p for p in game.call('props.list')['props'] if p['model'].startswith('rdhat_')}

    def spawn(name, offset):
        marker = '.rdhat_' + name.lower()
        def frames():
            return {f['addr'] for f in game.call('frames.find', name=marker, limit=1000)['found']}
        before = frames()
        actor = game.spawn_human(name, [start[0]+offset, start[1]+.2, start[2]])
        game.sleep(150)
        return actor, frames()-before

    def visible(frame):
        return bool(int(game.call('frame', addr=frame)['flags'], 16) & 1)

    def hit(actor):
        game.call('human.hit', actor=actor, type=0, damage=1000, dir=[0, 0, 0])

    for model in ('civil19', 'freegang01', 'TommyHAT', 'pauliecoathat', 'czena08'):
        owner, a = spawn(model, 3)
        _, b = spawn(model, 6)
        assert len(a) == len(b) == 1, (model, a, b)
        first, second = a.pop(), b.pop()
        assert visible(first) and visible(second)
        before = hats()
        hit(owner)
        assert visible(first) and set(hats()) == set(before), (model, 'detached on activation')
        for _ in range(40):
            game.sleep(80)
            if set(hats())-set(before):
                break
        after = hats()
        new = set(after)-set(before)
        assert len(new) == 1 and not visible(first) and visible(second), (model, new)
        prop = new.pop()
        assert after[prop]['model'].lower() == 'rdhat_' + model.lower() + '_lined.i3d'
        assert after[prop]['parent'] == 'world' and 'actor' in after[prop]['physics']
        game.sleep(850)
        distance = math.dist(after[prop]['pos'], hats()[prop]['pos'])
        assert math.isfinite(distance) and distance < 20, (model, distance)
        print(game.build, model, 'independent hat, travelled', round(distance, 3), 'm')
        game.cleanup()
    bare, markers = spawn('Tommy', 9)
    assert not markers
    before = set(hats())
    hit(bare)
    game.sleep(1500)
    assert set(hats()) == before
    print(game.build, 'bareheaded Tommy spawned no hat')
    game.cleanup()
    check_truck_carry(game)


if __name__ == '__main__':
    if sys.argv[1:] not in (['1.0'], ['1.2']):
        raise SystemExit('Pass one configured lab build (1.0 or 1.2).')
    lab = Game(sys.argv[1])
    try:
        check(lab)
    finally:
        lab.close()
