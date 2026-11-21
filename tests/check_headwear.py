"""Check per-wearer visibility and falling props in a running freeride lab."""
import math
import sys

from check_transport_occupants import Game
from check_scalp_meshes import check as check_scalps


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
        game.sleep(80)

    for model in ('civil19', 'freegang01', 'TommyHAT', 'pauliecoathat', 'czena08'):
        owner, a = spawn(model, 3)
        _, b = spawn(model, 6)
        assert len(a) == len(b) == 1, (model, a, b)
        first, second = a.pop(), b.pop()
        assert visible(first) and visible(second)
        before = hats()
        hit(owner)
        after = hats()
        new = set(after)-set(before)
        assert len(new) == 1 and not visible(first) and visible(second), (model, new)
        prop = new.pop()
        assert after[prop]['model'].lower() == 'rdhat_' + model.lower() + '.i3d'
        assert after[prop]['parent'] == 'world' and 'actor' in after[prop]['physics']
        game.sleep(850)
        distance = math.dist(after[prop]['pos'], hats()[prop]['pos'])
        assert distance > .1, (model, distance)
        print(game.build, model, 'independent hat, travelled', round(distance, 3), 'm')
        game.cleanup()
    bare, markers = spawn('Tommy', 9)
    assert not markers
    before = set(hats())
    hit(bare)
    assert set(hats()) == before
    print(game.build, 'bareheaded Tommy spawned no hat')


if __name__ == '__main__':
    if sys.argv[1:] not in (['1.0'], ['1.2']):
        raise SystemExit('Pass one configured lab build (1.0 or 1.2).')
    lab = Game(sys.argv[1])
    try:
        check(lab)
    finally:
        lab.close()
