"""Run in a fresh Motel Clark mission with ragdoll loaded. Uses the stock C_dog."""
import math
import os
import struct
import sys
import time

from check_transport_occupants import Game


def check(game):
    assert game.call('game')['mission_name'] == 'mise04-motorest'
    addresses = {
        '1.0': (0x47BDD0, 0x47F950, 0x5E3DF0, 0x5E6760),
        '1.2': (0x4352A0, 0x438E20, 0x5A84A0, 0x5AAE10),
    }
    hit_at, collision_at, shoot_at, explosion_at = addresses[game.build]

    def tracked():
        deadline = time.monotonic() + 3
        while True:
            dogs = game.call('ragdoll.dogs')
            if dogs:
                dog, = dogs
                assert dog['bound'], dog
                return dog
            assert time.monotonic() < deadline, 'No dog was tracked'
            time.sleep(.04)

    def read(addr, fmt):
        raw = game.call('mem.read', addr=hex(addr), size=struct.calcsize('<' + fmt))['hex']
        return struct.unpack('<' + fmt, bytes.fromhex(raw))[0]

    def wait_for_active():
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            dog = tracked()
            if dog['active'] and dog['age'] >= .4:
                assert dog['parts'] == 18 and all(map(math.isfinite, dog['centre'])), dog
                assert 0 <= dog['pose_error'] < .015 and dog['max_speed'] < 18, dog
                assert dog['head']['mesh_points'] >= 20, dog
                assert dog['head']['pitch'] <= 36, dog
                return dog
            time.sleep(.05)
        raise AssertionError(game.call('ragdoll.dogs'))

    def native(at, this, *args):
        result = game.call('native.call', at=hex(at), conv='thiscall', this=this, args=list(args))
        assert result['stack']['ok'], result
        return result

    def vector(*xyz):
        return {'buf': 12, 'hex': struct.pack('<3f', *xyz).hex()}

    def settled():
        deadline = time.monotonic() + 9
        while time.monotonic() < deadline:
            dog = tracked()
            if dog['sleeping']:
                x, y, z = dog['centre']
                ground = game.call('world.ground', at=[x, y + .35, z])
                assert ground['hit'] and .06 < y - ground['height'] < .4, (dog, ground)
                assert dog['max_speed'] < .04 and dog['pose_error'] < .015, dog
                head = dog['head']
                assert head['mesh_points'] >= 20 and head['pitch'] <= 34, dog
                for point in (head['lowest'], head['muzzle']):
                    floor = game.call('world.ground', at=[point[0], point[1] + .35, point[2]])
                    assert floor['hit'] and point[1] >= floor['height'] - .008, (point, floor, dog)
                return dog
            time.sleep(.1)
        raise AssertionError(('Dog did not settle', tracked()))

    def check_corpse_shot(dog, engine, player):
        # Observe TickShoot's static impact branch. A miss must reach the ground.
        # A shot resolved by a dog segment must stop before that same ground.
        tap_at = {'1.0': '5E5975', '1.2': '5AA025'}[game.build]
        tap = game.tool('tap', cmd=tap_at + ' n=0').split()[1].rstrip(':')

        def ground_impacts():
            return int(game.tool('tap', cmd='show ' + tap).split(': ', 1)[1].split()[0])

        def fire(x, y, z):
            native(shoot_at, engine, player, *[{'float': f} for f in (x, y, z, 60, -160, 0, 100)], 0, 0, 1)

        try:
            x, y, z = dog['centre']
            assert not dog['corpse_hits'] and dog['sleeping'], dog
            fire(x - .375, y + 1, z + 1.5)
            time.sleep(.25)
            assert ground_impacts() == 1 and tracked()['corpse_hits'] == 0
            origin = [x - .375, y + 1, z]
            static = game.call('world.ray', **{'from': origin, 'to': [x + .75, y - 2, z], 'what': 'static'})
            moving = game.call('world.ray', **{'from': origin, 'to': [x, y, z], 'what': 'moving'})
            assert static['hit'] and static['pos'][1] < y, static
            assert not moving['hit'] or moving.get('actor') == dog['actor'], moving
            fire(*origin)
            deadline = time.monotonic() + 1
            result = tracked()
            while result['corpse_hits'] == 0 and time.monotonic() < deadline:
                time.sleep(.02)
                result = tracked()
            assert result['corpse_hits'] == 1 and result['last_hit_part'], result
            assert not result['sleeping'] and result['max_speed'] > .05, result
            assert abs(result['last_shot_impulse']-31.8) < .01, result
            assert result['blood_effects'] == int(gore > 0), result
            time.sleep(.25)
            assert ground_impacts() == 1 and tracked()['corpse_hits'] == 1, tracked()
            print(game.build, 'settled corpse intercepted one shot and woke', result,
                  'native dog volume still in the world' if moving['hit'] else 'native dog volume absent here')
        finally:
            game.tool('tap', cmd='off ' + tap)

    def check_blast_and_save(engine, dead_name):
        before = settled()
        x, y, z = before['centre']
        for offset, hits in ((12, 0), (2, 1)):
            native(explosion_at, engine, 0, vector(x + offset, y, z),
                   {'float': 3}, {'float': 180}, 0, 0, 0, 0)
            after = tracked()
            assert after['blast_hits'] == hits, after
            if hits:
                assert not after['sleeping'] and after['max_speed'] > 1, after
            else:
                assert after['sleeping'], after
        saved = settled()
        assert math.dist(saved['centre'], before['centre']) > .15, (before, saved)
        result = game.tool('restore', save=dead_name)
        assert result['mods_state'], result
        game.tool('restore', load=dead_name)
        deadline = time.monotonic() + 9
        while time.monotonic() < deadline:
            result = tracked()
            if result['active'] and result['restored']:
                break
            time.sleep(.05)
        assert result['restored'] and result['sleeping'], result
        assert math.dist(result['centre'], saved['centre']) < .005, (saved, result)
        assert result['blood_effects'] == saved['blood_effects'], (saved, result)
        print(game.build, 'blast moved corpse and native save restored the displaced body', result)

    def check_shotgun(engine, player):
        corpse=settled()
        x, y, z=corpse['centre']
        seq=game.latest()
        native(shoot_at, engine, player, *[{'float': f} for f in (x-.38, y+.50, z, 65, -85, 0, 150)], 0, 0, 8)
        time.sleep(.14)
        result=tracked()
        events=game.call('events.since', seq=seq, types='SHOT')['events']
        assert any(ev['data']['shots']==8 for ev in events), events
        assert 4 <= result['corpse_hits'] - corpse['corpse_hits'] <= 8, result
        assert not result['sleeping'] and 25 <= result['last_shot_impulse'] <= 36, result
        print(game.build, 'native close-range pellet queue reached the corpse:', result)

    name = f'ragdoll_pes_{os.getpid()}'
    dead_name = name + '_pose'
    game.tool('restore', save=name)
    try:
        dog = tracked()
        actor = dog['actor']
        player = game.player()['actor']
        assert not dog['active']
        health = read(int(actor, 16) + 0xD0, 'f')
        native(hit_at, actor, 0, vector(1, 0, 0), vector(*dog['centre']), 0, {'float': 2}, player, 0, 0)
        assert read(int(actor, 16) + 0xD0, 'f') == health - 2
        assert not tracked()['active']

        native(hit_at, actor, 0, vector(1, 0, 0), vector(*dog['centre']), 0, {'float': 20}, player, 0, 0)
        fallen = wait_for_active()
        assert not fallen['dead'] and read(int(actor, 16) + 0xD0, 'f') == health - 22, fallen
        recovered = False
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            observation = tracked()
            recovered |= observation['recovering']
            if recovered and not observation['recovering'] and not observation['active']:
                break
            time.sleep(.04)
        assert recovered and not observation['recovering'] and not observation['active'], observation
        assert read(int(actor, 16) + 0xD0, 'f') == health - 22, observation
        print(game.build, 'heavy nonlethal impact fell and recovered without changing native damage', observation)

        game.tool('restore', load=name)
        actor = tracked()['actor']
        player = game.player()['actor']

        def shot_path():
            x, y, z = next(d['pos'] for d in game.call('actors') if d['actor'] == actor)
            return [x - 2, y + .42, z], [x + 2, y + .42, z]

        mission = int(game.call('game')['mission'], 16)
        engine = read(mission + 0x24, 'I')
        gore = read(mission + 0x9C, 'i')
        seq = game.latest()
        deadline = time.monotonic() + 3
        while read(int(actor, 16) + 0xD0, 'f') > 0:
            start, end = shot_path()
            ray = game.call('world.ray', **{'from': start, 'to': end, 'what': 'moving'})
            if ray.get('actor') == actor:
                native(shoot_at, engine, player, *[{'float': f} for f in (*start, 160, 0, 0, 180)], 0, 0, 1)
            assert time.monotonic() < deadline, ray
            time.sleep(.08)
        result = wait_for_active()
        assert read(int(actor, 16) + 0xD0, 'f') < 0
        assert 34.7 < result['last_shot_impulse'] <= 36, result
        hits = game.call('events.since', seq=seq, types='BULLET_HIT')['events']
        assert any(hit['data'].get('struck') == actor for hit in hits), hits
        print(game.build, 'shot', result)
        result = settled()
        print(game.build, 'settled after shot', result)
        check_corpse_shot(result, engine, player)
        check_blast_and_save(engine, dead_name)
        check_shotgun(engine, game.player()['actor'])

        game.tool('restore', load=name)
        dog = tracked()
        assert not dog['active'] and read(int(dog['actor'], 16) + 0xD0, 'f') == health, dog
        car = next(a['actor'] for a in game.call('actors') if a['type'] == 4)
        native(collision_at, dog['actor'], car, 0, 0, 0, 0, 0)
        result = wait_for_active()
        assert read(int(dog['actor'], 16) + 0xD0, 'f') < 0
        print(game.build, 'native vehicle collision', result)
        print(game.build, 'settled after vehicle collision', settled())

        game.tool('restore', load=name)
        dog = tracked()
        assert not dog['active'] and read(int(dog['actor'], 16) + 0xD0, 'f') == health, dog
        print(game.build, 'mission restore returned the dog to native animation')
    finally:
        game.tool('restore', delete=dead_name)
        game.tool('restore', delete=name)


if __name__ == '__main__':
    if sys.argv[1:] not in (['1.0'], ['1.2']):
        raise SystemExit('Pass one configured lab build (1.0 or 1.2).')
    lab = Game(sys.argv[1])
    try:
        check(lab)
    finally:
        lab.close()
