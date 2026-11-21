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
        '1.0': (0x47BDD0, 0x47F950, 0x5E3DF0),
        '1.2': (0x4352A0, 0x438E20, 0x5A84A0),
    }
    hit_at, collision_at, shoot_at = addresses[game.build]

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
                assert dog['parts'] == 17 and all(map(math.isfinite, dog['centre'])), dog
                return dog
            time.sleep(.05)
        raise AssertionError(game.call('ragdoll.dogs'))

    def native(at, this, *args):
        result = game.call('native.call', at=hex(at), conv='thiscall', this=this, args=list(args))
        assert result['stack']['ok'], result
        return result

    def vector(*xyz):
        return {'buf': 12, 'hex': struct.pack('<3f', *xyz).hex()}

    name = f'ragdoll_pes_{os.getpid()}'
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

        def shot_path():
            x, y, z = tracked()['centre']
            return [x - 2, y, z], [x + 2, y, z]

        deadline = time.monotonic() + 3
        while True:
            start, end = shot_path()
            ray = game.call('world.ray', **{'from': start, 'to': end, 'what': 'moving'})
            if ray.get('actor') == actor:
                break
            assert time.monotonic() < deadline, ray
            time.sleep(.03)
        mission = int(game.call('game')['mission'], 16)
        engine = read(mission + 0x24, 'I')
        seq = game.latest()
        native(shoot_at, engine, player, *[{'float': f} for f in (*start, 160, 0, 0, 180)], 0, 0, 1)
        result = wait_for_active()
        assert read(int(actor, 16) + 0xD0, 'f') < 0
        hits = game.call('events.since', seq=seq, types='BULLET_HIT')['events']
        assert any(hit['data'].get('struck') == actor for hit in hits), hits
        print(game.build, 'shot', result)

        game.tool('restore', load=name)
        dog = tracked()
        assert not dog['active'] and read(int(dog['actor'], 16) + 0xD0, 'f') == health, dog
        car = next(a['actor'] for a in game.call('actors') if a['type'] == 4)
        native(collision_at, dog['actor'], car, 0, 0, 0, 0, 0)
        result = wait_for_active()
        assert read(int(dog['actor'], 16) + 0xD0, 'f') < 0
        print(game.build, 'native vehicle collision', result)

        game.tool('restore', load=name)
        dog = tracked()
        assert not dog['active'] and read(int(dog['actor'], 16) + 0xD0, 'f') == health, dog
        print(game.build, 'mission restore returned the dog to native animation')
    finally:
        game.tool('restore', delete=name)


if __name__ == '__main__':
    if sys.argv[1:] not in (['1.0'], ['1.2']):
        raise SystemExit('Pass one configured lab build (1.0 or 1.2).')
    lab = Game(sys.argv[1])
    try:
        check(lab)
    finally:
        lab.close()
