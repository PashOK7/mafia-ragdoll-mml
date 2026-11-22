"""Compare native acceleration and braking with the same passenger death and physics enabled or absent."""
import math
import os
import struct
import sys
import time

from check_transport_occupants import Game


def check(game):
    new = game.build == '1.2'
    kill = 0x484230 if new else 0x4D5160
    select_state = 0x497C90 if new else 0x4E8EE0
    save = f'rail_motion_{os.getpid()}'

    def read(at, fmt):
        raw = game.call('mem.read', addr=hex(at), size=struct.calcsize('<'+fmt))['hex']
        return struct.unpack('<'+fmt, bytes.fromhex(raw))

    def native(at, actor, *args):
        result = game.call('native.call', at=hex(at), conv='thiscall', this=actor, args=list(args))
        assert result['stack']['ok'], result

    def rails(is_train, name=None):
        for a in game.call('actors'):
            if a['type'] != 8 or (name and a['model'] != name):
                continue
            carrier = int(a['actor'], 16)
            wagons, = read(carrier+0x1a8,'I')
            speed, _, state = read(carrier+0xd4,'fII')
            if bool(wagons) != is_train or (not name and (state or speed<5)):
                continue
            lo, hi = read(carrier+0x254,'2I')
            if not 0 < hi-lo <= 4096:
                continue
            riders = read(lo, f'{(hi-lo)//4}I')
            for rider in riders:
                if rider == carrier+0x2b0:
                    continue
                phase, model, _, _, _, _, carriage = read(rider,'7I')
                if phase in (9,19) and model and ((is_train and carriage>0) or (not is_train and carriage==0)):
                    yield a, carrier, rider, model
                    break

    def phase(carrier, expected_direction, samples=12):
        readings = []
        for _ in range(samples):
            speed, _, state = read(carrier+0xd4,'fII')
            readings.append((speed,state))
            time.sleep(.06)
        assert expected_direction*(readings[-1][0]-readings[0][0]) > .08, readings
        return readings

    def experiment(train, name, physics, braking):
        game.call('state.time', scale=1)
        game.tool('restore', load=save)
        game.call('state.time', scale=0)
        if physics:
            game.call('mod.load', id='ragdoll')
            assert game.call('ragdoll.state')['enabled']
        else:
            mods = game.call('mods.list')
            if any(mod['id']=='ragdoll' and mod['loaded'] for mod in mods):
                game.call('mod.unload', id='ragdoll')
        a, carrier, rider, model = next(rails(train,name))
        fx, _, fz = a['facing']
        near = [a['pos'][0]+12*fz, a['pos'][1]+2, a['pos'][2]-12*fx]
        game.call('world.teleport', to=near)
        game.call('camera.set', follow=a['actor'], eye=[4,3,-9], at=[0,1,0])
        # Drive under the normal native route controller, from below or above
        # its current safe speed. Both samples use a fresh passenger death.
        start = (19 if train else 14) if braking else 1.5
        game.call('mem.write', addr=hex(carrier+0xd4), hex=struct.pack('<f', start).hex())
        native(select_state, carrier, 0)
        native(kill, rider, carrier)
        if physics:
            def body():
                return next(o for o in game.call('ragdoll.state')['occupants'] if int(o['model'],16)==model)
            created=body()
            assert created['in_cabin'] and abs(math.hypot(*created['pelvis_velocity'])-start)<.03, created
        game.call('state.time', scale=.5)
        samples=phase(carrier,-1 if braking else 1)
        if physics:
            after=body()
            assert 0 <= after['drawn_neck_error'] < .025, after
            assert after['in_cabin'], after
        return samples

    # The native passenger roster populates after the traffic managers start.
    targets = {}
    deadline = time.monotonic()+40
    while len(targets)<2 and time.monotonic()<deadline:
        for train in (False,True):
            if train in targets:
                continue
            candidate = next(rails(train), None)
            if candidate:
                targets[train] = candidate[0]['model']
        time.sleep(.15)
    assert len(targets)==2, f'Only found these active occupied railway types: {targets}'
    game.tool('restore', save=save)
    try:
        for train,name in targets.items():
            for braking in (False,True):
                stock=experiment(train,name,False,braking)
                physics=experiment(train,name,True,braking)
                print(game.build,name,'braking' if braking else 'acceleration',
                      'native',stock,'physical',physics,flush=True)
                assert abs(stock[-1][0]-physics[-1][0]) < .65, (stock,physics)
    finally:
        game.call('state.time', scale=1)
        game.tool('restore', delete=save)


if __name__ == '__main__':
    if sys.argv[1:] not in (['1.0'],['1.2']):
        raise SystemExit('Pass one configured lab build (1.0 or 1.2).')
    lab = Game(sys.argv[1])
    try:
        check(lab)
    finally:
        lab.close()
