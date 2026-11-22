"""Carry a native metro5 passenger through both platform barriers in a fresh city lab."""
import struct
import sys
import time

from check_transport_occupants import Game


def check(game):
    build = game.build
    assert game.call('game')['mission_name'] == 'freeride'
    addresses = {
        '1.0': (0x49EA60, 0x492000, 0x4E8EE0, 0x4D5160),
        '1.2': (0x57E500, 0x571B90, 0x497C90, 0x484230),
    }
    leave_car, enter_railway, run_railway, kill_rider = addresses[build]

    def read(addr, fmt):
        raw = game.call('mem.read', addr=hex(addr), size=struct.calcsize('<'+fmt))['hex']
        return struct.unpack('<'+fmt, bytes.fromhex(raw))

    def write(addr, fmt, value):
        game.call('mem.write', addr=hex(addr), hex=struct.pack('<'+fmt, value).hex())

    def native(at, owner, *args):
        result = game.call('native.call', at=hex(at), conv='thiscall', this=owner, args=list(args))
        assert result['stack']['ok'], result

    def wagon_pose():
        return game.call('frame', addr=hex(wagon))

    def passengers():
        lo, hi = read(train+0x254, '2I')
        if not 0 < hi-lo <= 4096:
            return
        for rec in read(lo, f'{(hi-lo)//4}I'):
            phase, model, _, _, _, _, car = read(rec, '7I')
            if model and car == 1 and phase in (9, 19):
                yield rec, model

    def wait_for(observe, timeout):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if result := observe():
                return result
            time.sleep(.06)
        raise AssertionError(('condition timed out', observe.__name__))

    def barrier(z, x, y):
        hit = game.call('world.ray', **{
            'from': [x-.207, y+.039, z-1.485], 'to': [x+.207, y-.039, z+1.485], 'what': 'static'
        })
        assert hit['hit'] and hit['flags'] == '0x21000083', hit
        return hit['prim']

    gates = {
        'entrance': (barrier(-26.759, -1615.951, 1.55), -37, (-34, -25)),
        'exit': (barrier(-72.529, -1622.321, 2.45), -84, (-82, -71)),
    }
    train = int(next(a['actor'] for a in game.call('actors') if a['type'] == 8 and a['model'] == 'metro5'), 16)
    wagon, = read(read(train+0xC4, 'I')[0], 'I')
    game.call('state.time', scale=0)
    try:
        player = game.player()
        actor = int(player['actor'], 16)
        assert player['alive'], player
        if player['vehicle']:
            assert player['vehicle']['type'] == 4, player
            native(leave_car, actor, player['frame'])
        pose = wagon_pose()
        to = [x+.674*up-1.75*forward for x, up, forward in zip(pose['pos'], pose['axes'][1], pose['axes'][2])]
        game.call('world.teleport', actor='player', to=to)
        write(actor+0x584, 'I', 1)
        write(actor+0x9C, 'I', train)
        native(enter_railway, actor, 1)
        assert int(game.player()['vehicle']['actor'],16) == train
        game.call('state.time', scale=1)
        rec, model = wait_for(lambda: next(passengers(), None), 10)
        game.call('state.time', scale=0)
        game.call('scene.focus', at=wagon_pose()['pos'], radius=9)

        def stationary_mesh():
            return any(p['visual']=='metro5_vagon0.metro' and not p['moving']
                       for p in game.call('scene.pieces', limit=500)['list'])

        wait_for(stationary_mesh, 5)
        write(train+0xD4, 'f', 7.0)
        native(run_railway, train, 0)
        native(kill_rider, rec, train)
        game.call('state.time', scale=1)

        def trace():
            return game.call('ragdoll.railtrace', model=hex(model))

        def outside_sources(rows):
            return {s['outside_visual'] for s in rows if s['outside_visual'] != '0x00000000'}

        def known_cabin_mesh(source):
            if read(int(source,16), 'I')[0] & 0xff == 0x83:
                return False
            return int(game.call('frame', addr=source).get('parent', '0'), 16) == wagon

        def early_trace():
            rows = trace()
            return rows if rows and rows[-1]['age'] >= 3 else []

        early = wait_for(early_trace, 8)
        sources = outside_sources(early)
        assert all(not known_cabin_mesh(source) for source in sources), sources
        print(build, 'stationary cabin geometry stayed out of the ragdoll scenery', flush=True)
        for label, (primitive, past, window) in gates.items():
            wait_for(lambda: wagon_pose()['pos'][2] < past, 52)
            rows = [s for s in trace() if window[0] < s['origin'][2] < window[1]]
            assert rows and any(s['barriers_rejected'] for s in rows), (label, rows[-5:])
            assert primitive not in outside_sources(rows), (label, primitive)
            peak = max(s['end_approach'] for s in rows)
            assert peak < 3, (label, peak)
            print(build, label, 'native wall', primitive, 'rejections',
                  sum(s['barriers_rejected'] for s in rows), 'peak cabin approach', peak, flush=True)
        active = game.call('ragdoll.state')['occupants']
        assert any(int(o['model'], 16)==model and o['in_cabin'] for o in active), active
        assert game.player()['alive']
    finally:
        game.call('scene.focus', off=True)
        game.call('state.time', scale=1)


if __name__ == '__main__':
    if sys.argv[1:] not in (['1.0'], ['1.2']):
        raise SystemExit('Pass one configured lab build (1.0 or 1.2).')
    lab = Game(sys.argv[1])
    try:
        check(lab)
    finally:
        lab.close()
