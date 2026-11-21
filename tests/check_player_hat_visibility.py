"""Exercise the native player fade and mission reload in a running city lab."""
import os
from pathlib import Path
import struct
import sys
import time

from check_transport_occupants import Game


def screen(game):
    shot = game.call('screenshot')['id']
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        result = game.call('screenshot.state', id=shot)
        if result['done']:
            assert result['ok'], result
            width, height = result['width'], result['height']
            data = Path(result['path']).read_bytes()[16:]
            return b''.join(data[4 * (y * width + x):4 * (y * width + x) + 3]
                            for y in range(3 * height // 8, 5 * height // 8)
                            for x in range(width // 5, 3 * width // 5))
        time.sleep(.04)
    raise AssertionError('No screenshot was returned')


def difference(a, b):
    return sum(abs(x - y) for x, y in zip(a, b, strict=True))


def check(game):
    assert game.on_foot()

    def select_outfit():
        game.tool('script', code='human_changemodel 0, "TommyDelnik.i3d"', act={'0': 'player'})

    select_outfit()
    ground = game.call('world.ground', at=[-1981, 0, 75])['height']
    game.tool('teleport', actor='player', to=[-1981, ground + .03, 75], dir=[-1, 0, 0])
    game.sleep(300)

    def read(addr, fmt):
        data = game.call('mem.read', addr=hex(addr), size=struct.calcsize('<' + fmt))['hex']
        return struct.unpack('<' + fmt, bytes.fromhex(data))[0]

    def write(addr, fmt, *values):
        game.tool('mem_write', addr=hex(addr), hex=struct.pack('<' + fmt, *values).hex())

    def wearer():
        player = int(game.player()['actor'], 16)
        hat, = game.call('frames.find', name='Tommy.rdhat_tommydelnik_scalp', limit=4)['found']
        assert hat['type'] == 10 and hat['parent_name'] == 'Tommy.neck', hat
        return player, int(hat['addr'], 16)

    def fade(player, opacity):
        native = 0x4CD7A0 if game.build == '1.0' else 0x595E10
        bits, = struct.unpack('<I', struct.pack('<f', opacity))
        write(player + 0xAB8, 'f', opacity)
        reply = game.call('native.call', at=hex(native), conv='thiscall', this=hex(player), args=[bits])
        assert reply['stack']['ok'], reply
        assert read(read(player + 0xAB4, 'I') + 0x214, 'f') == opacity

    def scale(hat, value):
        write(hat + 0x90, '3f', value, value, value)
        flags = (read(hat + 0xAC, 'I') & 0xBFFFFEC3) | 0x40000008
        write(hat + 0xAC, 'I', flags)
        update = read(read(hat, 'I') + 0x18, 'I')
        reply = game.call('native.call', at=hex(update), conv='stdcall', args=[hex(hat)])
        assert reply['stack']['ok'], reply

    def render_sweep():
        player, hat = wearer()
        assert read(hat + 0x90, 'f') == 1
        neck = next(b for b in game.call('human.skeleton', actor='player')['bones'] if b['name'] == 'neck')
        x, y, z = neck['world_pos']
        y += .18
        game.call('state.time', scale=0)
        game.call('hud.game', show=False)
        game.call('camera.set', eye=[x-2.6, y+.24, z], at=[x, y, z], fov=45, near=.05, ms=60000)
        differences = []
        prop = None
        try:
            for opacity in (1, .25, 0):
                fade(player, opacity)
                worn = screen(game)
                scale(hat, .0001)
                bare = screen(game)
                scale(hat, 1)
                differences.append(difference(worn, bare))
            assert differences[0] > 10000 and 0 < differences[1] < differences[0] / 2, differences
            assert differences[2] == 0, differences

            loose = game.call('prop.create', model='rdhat_tommydelnik_lined', at=[x, y, z+.65])
            prop = loose['prop']
            assert difference(screen(game), bare) > 10000, loose
            game.call('prop.remove', prop=prop)
            prop = None
            fade(player, 1)
            restored = screen(game)
            scale(hat, .0001)
            assert difference(restored, screen(game)) == differences[0]
        finally:
            if prop:
                game.call('prop.remove', prop=prop)
            scale(hat, 1)
            fade(player, 1)
            game.call('camera.release')
            game.call('hud.game', show=True)
            game.call('state.time', scale=1)
        return differences

    name = f'ragdoll_hat_visibility_{os.getpid()}'
    game.tool('restore', save=name)
    try:
        before = render_sweep()
        player, _ = wearer()
        game.call('state.time', scale=0)
        fade(player, .25)
        game.tool('restore', load=name)
        game.call('state.time', scale=1)
        # The stock save reloads the mission's own outfit.
        select_outfit()
        player, hat = wearer()
        assert read(read(player + 0xAB4, 'I') + 0x214, 'f') == 1
        assert read(hat + 0x90, 'f') == 1
        after = render_sweep()
        print(game.build, 'worn cap contrast at opacity 1, .25, 0:', before,
              'after mission reload:', after)
    finally:
        game.call('state.time', scale=1)
        game.tool('restore', delete=name)


if __name__ == '__main__':
    if sys.argv[1:] not in (['1.0'], ['1.2']):
        raise SystemExit('Pass one configured lab build (1.0 or 1.2).')
    lab = Game(sys.argv[1])
    try:
        check(lab)
    finally:
        lab.close()
