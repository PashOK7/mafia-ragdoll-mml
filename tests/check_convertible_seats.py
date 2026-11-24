"""Exercise ejection and landing, followed by an explosion or normal Action.

Run against a disposable city freeride with the matching local DLL already
loaded. The map resolves private test setup calls. Gameplay sees the same
corpse, car and normal seat transitions throughout each scenario.
"""
from hashlib import sha256
import json
import math
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
MML = ROOT.parent / 'MafiaModLoader'
sys.path.insert(0, str(MML / 'tests/game'))
from mml_game import Game
from check_car_explosion import until, vehicle


CASES = (
    ('land-explode', -25),
    ('leave-explode', -15),
    ('land-jack', 20),
    ('leave-jack', 122),
)


def native(g, address, args, result, this=None):
    options = {'this': this} if this else {}
    reply = g.call('native.call', at=hex(address), conv='thiscall' if this else 'cdecl',
                   args=args, ret=result, **options)
    assert reply['stack']['ok'], reply
    return reply.get('ret')


def word(g, at):
    data = g.call('mem.read', addr=hex(at), size=4)['hex']
    return struct.unpack('<I', bytes.fromhex(data))[0]


def symbols(g):
    build = ROOT / 'build/x86'
    module = next(m for m in g.call('modules') if 'mod_ragdoll_' in m['name'].lower())
    built = build / 'out/mml/mods/ragdoll/ragdoll.dll'
    assert sha256(Path(module['path']).read_bytes()).digest() == sha256(built.read_bytes()).digest(), module
    rows = (build / 'ragdoll.map').read_text().splitlines()

    def address(name):
        line = next(row for row in rows if name in row and ' f ' in row)
        return module['base'] + int(line.split()[2], 16) - 0x10000000

    return address


def action(g):
    g.tool('input', steps=[{'action': 'Action', 'ms': 100}])


def check(g, address, name, z):
    lands = name.startswith('land')
    assert g.on_foot()
    g.call('world.teleport', actor='player', to=[-1975, -4, z - 9])
    car = g.tool('spawn', car='Lassiter V16 Roadster', at=[-1975, -4, z])['actor']
    driver = g.spawn_human('civil15', [-1971, -4, z])
    centre = g.pos(car)
    g.call('world.seat', actor='player', car=car, seat=0)
    g.sleep(700)
    assert g.on_foot()
    g.sleep(300)
    door = g.player()['pos']

    def approach():
        g.call('world.teleport', actor='player', to=door,
               dir=[centre[0] - door[0], 0, centre[2] - door[2]])
        g.sleep(350)

    g.call('world.teleport', actor='player', to=[centre[0] - 5, centre[1], centre[2] - 5])
    g.call('world.seat', actor=driver, car=car, seat=0)
    g.sleep(650)
    g.call('human.hit', actor=driver, type=0, damage=1000, dir=[0, 0, 0])
    g.sleep(500)

    def body():
        return next(b for b in g.call('ragdoll.state')['ragdolls'] if int(b['actor'], 16) == int(driver, 16))

    slot = word(g, int(car, 16) + (0x210C if g.build == '1.0' else 0x21CC))
    assert word(g, slot + 0x10) == int(driver, 16) and vehicle(g, driver) == int(car, 16)
    ptr = native(g, address('?Find@?A'), [driver], 'ptr')
    serial = body()['body_serial']

    def kick(v):
        def vector(data):
            return {'buf': 12, 'hex': struct.pack('<3f', *data).hex()}

        args = [vector(v), vector(body()['pelvis']), {'float': 1}, {'float': .3}, {'float': 40}, 0]
        native(g, address('?Kick@Ragdoll@@'), args, 'float', ptr)

    if lands:
        # Seed a freely moving body in a pose that fits above the cushion.
        exit_car = 0x4A1770 if g.build == '1.0' else 0x581220
        native(g, exit_car, [], 'void', driver)
        native(g, address('?TransferSeat@jolt_ragdoll@@'), [ptr, 0], 'int')
        assert word(g, slot + 0x10) == 0 and vehicle(g, driver) == 0
        kick([0, 225, 0])
        until(lambda: word(g, slot + 0x10), lambda owner: owner == int(driver, 16))
        assert vehicle(g, driver) == int(car, 16)
    else:
        kick([0, 2700, 0])
        # An upward-only trajectory can land back on the car. Give the body
        # horizontal momentum while it is above the open cabin.
        for _ in range(8):
            if body()['pelvis'][1] > centre[1] + 1.65:
                break
            g.sleep(50)
        kick([-1800, 1150, 0])
        g.sleep(750)
        if math.dist(body()['pelvis'], g.pos(car)) <= 3:
            kick([-1800, 1700, 0])
        until(lambda: math.dist(body()['pelvis'], g.pos(car)), lambda distance: distance > 3)
        g.sleep(3800)
        assert word(g, slot + 0x10) == 0 and vehicle(g, driver) == 0
    assert body()['body_serial'] == serial

    if name.endswith('explode'):
        g.tool('script', code='car_explosion 0', act={'0': car})
        g.sleep(1100)
        asset = g.call('model.file', actor=driver)['file'].lower()
        assert asset.endswith(r'\ohorelec01.i3d') == lands, asset
        assert body()['body_serial'] == serial
        assert vehicle(g, driver) != int(car, 16) and bool(vehicle(g, driver)) == lands
    else:
        approach()
        scale = g.call('state.time')['scale']
        g.call('state.time', scale=.45)
        try:
            action(g)
            player_clip = g.call('human.anim_state', actor='player')['playing']
            corpse_clip = g.call('human.anim_state', actor=driver)['playing']
            assert (player_clip in (306, 307)) == lands, (player_clip, corpse_clip)
            assert (312 <= corpse_clip <= 315) == lands, (player_clip, corpse_clip)
        finally:
            g.call('state.time', scale=scale)
        g.sleep(6800)
        if lands:
            folder = Path(g.tool('game_status')['game_dir'])
            trace = json.loads((folder / 'mml/data/ragdoll/driver-pull.json').read_bytes())
            samples = trace['samples']
            gaps = [s['gap'] for s in samples if s['gap'] >= 0]
            assert trace['body_serial'] == serial and trace['end'] == 'completed'
            assert len(gaps) > 10 and max(gaps) < .04, gaps
            assert math.dist(samples[0]['pelvis'], samples[-1]['pelvis']) > .65
            assert word(g, slot + 0x10) == 0 and vehicle(g, driver) == 0
        for _ in range(2):
            if vehicle(g, g.player()['actor']) == int(car, 16):
                break
            approach()
            action(g)
            for _ in range(2):
                g.sleep(300)
                if not any(m['name'] in ('itempickup', 'actionmenu') for m in g.call('menu.state')['menus']):
                    break
                g.tool('input', steps=[{'key': '1', 'ms': 140}])
            g.sleep(4000)
        assert word(g, slot + 0x10) == int(g.player()['actor'], 16)
    print(f'{g.build} {name}: pass, body {serial}', flush=True)


if __name__ == '__main__':
    if sys.argv[1:] not in (['1.0'], ['1.2']):
        raise SystemExit('Pass the build of a running city freeride lab (1.0 or 1.2).')
    game = Game(sys.argv[1])
    try:
        address = symbols(game)
        for case, station in CASES:
            check(game, address, case, station)
    finally:
        game.close()
