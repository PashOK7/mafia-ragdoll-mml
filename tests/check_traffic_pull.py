"""Check a dead traffic driver's physics and drawn pose across the normal Action interaction."""
import argparse
import json
import math
from pathlib import Path
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
MML = ROOT.parent / 'MafiaModLoader'
sys.path.insert(0, str(MML / 'tests/game'))
from mml_game import Game


def run(build, capture):
    folders = json.loads((MML / 'mml.config.json').read_bytes())['labs']
    trace = Path(folders[build]) / 'mml/data/ragdoll/driver-pull.json'
    g = Game(build, 4 if build == '1.2' else 'Run')
    started = False
    try:
        assert not g.tool('game_status')['running'], 'lab in use'
        g.tool('game_launch', go='freeride', kind='city-small', profile=g.profile, police=0,
               traffic=1, pedestrians=0, fullscreen=False, wait_s=90)
        started = True
        if not any(m['id'] == 'ragdoll' and m['loaded'] for m in g.call('mods.list')):
            g.call('mod.load', id='ragdoll')
        assert g.on_foot()

        def read(address, fmt):
            data = g.call('mem.read', addr=hex(address), size=struct.calcsize(fmt))['hex']
            return struct.unpack(fmt, bytes.fromhex(data))

        def matrix(frame):
            return read(frame + 0x10, '<16f')

        limit = time.monotonic() + 25
        candidates = []
        while not candidates and time.monotonic() < limit:
            p = g.player()['pos']
            start, end = read(0x63D848 if build == '1.2' else 0x6561A0, '<II')
            if 0 < end - start <= 4096:
                for record in read(start, '<' + 'I' * ((end-start)//4)):
                    if not record or not read(record, 'B')[0]: continue
                    car, _, _, _, model = read(record+12, '<5I')
                    if not car or not model: continue
                    frame = read(car+0x68, '<I')[0]
                    if not frame: continue
                    m = matrix(frame)
                    distance = math.dist(p, m[12:15])
                    if distance < 65 and not read(car + (0x2131 if build == '1.2' else 0x2071), 'B')[0]:
                        candidates.append((distance, car, model))
            if not candidates: time.sleep(.3)
        assert candidates, 'no nearby live traffic driver'
        _, car, model = min(candidates)
        g.call('native.call', at=hex(0x4BADD0 if build == '1.2' else 0x506C00), conv='fastcall',
               ret='void', args=[car, model])

        def occupant():
            return next(o for o in g.call('ragdoll.state')['occupants'] if int(o['model'], 16) == model)

        g.sleep(1800)
        # World-space continuity is meaningful once the arriving car and its occupant are at rest.
        for _ in range(40):
            previous = occupant()
            g.sleep(400)
            now = occupant()
            velocity = g.call('vehicle.body', car=hex(car))['velocity']
            if math.hypot(*velocity) < .05 and max(math.dist(previous[k], now[k]) for k in ('head', 'pelvis')) < .004:
                break
        else:
            raise AssertionError('traffic fixture did not settle')
        # Ask the game for the driver's door approach point and direction from its animation track.
        def car_call(v12, v10, args):
            return g.call('native.call', at=hex(v12 if build == '1.2' else v10),
                          conv='thiscall', ret='uint', args=args, **{'this': car})

        flags = car_call(0x41DC30, 0x464C50, [0] + [{'buf': 4} for _ in range(4)])
        assert flags['ret'] & 255
        bits = [bytes.fromhex(v['hex'])[0] for v in flags['buffers']]
        clip = g.call('native.call', at=hex(0x57DFC0 if build == '1.2' else 0x49E520),
                      conv='fastcall', ret='uint', args=bits + [0, 0])['ret']
        placement = car_call(0x41C550, 0x463500, [0, clip, {'buf': 12}, {'buf': 12}])
        assert placement['ret'] & 255
        door, facing = [v['floats'] for v in placement['buffers']]
        door[1] = g.call('world.ground', at=[door[0], now['pelvis'][1] + .5, door[2]])['height'] + .03
        g.call('world.teleport', actor='player', to=door, dir=facing)
        g.sleep(800)
        find_child = read(read(model, '<I')[0] + 0x38, '<I')[0]
        neck = g.call('native.call', at=hex(find_child), conv='stdcall', ret='ptr',
                      args=[model, {'str': 'neck'}, 0xffff])['ret']
        before = occupant()
        neck_world = matrix(int(neck, 16))
        up = neck_world[4:7]
        drawn_head = [p + .20 * axis / math.hypot(*up) for p, axis in zip(neck_world[12:15], up)]
        old = trace.stat().st_mtime_ns if trace.exists() else None
        result = g.tool('input', steps=[{'action': 'Action', 'ms': 110}])
        for _ in range(2):
            g.sleep(250)
            menus = g.call('menu.state')['menus']
            if not any(m['name'] in ('itempickup', 'actionmenu') for m in menus):
                break
            g.tool('input', steps=[{'key': '1', 'ms': 140}])
        limit = time.monotonic() + 10
        while time.monotonic() < limit and (not trace.exists() or trace.stat().st_mtime_ns == old):
            time.sleep(.15)
        assert trace.exists() and trace.stat().st_mtime_ns != old, (result, before, door, g.player())
        data = trace.read_bytes()
        if capture:
            with capture.open('xb') as out: out.write(data)
        pull = json.loads(data)
        first = pull['samples'][0]
        assert pull['body_serial'] == before['body_serial'], 'promotion recreated the body'
        shift = max(math.dist(before[k], first[k]) for k in ('head','pelvis'))
        assert shift < .04, f'head or pelvis jumped {shift:.3f} m'
        drawn_shift = math.dist(drawn_head, first['drawn_head'])
        assert drawn_shift < .04, f'drawn head jumped {drawn_shift:.3f} m'
        assert pull['end'] == 'completed' and any(s['coat'] for s in pull['samples'])
        print(f"PASS {build} {pull['car_model']}, body {pull['body_serial']}, "
              f"physical shift {shift:.3f} m, drawn head shift {drawn_shift:.3f} m")
    finally:
        if started and g.tool('game_status').get('running'): g.tool('game_close')
        g.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('build', choices=('1.0','1.2'))
    parser.add_argument('--capture', type=Path)
    args = parser.parse_args()
    run(args.build, args.capture)
