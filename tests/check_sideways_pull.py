"""Exercise the driver's cross-seat pose in the owner's car model in a windowed lab."""
import argparse
import json
import time
from pathlib import Path
import sys
from check_chest_grip import check
from analyze_driver_pull import analyze

MML = Path(__file__).resolve().parents[2] / 'MafiaModLoader'
sys.path.insert(0, str(MML / 'tests/game'))
from mml_game import Game


def run(build, capture):
    lab = Path(json.loads((MML / 'mml.config.json').read_bytes())['labs'][build])
    log = lab / 'mml/data/ragdoll/driver-pull.json'
    g = Game(build, 4 if build == '1.2' else 'Run')
    started = False
    try:
        assert not g.tool('game_status')['running'], 'lab in use'
        g.tool('game_launch', go='freeride', kind='city-small', profile=g.profile, police=0,
               traffic=0, pedestrians=0, fullscreen=False, resolution=[800, 600], wait_s=90)
        started = True
        if not any(m['id'] == 'ragdoll' and m['loaded'] for m in g.call('mods.list')):
            g.call('mod.load', id='ragdoll')
        assert g.on_foot()
        g.sleep(1300)
        car = g.call('world.spawn', model='pontfor11.i3d', kind='car', at='player', ahead=12)['actor']
        g.call('world.seat', actor='player', car=car, seat=0)
        g.sleep(1400)
        assert g.on_foot()
        g.sleep(1600)
        door, centre = g.player()['pos'], g.pos(car)
        inward = g.call('vehicle.body', car=car)['right']
        if sum(a*(b-c) for a,b,c in zip(inward, centre, door)) < 0:
            inward = [-v for v in inward]
        actor = g.spawn_human('civil15', [door[0]+3, door[1]+.1, door[2]])
        g.call('world.seat', actor=actor, car=car, seat=0)
        g.sleep(1000)
        assert g.call('human.anim_state', actor=actor)['state'] == 9
        g.call('human.hit', actor=actor, type=0, damage=1000, dir=inward)

        def pose():
            return next(r for r in g.call('ragdoll.state')['ragdolls'] if r['actor'] == actor)

        def across(row):
            return sum(axis*(c-p) for axis,c,p in zip(inward, row['chest'], row['pelvis']))

        for _ in range(24):
            g.sleep(160)
            body = pose()
            if across(body) >= .36:
                break
            g.call('human.hit', actor=actor, type=0, damage=100, dir=inward, pos=body['chest'])
        print(build, 'cross-seat separation before Action', round(across(pose()), 3), flush=True)
        g.call('world.teleport', actor='player', to=door, dir=inward)
        g.sleep(250)
        old = log.stat().st_mtime_ns if log.exists() else None
        g.tool('input', steps=[{'action':'Action', 'ms':110}])
        for _ in range(2):
            g.sleep(250)
            if not any(m['name'] in ('itempickup', 'actionmenu') for m in g.call('menu.state')['menus']):
                break
            g.tool('input', steps=[{'key':'1','ms':140}])
        until = time.monotonic()+10
        while time.monotonic()<until and (not log.exists() or log.stat().st_mtime_ns==old):
            time.sleep(.15)
        assert log.exists() and log.stat().st_mtime_ns != old, 'Action did not start a pull'
        data = log.read_bytes()
        with capture.open('xb') as out:
            out.write(data)
        trace = json.loads(data)
        rows = trace['samples']
        i = next((i for i, r in enumerate(rows) if r['coat']), None)
        assert i is not None and i > 0, 'driver was never grasped after the reach'
        separation = across(rows[i-1])
        print(build, 'cross-seat separation at grip', round(separation,3), rows[i].get('grip_part'))
        assert separation >= .30, 'driver did not retain the cross-seat pose'
        assert rows[i]['grip_part'] in ('pelvis','abdomen'), 'selected the far end of the trunk'
        check(capture)
        assert min(analyze(trace)['final_tyre_clearance_m']) > 0, 'driver stayed in the doorway'
    finally:
        if started and g.tool('game_status').get('running'):
            g.tool('game_close')
        g.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('build', choices=('1.0','1.2'))
    parser.add_argument('capture', type=Path)
    args = parser.parse_args()
    run(args.build, args.capture)
