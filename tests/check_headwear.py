"""Check per-wearer visibility and falling props in a running freeride lab."""
import math
import struct
from pathlib import Path
import sys

from check_transport_occupants import Game
from check_scalp_meshes import check as check_scalps, skin_lods


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
        stopped = False
        for _ in range(45):
            positions = hats()
            for prop in positions.keys()-before:
                flight.append(positions[prop]-start)
            if flight and not stopped:
                # Measure free flight after the first strike. A truck still
                # advancing can catch and keep pushing the same hat.
                game.call('vehicle.move', car=truck, ms=0)
                game.call('vehicle.body', car=truck, velocity=[0,0,0])
                stopped = True
            if len(flight) >= 12:
                break
            game.sleep(24)
        # The stopped truck or the wearer can still catch the hat at low
        # clearance. Initial carry is required even when that arrests flight.
        assert flight and flight[0] > 1 and max(flight)-flight[0] < 12, flight
        print(game.build, 'truck impact: hat first seen', round(flight[0],2),
              'm forward, then travelled another', round(max(flight)-flight[0],2), 'm')
    finally:
        game.cleanup()


def check(game):
    check_scalps(game)
    start = game.pos('player')

    def hats():
        return {p['prop']: p for p in game.call('props.list')['props'] if p['model'].startswith('rdhat_')}

    def mesh_world(prop):
        pos=hats()[prop]['pos']
        frame=game.call('frame',addr=prop)
        model=hats()[prop]['model'].rsplit('.',1)[0]
        files=game.call('files.list',mod='ragdoll')['files']
        path=next(Path(f['path']) for f in files if f['name']=='models\\'+model+'.4ds')
        vertices,faces=next(skin_lods(path.read_bytes(),lined_prop=True))
        points=[[pos[j]+sum(v[i]*frame['axes'][i][j] for i in range(3)) for j in range(3)] for v in vertices]
        return model,points,faces

    def above_ground(prop, start):
        pos=hats()[prop]['pos']
        height=game.call('world.ground',at=[pos[0],max(start[1],pos[1])+.3,pos[2]])['height']
        assert -.045<pos[1]-height<.35, (pos,height)
        model,points,_=mesh_world(prop)
        lowest=sorted(points,key=lambda v:v[1])
        checked=set()
        for x,y,z in lowest:
            column=round(x,2),round(z,2)
            if column in checked:
                continue
            checked.add(column)
            support=game.call('world.ground',at=[x,y+.18,z])['height']
            assert y>=support-.025,(model,[x,y,z],support)
            if len(checked)==5:
                break

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

    def shoot_hat(prop):
        game.sleep(1600)
        pos=hats()[prop]['pos']
        mission=int(game.call('game')['mission'],16)
        engine=struct.unpack('<I',bytes.fromhex(game.call('mem.read',addr=hex(mission+0x24),size=4)['hex']))[0]
        shooter=int(game.call('game')['player'],16)
        def bits(value):
            return struct.unpack('<I',struct.pack('<f',value))[0]
        _,points,faces=mesh_world(prop)
        targets=[]
        for i,j,k in faces:
            a,b,c=points[i],points[j],points[k]
            area=(b[0]-a[0])*(c[2]-a[2])-(c[0]-a[0])*(b[2]-a[2])
            if abs(area)<.00004:
                continue
            centre=[(a[d]+b[d]+c[d])/3 for d in range(3)]
            targets.append(centre)
        targets.sort(key=lambda v:math.hypot(v[0]-pos[0],v[2]-pos[2]),reverse=True)
        def latest_hit():
            return next((v for v in reversed(game.tool('log_tail',lines=150).splitlines())
                         if 'headwear bullet impulse' in v),None)
        before=latest_hit()
        for x,y,z in targets[:12]:
            origin=[x,y+.20,z]
            game.call('native.call',at='0x5A84A0' if game.build=='1.2' else '0x5E3DF0',
                      conv='thiscall',this=engine,ret='uint',
                      args=[shooter,*map(bits,origin),*map(bits,[0,-160,0]),bits(28),0,0,1])
            game.sleep(200)
            travelled=math.dist(pos,hats()[prop]['pos'])
            assert travelled<1.4, travelled
            if latest_hit()!=before:
                break
        else:
            raise AssertionError(('no hit on visible hat triangles',pos))
        print(game.build,'shot confirmed on outer hat mesh, displacement',round(travelled,3),'m')

    def walk_into_hat(prop):
        car=game.player()['vehicle']['actor']
        assert game.on_foot()
        try:
            game.sleep(800)
            for dx,dz in ((0,1),(1,0),(0,-1),(-1,0)):
                before=hats()[prop]['pos']
                # Align a shoe rather than the gap between Tommy's feet.
                start=[before[0]-dx*.95+dz*.10, before[1], before[2]-dz*.95-dx*.10]
                start[1]=game.call('world.ground', at=start)['height']+.02
                game.tool('teleport', to=start, dir=[dx,0,dz])
                game.sleep(150)
                game.tool('input', steps=[{'action':'Forwards','ms':1050}])
                moved=math.dist(before,hats()[prop]['pos'])
                if moved>.02:
                    print(game.build,'walking nudged hat',round(moved,3),'m')
                    return
            raise AssertionError('four footsteps failed to nudge the hat')
        finally:
            game.call('world.seat',actor='player',car=car,seat=0)

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
        assert after[prop]['parent'] == 'world'
        game.sleep(850)
        distance = math.dist(after[prop]['pos'], hats()[prop]['pos'])
        assert math.isfinite(distance) and distance < 20, (model, distance)
        print(game.build, model, 'independent hat, travelled', round(distance, 3), 'm')
        if model == 'civil19':
            shoot_hat(prop)
            walk_into_hat(prop)
        else:
            game.sleep(1600)
        above_ground(prop,after[prop]['pos'])
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
