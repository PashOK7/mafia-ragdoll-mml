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


def check_shotgun_carry(game, axis=(0,0,1), model='civil17'):
    ground=game.call('world.ground',at=[-1984,0,75])['height']
    player_start=game.pos('player')
    game.tool('teleport',to=[-1988,ground+.10,72],dir=[0,0,1])
    actor=game.spawn_human(model,[-1984,ground+.03,75])
    try:
        game.call('human.anim',actor=actor,play='stand')
        if model=='civil52':
            game.call('human.health',actor=actor,set=30)
        game.sleep(2400)
        bones=game.call('human.skeleton',actor=actor)['bones']
        x,y,z=next(b['world_pos'] for b in bones if b['name'].lower()=='neck')
        mission=int(game.call('game')['mission'],16)
        engine=struct.unpack('<I',bytes.fromhex(game.call('mem.read',addr=hex(mission+0x24),size=4)['hex']))[0]
        shooter=int(game.call('game')['player'],16)
        def bits(v):
            return struct.unpack('<I',struct.pack('<f',v))[0]
        seq=game.latest()
        previous=game.tool('log_tail',lines=100).splitlines()
        origin=[x-.85*axis[0],y-.20,z-.85*axis[2]]
        game.call('native.call',at='0x5A84A0' if game.build=='1.2' else '0x5E3DF0',
                  conv='thiscall',this=engine,ret='uint',
                  args=[shooter,*map(bits,origin),*map(bits,[160*v for v in axis]),bits(200),0,0,8])
        shot=game.call('events.since',seq=seq,types='SHOT')['events']
        assert any(e['data']['shots']==8 for e in shot),shot
        flight=[]
        for _ in range(20):
            game.sleep(20)
            props=game.call('props.list')['props']
            for prop in props:
                if prop['model'].lower()==f'rdhat_{model}_lined.i3d':
                    flight.append(sum((v-p)*d for v,p,d in zip(prop['pos'],(x,y,z),axis)))
        assert len(flight)>=4 and max(flight)>.60,(flight,game.call('events.since',seq=seq,types='HUMAN_HIT,BULLET_HIT')['events'])
        logs=[line for line in game.tool('log_tail',lines=100).splitlines() if line not in previous]
        lift=next((line.split(' lift along head ')[1].split() for line in logs
                   if 'headwear shot carry' in line),None)
        assert lift and float(lift[0])>0 and float(lift[3])<.34*float(lift[0]),logs[-15:]
        assert any('headwear shot separation' in line for line in logs),logs[-15:]
        print(game.build,model,'eight-pellet shot: hat forward travel',round(max(flight),3),'m')
    finally:
        game.cleanup()
        game.tool('teleport',to=player_start)


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
            assert y>=support+.001,(model,[x,y,z],support)
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
        # First instruction of the native static-impact branch, after
        # TickShoot has checked both the dynamic result and shot.hasImpact.
        tap_at='5AA025' if game.build=='1.2' else '5E5975'
        tap=game.tool('tap',cmd=tap_at+' n=0').split()[1].rstrip(':')
        try:
            check_hat_shots(prop,tap)
        finally:
            game.tool('tap',cmd='off '+tap)

    def check_hat_shots(prop,tap):
        game.sleep(1600)
        pos=hats()[prop]['pos']
        mission=int(game.call('game')['mission'],16)
        engine=struct.unpack('<I',bytes.fromhex(game.call('mem.read',addr=hex(mission+0x24),size=4)['hex']))[0]
        shooter=int(game.call('game')['player'],16)
        def bits(value):
            return struct.unpack('<I',struct.pack('<f',value))[0]
        def fire_down(origin,damage=28):
            game.call('native.call',at='0x5A84A0' if game.build=='1.2' else '0x5E3DF0',
                      conv='thiscall',this=engine,ret='uint',
                      args=[shooter,*map(bits,origin),*map(bits,[0,-160,0]),bits(damage),0,0,1])
        def ground_impacts():
            return int(game.tool('tap',cmd='show '+tap).split(': ',1)[1].split()[0])
        control=[pos[0]+1.5,pos[1]+1,pos[2]]
        fire_down(control)
        game.sleep(200)
        assert ground_impacts()==1
        def hit_lines():
            return [v for v in game.tool('log_tail',lines=200).splitlines() if 'headwear bullet impulse' in v]
        displacements=[]
        lifts=[]
        rebounds=[]
        attempts=0
        for damage in (28,50,28,50,28,50):
            pos=hats()[prop]['pos']
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
            for x,y,z in targets[:12]:
                attempts+=1
                before=hit_lines()
                ground_before=ground_impacts()
                origin=[x,y+.20,z]
                fire_down(origin,damage)
                trail=[]
                for _ in range(5):
                    game.sleep(20)
                    trail.append(hats()[prop]['pos'])
                hits=[line for line in hit_lines() if line not in before]
                if not hits:
                    continue
                assert len(hits)==1,hits
                assert ground_impacts()==ground_before, 'hat hit also produced a native floor impact'
                travelled=math.dist(pos,hats()[prop]['pos'])
                assert travelled<1.25,travelled
                displacements.append(round(travelled,3))
                lifts.append(round(max(sample[1]-pos[1] for sample in trail),3))
                rebound=float(hits[0].split(' rebound ')[1].split()[0])
                expected=2*min(1.5,damage*.045)
                if ' support 1 ' in hits[0]:
                    assert .5*expected<=rebound<=expected+.001,(hits[0],expected)
                else:
                    assert rebound==0,hits[0]
                rebounds.append(rebound)
                game.sleep(650)
                break
            else:
                raise AssertionError(('no hit on visible hat triangles',pos))
        assert max(rebounds)>2 and .03<max(lifts)<.52,(rebounds,lifts)
        print(game.build,'six resolved shots in',attempts,'attempts, rebound',rebounds,
              'lift in m',lifts,'displacement',displacements)

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
        _, b = spawn(model, 12 if model=='civil19' else 6)
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
        elif model == 'freegang01':
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
    check_shotgun_carry(game)
    check_shotgun_carry(game,axis=(.85,0,math.sqrt(1-.85**2)),model='civil52')


if __name__ == '__main__':
    if sys.argv[1:] not in (['1.0'], ['1.2']):
        raise SystemExit('Pass one configured lab build (1.0 or 1.2).')
    lab = Game(sys.argv[1])
    try:
        check(lab)
    finally:
        lab.close()
