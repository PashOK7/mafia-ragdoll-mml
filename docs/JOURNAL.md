# Journal

What was done, found and tried, in order (newest last). Agents: read the end first.

## 2026-10-22: the port begins

- **Asked** (the owner): the ragdoll as a MafiaModLoader mod in a repository of its own, made "until it is ideal".
- **Made**: the repository (CMake against MafiaModLoader's SDK and its pinned Jolt 5.6, `tools/deploy.py` into the
  labs); mafia-ragdoll's physics copied as it was (`ragdoll`, `jolt_ragdoll`, `jolt_world`, `knock`, `collision`,
  `tunables`; its `game.h`, `game_build.h`, `mathx.h` are the SDK's now); `scenery.cpp` on the loader's scene pieces
  (a piece's matrix split into a proper rotation and its scale; water faces found by `world_water`: a level triangle
  at the water's height); `vehicles.cpp` on `actor_meshes`; `core.cpp` the ragdoll part of mafia-ragdoll's core on the
  loader's services (humans_list, pedestrians_list, the hit events and hit_filter, bullet_shape, human_hit,
  human_get_up, frame_watch, hook for the dodge fix). The bones are written through `bone_set` (the loader updates
  the model and keeps the pose drawn for the get-up).
- **Loader services added on the way** (MafiaModLoader): `scene_ready` (every visual of a box among the pieces: a
  body put there does not fall through a street not loaded yet), bone_set's model lookup cached a frame (a ragdoll
  writes twenty bones a frame), a pedestrian's bullet told with the point it struck.
- **Seen** (1.2 lab): the mod loads, the world starts around the player (the taxi a body of 20 parts, 23 scenery
  pieces, the river's 54 water faces left out); a person shot dead becomes a ragdoll from the last tick's pose and
  lies on the pavement.
- **Seen** (1.2 lab): a traffic pedestrian shot dead lies where he fell (he had walked on 10 m since the list was read:
  not a jump); the ragdoll waited 0.02 s for the loader to say the ground was there. A car at 7.4 m/s knocked a person
  over (a knock-down ragdoll; the same car's next hits, the game's damage, killed him: as in mafia-ragdoll). Bodies
  knocked two pedestrians over, who got up the game's way; a hot reload adopted the three corpses again from where
  they lay. The player's Thompson on a corpse: two bullets taken by the mod's hit filter (the game's own test found
  the dead person's collision, which follows his bones), the body pushed.
- **Fixed**: people walking past corpses fell over them (487 "blows" in four minutes, two knock-downs): a blow now
  needs the body to move as a thrown one (1.5 m/s where it touches); a body shoved by the walker's own feet is none.
  The scenery's sets came every tick: the host gave its pieces in no fixed order (the signature is a sum now), and it
  scanned every frame because the focus came every tick (fixed in the loader).
- **Seen** (both labs, after the fixes): a person shot dead becomes a ragdoll and lies with his pelvis 17 cm over the
  pavement, in 1.2 and in 1.0; the scenery's sets stay put while nothing changes.

- **Measured** (six corpses in each lab, `profile {focus: "mod_ragdoll"}`): 0.84 ms a frame, then 0.30 in both builds.
  The loader's part went first (bone_set's model lookup, the scene walk: MafiaModLoader's journal). The mod's own:
  - a corpse at rest was written every tick and every frame: a body asleep whose bones nobody turned since our write
    (0.1 degrees) is written twice more and then left be (AtRest), until it wakes or the engine writes them;
  - the corpses at the feet of people standing never slept (the Jolt step 0.99 ms): each living person's capsules were
    moved every step by his idle sway of millimetres, and a kinematic body moved stays awake and wakes what it
    touches. A part within a centimetre and a degree of where it is stays there, its velocity zeroed once, and
    sleeps (the step 0.16 to 0.34 ms, with five living men standing in the pile);
  - the vehicles near were asked of the loader four times a tick (world_near sorts every actor): candidates four
    times a second, 20 m past the radius, filtered by where they are now; a destroyed one goes at once;
  - `ragdoll.state` with seven ragdolls passed its 1024-byte buffer and came back cut, as a string: one Fmt of any
    length (src/fmt.h) for the four copies. Each ragdoll says its `fastest_part` (m/s).
  What is left: the poses of the people near kept every tick (2 % of the frame, what makes a death start moving as
  he moved), the people's list, the Jolt step, the hits' shapes.
- **Learnt**: `human.hit` with `part` 1 to 4 (the limbs) takes 0.3 of the damage from the health (C_human::Hit
  0x5762A0): 300 on a leg leaves 10 of Paulie's 100. A test that means a death gives part 0 or a damage of 1000.
- **Made**: the settings page (Options > Mods > Ragdolls): two tabs, Bodies (ragdolls, cars knocking down, cars
  feeling bodies, bullets pushing bodies, how far people are watched, the bodies drawn) and Forces (a car's push, a
  bullet's push, its whole-body share and spread, the fastest a blow makes a part, how long a living man's last shots
  are kept for his body); every default in config.json. `tools/check_settings.py` lays it out in a running lab
  (panel.layout) and judges it with the loader's own offline check: ALL PASSED. deploy.py had kept a lab's
  config.json, the defaults (what the game changed is in mml/data/ragdoll/): it goes as built now.
- **Seen** (both labs): the page with the game's controls (1.0's in French around it); "Bodies drawn" ticked there
  reached the mod at once (the capsules over the six corpses, the sleeping ones violet).
- **Seen** (both labs): a man killed in a Bolt-Thrower's seat slumps over in it; with the player driving (2.5 s of
  throttle to 11 m/s, a turn, a hard brake) the dead passenger stays in his seat, 15 cm to the side in the turn and a
  few forward at the brake; into a lamp post at 12 m/s (1.2) he stays in, 30 cm forward. Moved by the loader's
  `vehicle.move` (no physics of the game's) the body stays seated too; the one that left the car had been carried
  through a shop's wall by it (the wall pushed it out) and once more when the car, given back to the game's physics,
  rolled into a shop front. A cabin's jump (a car teleported) is logged now ("cabin jumped"): none seen yet.
- **Fixed**: the player driving into a lamp post at 12 m/s was made a knock-down ragdoll (the game's crash hit on the
  driver is type 5, a car's), got up after 4 s under his own car, still seated, and died there. Someone alive in a
  car's seat is never knocked down now (the loader's human_get_up refuses such a man too). Seen in both labs: the same
  crash, Tommy alive in his seat with the game's own damage (169 of 200), the dead passenger still beside him.
- **Fixed**: a car teleported with a dead man in it (a script, `teleport`) left the body behind: the seat's spring
  dragged it 35 m after the car, through the street, and it arrived on the floor at 3 m/s. A cabin's jump now tells
  jolt_ragdoll, which carries every body seated in it by the same move and turn, at rest. Seen in both labs: a car
  teleported 32 m and turned 90 degrees, the passenger's pelvis at the same place in his seat to the centimetre.
- **Made**: the ragdoll camera (src/ragcam.cpp, mafia-ragdoll's ragcam on the loader): while the player is a ragdoll
  and while he gets up (his get-up's own track: state 4 with the game's 111 or 338; 5 s the most) the frame is drawn
  from a camera circling his body (camera_override, taken only then and given back after a 0.6 s blend into the
  game's camera), turned by the mouse (Aim X, Aim Y) and brought nearer by the wheel (Cycle Up, Cycle Down), those
  actions kept from the game meanwhile (action_capture: Tommy neither turns nor cycles his weapons), kept out of
  walls and cars (the game's collision but people). Its settings on a third tab, Camera; `ragdoll.camera`.
- **Seen** (both labs, `ragdoll.start` on the player then `drop`): the camera on his body at once; 0.01 of Aim X for a
  second turned it 80 to 95 degrees round him while his facing stayed; it stayed on him the 2 s of the get-up, blended
  back in 0.6 s and gave the camera back. In 1.0 his body lay behind a car and the camera looked into its wheel: the
  test against walls only took the static world; with the cars too it stops in front of the car (100 frames kept out).
- **Not measured**: the wheel's value in the actions (a notch taken as 120 counts, an action's 1.0 as 400): the
  loader injects the wheel only into the menus, so it wants a real mouse.
- **Seen** (1.0 lab): the player driving a Bolt-Thrower into Paulie at 11.1 m/s: a knock-down ragdoll (the game's
  type 8, 35 of damage), thrown 5 m, then killed by the car's next hits as in 1.2; his body 20 cm over the street. A
  traffic pedestrian shot dead lies on the kerb, his pelvis 14 cm over it. (A car the player was seated in by
  `world.seat` takes no throttle until his getting in has played: the first try reversed under the brake.)
- **Made**: explosions. The EXPLOSION event (after the game's own damage: its dead and knocked over are ragdolls
  already) throws every body within its radius and a half away from its centre, each part by its own distance
  ((1 - d/r)^1.5: the near side more, so a body turns), a third of it up; 12 m/s at the centre for a grenade's force
  of 400, growing with its root, 25 at most ("Blast's push" scales it). The living in it are knocked down ("Blasts
  knock down"). The game hits them with type 2, not 3 as the port believed (C_game::NewExplosion 0x5AAE10: the
  loader's SDK said 3 too, corrected there): with no vehicle behind it a type 2 is a blast now.
- **Fixed**: one hit's momentum is held to 120 N s ("Strongest shot"): a damage of 1000 (a script's, a test's) gave
  1000 N s and threw a body 12 m.
- **Seen** (both labs, the scripts' `explosion 6, 400`): two corpses 2 m from it thrown 4.6 to 7.3 m, a man 3 m away
  killed by it and thrown, a man 5 m away (67 of damage, alive) knocked 2.4 m and up again 2.5 s later.
