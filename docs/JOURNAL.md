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
- **Measured** (1.2 lab, the loader's `input.mouse {wheel: 2}` while the camera was on the body): two notches read
  0.6 of Cycle Up (0.3 a notch: 120 counts of an action's 400) and brought the camera from 2.98 to 2.26 m (0.87
  squared). (Written first as "not measured: the wheel reaches only the menus": it reaches the game everywhere.)
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
- **Fixed**: bodies never floated ("no water known"): the port had not given jolt_world its water. It is the loader's
  world_water now, asked from just over each body (and from 12 m over for one sunk deeper) and taken only where no
  static face is between (the city's river is one plane under the whole city: asked from high up, every street had
  water under it), kept in cells of 2 m by 1 m for the mission. WaterFn takes the body's height.
- **Seen** (both labs, the city's embankment by Little Italy's bridge): a man by the parapet killed by a blast behind
  him flew over it into the river and floats there, his back and the back of his head at the surface (-9.99; the
  pelvis 0.6 to 0.8 m under, the parts' buoyancies as mafia-ragdoll had them: a body is hardly lighter than water).
- **Learnt**: the game drowns a person in deep water its own way: his health 0 at once, his alive flag kept while he
  panics and goes under ("x panika leh dolu", "!plachteni"), and he sinks to -137 m before it drops: no ragdoll then
  (he is far out of reach). The AGENTS.md place "the embankment" is the pavement beside it: the open water is 5 m on
  along +x (-1591.1, -33.8).
- **Measured** (1.2, 22 corpses near the player, 13 people tracked): 0.82 ms a frame; the bodies' capsules given to
  the loader's bullets every tick were 3 % of the frame. The loader keeps them until they are given again: a body
  asleep gives them once now (0.58 ms a frame). A sleeping corpse shot afterwards: two bullets on its capsules, taken.
  A restore point loaded with corpses lying: the game's own load brought them back and the mod adopted them again.
- **Fixed**: a leak. jolt_world gathers the splashes of the bodies and cars in water for mafia-ragdoll's water (not a
  part of this port): nobody took them, and a body floating in the river added fifteen a tick for as long as it lay
  there. A step's splashes are dropped at the next step now. And a body asleep on dry land no longer asks for water.

### 2026-11-08 — Reference parity: get-up and car handover (in progress)

- Compared the standalone reference read-only. The articulated solver is retained; its staged get-up was lost
  in the SDK service. MafiaModLoader now holds/fits/blends the real animation, tracks pedestrian collision cells,
  suppresses track lift and cancels handovers on owner unload. The mod cancels on re-knockdown, and supplies the
  current physical pose through optional human_get_up_pose before destroying the body (SDK bone-order mapping).
- Pedestrian car hits report a root position, not contact: fallback impulse acts at the centre as in the reference.
  Normal Jolt car contacts remain unchanged. No solver/tuning changes. Both-build runtime checks follow.
- Owner also reports missed taxi-waver contacts. Investigate the native pre-hit/boarding transition separately;
  no broad radius or flag patch is justified yet. Comparison record: ../MafiaModLoader/docs/RAGDOLL_PORT_REVIEW.md.

### 2026-11-08 — Parity fixes validated in both builds

- Final shared-service and port path passed 18 focused assertions in each build, serially: displaced-pelvis
  get-up sides, current Jolt pose, pause, re-knockdown, owner unload, pedestrian grid/native walking, native
  contact service and weak/strong car thresholds. Actual-pose handover also passed in the owner's native-D3D8 copy.
- Actual car convex hulls now meet living anatomical capsules in a sensor layer. Contacts are deferred until
  after the solver and submitted through the SDK's native vehicle-hit service when native contact was missed.
  The car's own approach is required; minor contacts/occupants/duplicate knockdowns are excluded.
- External strikes no longer disable the striking car's entire hull through the open-door occupant exemption.
  Genuine seated/doorway behavior remains. Centre-applied fallback impulses match the reference; articulated
  solver, constraints, masses and tuning remain. No additional user-facing settings.
- Natural taxi wavers with native proxies deliberately removed passed in both builds: native wave termination,
  hit before the car centre reaches the body and solid striking hull. Native boarding scripts are unchanged.
  These tests cover induced misses, not every cause of native missed collision. See the shared review for evidence.
- DLL SHA256 9B6447E668972692832AC829245E051EE8DF1CA406C05A5123CCF3248D8DEAE4 and matching PDB installed
  in both labs and W:/Mafia10Fixes with host 5A8902E8...; owner copy restarted, only ragdoll loaded, original
  renderer retained. Lab water manifests restored and labs closed. No new crash during focused checks.

### 2026-11-08 — Get-up trajectory follow-up and hard-stop evidence

- Owner saw a 3.778 m get-up teleport. MafiaModLoader reproduced 3.966 m despite an initially correct fit: the
  native root-motion machine retained/restored its old fall trajectory. Host now clears obsolete root tracks
  and rebases the active trajectory with native RecompileMatrix after fitting. Physics solver is unchanged.
  Expanded shared regression (20 assertions including continuous native fall/get-up) passed in both builds.
- Rare low-speed dodge hard stop remains open. A curb stop was excluded by fresh modless reproduction; a
  controlled free-street dodge/strike did not stop the car like a stone. No speculative collision policy change.
  The mod now logs large speed drops near a pedestrian with native speeds/dt, Jolt velocity delta, animation,
  physical-body status and element/model separation. Bounded to once per car per 60 ticks, no UI setting.
- Final follow-up DLL FB195440FBA066B0EE2B6014F9D64057710209C57DCD284715EDD77E2BCDCD5C and PDB installed
  in both labs and Mafia10Fixes with host F78DCF7E... . Owner native-D3D8 continuity check passed, restarted
  PID 15164 left running; labs closed with water manifests restored. Rare hard stop remains open.

### 2026-11-08 — Lying spine axes from exact live memory

- Exact corpse 1AB5AAF4 in Mafia10Fixes had an approximately -179 degree initial middle-spine twist. Its
  anatomical fronts were consistent, but signing each against the standing root heading flipped one segment.
  Initial-pose limit expansion admitted the error. Rendering matched the physical state; this was neither a
  screenshot-only artifact nor the earlier get-up/impulse issue. Raw evidence in MafiaModLoader/build/ragdoll-joints-20261108.
- src/anatomy.h supplies the same ordered-landmark frames to Setup and Jolt Add, independent of root heading.
  Solver, muscles, masses and limits unchanged. New ragdoll.inspect {actor|pedestrian} exposes initial landmarks,
  bone-binding errors and actual Jolt angles/ranges without a new setting.
- Opt-in regression: cmake --build --preset x86-release --target ragdoll_trunk_axes, then
  build/x86/RelWithDebInfo/ragdoll_trunk_axes.exe. Passed curved lying fronts under opposite standing headings.
  Native lying A/B checks plus get-up/cancel/continuity: 11 assertions passed per build, serially. Native start
  pose is explicitly verified; render-only pose overrides do not reliably enter mission-tick pose history.
- Mod B3CC97E11B1170C06E4DC0ADF27DF4D82D36BF24D872551AF5D960D96236A596 and matching PDB installed in both
  labs and Mafia10Fixes; host unchanged. Labs closed, water manifests restored. Shared review documents one
  intermittent 0.721 m get-up sample not reproduced by a full trace, and the still-open rare car hard stop.
- Native Mafia10Fixes also passed six lying-pose assertions plus live DLL hash/only-ragdoll checks (eight total),
  PID 3308, RTSS attached, original D3D8. First restart had an RTSSHooks.dll startup crash before any body;
  triaged and retained, next unchanged launch passed. Shared Known gaps tracks the unconfirmed interaction.

### 2026-11-08 — Full skeleton calibration and walking-contact attribution

- Owner's next prone corpse (18BE9C94) still inverted its head and arms. Exact memory preserved in the shared
  build/rtss-focus-20261108 evidence. Initial skin was plausible; neck range -172..70 admitted inversion.
  Head/nose and hand/foot helpers used standing facing; all constraints were fitted/widened to a posed death.
- Read actual civil15 rest skeleton through SDK model_open/model_nodes/model_close. Production now caches
  only small rest poses per model, closes parsed meshes immediately, uses complete bone orientations for
  physical frames/helpers and calibrates joint axes in rest space. Axes move with each posed body; limits
  remain anatomical. Hinge motors use calibrated local normals too. Initial 24 skin/joint assertions and
  11 spine/get-up assertions passed initially; final both-build checks below supersede these intermediate counts.
- Person contact attribution now snapshots pre-push velocities and mass-weighted assembly momentum using
  Jolt's ragdoll owner. A foot-driven spinning limb is not an incoming thrown body. Push resolution still
  sees each preceding push's velocity. COM gating alone failed real walking checks and was rejected; exact impulse
  work supplied by each kinematic source is conservatively excluded from its later incoming-body energy.
- Owner then found weightless-body behaviour: the impact term bypassed the force budget and supplied 20 kg per
  touching part every frame. Shared SegmentMass now supplies the same 78 kg anatomical distribution to dynamic
  bodies and living proxies; full posed rotations also reach those proxies. Walker-powered impact plus overlap
  correction shares one maxForce*dt allowance per person. Incoming dynamic reaction is separate. No mass inflation,
  tuning reduction or new setting. Native animation remains kinematic, coupled with finite effort.
- Final DLL ECB39D43FEDC5F01DA72563D9FF70E372A125F8138F5C10D7AC85E466E2C60F7 / PDB 75FDA2FB...:
  43 assertions passed per serial lab (native Paulie/civil15 deaths, fixed joint limits, actual head/elbow skin
  axes, get-up/cancel, four real corpse crossings, shared effort). Physical 9 m/s incoming bodies still knock
  the same walker over in both builds. Walking total force peaks at 700 N regardless of contact multiplicity.
  Full evidence is in MafiaModLoader/build/ragdoll-anatomy-20261108 and ragdoll-contact-budget-20261108.
- Exact binaries installed in both labs and native-D3D8 Mafia10Fixes with backups. Owner copy passed 28 anatomy
  assertions and nine final contact/install checks. A native velocity-sampling fixture failure was corrected to
  measure lasting physical displacement instead of a brief >0.25 m/s peak; no production retuning. Both labs closed,
  original water manifests restored, owner window visible. Shared review retains rare dodge/car-stop and get-up
  outlier limitations. RTSS crash root fix belongs to the host; read its shared journal before investigating anew.

### 2026-11-09 — Palm geometry and local bullet impulse

- The host resolves a registered corpse's current capsule and forwards its exact contact. `HitFilter` now passes
  that part to `Kick` instead of redistributing its local share by distance. The whole-body share and per-tick
  caps still govern the response. `ragdoll.state.last_bullet` exposes the capsule, body, contact and local velocity
  increment for diagnosis. Legacy hits without a capsule still use the distance-based path.
- `HandShape` describes elbow, wrist and palm tip in forearm space, with palm direction calibrated from the
  model rest pose and carried by the actual bound wrist. Bullet capsules and Jolt's compound forearm both
  use that geometry. The wrist retains its pose as part of the forearm rigid body.
- A native 1.0 exit dump showed Jolt body destruction after the physics world was gone. The world is explicitly
  created and destroyed by Start/Stop, with process teardown left to the OS. A 1.0 ExitProcess check with a live
  ragdoll returned zero. See the shared journal for the unrelated fail-fast from direct exit in the 1.2 lab.
- DLL AE8804E3... with host 6E969ED2... installed in both labs and W:/Mafia10Fixes. Both palm projectiles and
  native hit filters passed on both builds and the owner's copy. Each lab passed 43 prior anatomy/get-up/walking
  assertions with the palm collider. Evidence and rollback files are in MafiaModLoader/build/ragdoll-bullets-20261109.

### 2026-11-09 — Use the actual solver input for car reactions

- The owner's vehicle trace showed 3 m/s per tick from this mod after an Omerta load. The live paused car
  had a compound centre of mass 1.6 km away, no current Jolt contacts, and a 15348.97 m/s request clamped
  by Jolt to 500 m/s. Comparing the solver output with the unclamped request produced a fictional impact.
- `ApplyKinematics` now reads both velocities back immediately after the clamping setter. The host separately
  fixes the rigid model vertex transforms that misplaced two parts of the compound. See the shared journal.
- DLL B4A6C097... is installed in both labs. The 1.0 slot 230 drive gave no fictitious reaction and a 1.2
  drive into a physical corpse retained the real contact reaction. Evidence: MafiaModLoader/build/vehicle-trace-20261109.
- The airport repeat was still running the old binary. Updated Mafia10Fixes to B4A6C097... and the paired
  host D8D4B506..., then restored the live mission with the car back on the ground. Verified the exact
  loaded DLL and correct positions of both previously misplaced car parts in PID 25280.

### 2026-11-09 — Keep native driving and living hits isolated

- Publish bullet capsules for dead actors only. A live puppet keeps native hit detection and gets its 17
  capsules after it dies. The paired loader fixes the x86 mid-hook flags responsible for the airport misses.
- Export car reactions only for the current step's solid kDynamic contact. Clear the previous reaction before
  a new step can return early. Both labs had zero reaction during a clear drive and a nonzero reaction upon
  contacting a corpse. Keep the original solver velocity readback from the previous change.
- Remove the pedestrian movement sweep's nearest-car inference and leave world obstacle hits to the engine.
  Keep the dedicated pedestrian car sweep/hit correction, gated by enabled and car_knockdown, and retain the
  animated proxy fallback. The latest 1.2 taxi path missed the selected waver and was inconclusive.
- DLL 11F1CBCE... installed in both labs and the owner copy with host 59EA85E8.... Owner PID 18072 has the
  matching DLL active and the airport mission restored and paused. Evidence in MafiaModLoader/build/vehicle-trace-20261109.

### 2026-11-09 — Inspect a sideways slide after a crowd strike

- In the owner's Omerta city mission, 0x18E6E20C and 0x18E6EC0C both started on the same native tick with
  almost 12 m/s in +X from the last two poses. The car's hit direction was predominantly -Z. The last body
  moved 2.63 m towards the native wall face. The current clamp lets a rapid pose change supply a full-speed
  sideways seed. The capture establishes the seed and final contact, but lacks the original pose pair needed
  to identify which native animation transition produced it. See the shared journal and the incident capture.

### 2026-11-09 — Seed pedestrian motion from the native position

- Track continuous native element movement alongside the rendered pose. A missing tick, model replacement,
  clip switch or over-limit position change invalidates the pair. Bone differences give relative limb velocity
  and turning, with zero mass-weighted linear mean. The native position change supplies the body's translation.
- The opt-in `ragdoll_pedestrian_motion` target links the production seeding code and checks a 20 cm pose jump,
  3 m/s native running with limb motion and the no-history path. Both builds passed 11 live checks for
  pedestrian handover, car-hit thresholds and bullets. The original wall route was not replayed.
- DLL 1D8E110B... installed in both labs and hot-reloaded in Mafia10Fixes PID 18072, generation 2. Verified the
  running hash without restarting or loading a save.

### 2026-11-09 — Preserve NPC blast gameplay and let an awake body recover

- The exact garage two-car sequence demonstrated a mission regression: the first blast's physical knockdown
  moved Enemy1 about 2 m and the second blast no longer killed him. The scripted vehicle transfer depends
  on his death. Surviving NPCs keep their native reaction to the blast; the player's optional knockdown and
  existing physical bodies still receive the blast. The original mission script transferred the occupied car
  with the corrected port in the 1.0 lab.
- Gameplay recovery measures quiet linear and angular motion on support, independent of solver sleeping.
  The original 2.5 s minimum and the ten-second fail-safe remain. On the garage replay with sleep disabled
  for the fixture, Tommy's native get-up began at 2.512 s. Both builds passed the carried-player blast checks.
- Port 544F3E04... installed in both labs and in Mafia10Fixes, hot-reloaded generation 3. Evidence and rollback
  binaries are in MafiaModLoader/build/vehicle-trace-20261109.

### 2026-11-09 — Native garage floor and same-mission reload

- Merge the game's finite two-sided 0x83 collision slabs into the local Jolt scenery. A native ray and a sphere
  sweep confirmed the missing slab's physical plane. The same explosion made the old body fall through the
  floor before reaching its edge. The new body stayed on the slab and got up in both builds.
- Stop pose, camera and interaction overrides while the host reports a native load. Reset physical bodies,
  bullet shapes, pose history and the camera on `GAME_LOADED` even if the game kept the scene and Tommy's model.
  Existing frame watches still follow those models until release. Native Retry from a dead body below the map
  restores the real checkpoint in both builds.
- Port BAED20C1... with host 7B7E4040... installed in both labs and Mafia10Fixes. The exact checkpoint and the
  user's pre-install position were verified. See the loader journal for the crash boundary of the fixture,
  runtime evidence and the rollback files.

### 2026-11-09 — Preserve Tommy's native weak blast reaction

- A live garage blast took 27.53 health and triggered a false physical fall. The controlled stock run kept
  Tommy on his feet at the same range while receiving the original damage.
- Tommy's optional living blast ragdoll now requires a type-2 hit above the native 100-damage threshold
  and a native fall animation. The engine handles light damage and medium hit animations directly.
- Twelve live checks passed per build, covering the two weaker ranges, the close fall and get-up, and
  same-mission loading. Port A2B8068D... installed in both labs and hot-reloaded in Mafia10Fixes generation 2.

### 2026-11-11 — Report body support to native car usability

- Tag car wheel hulls with native S_wheel indices, independent of compound part reordering. Physical
  body contacts below the tyres supply an additive support mask for the host's new optional
  `vehicle_support_hook`. The snapshot clears every physics step and world reset. Disabled car
  reactions, absent cars, hull/side contacts and sensors report no extra support.
- The native `ISCARUSABLE` predicate now receives the completed external contact state. Jolt keeps
  ownership of those contact forces. The loader restores the temporary on-ground additions after
  its native gameplay query and removes the provider at fault/unload.
- Both builds passed 15 host query/lifetime checks and the physical-body fixture with corrected
  bool-in-AL readers. With native road-contact mask zero, real wheel/body support made the
  predicate succeed. Driving away cleared it. A genuinely inverted native car stayed unusable.
  The exact owner story route was not replayed.
- Port E2B4012C... and loader B0845893... installed in both labs and Mafia10Fixes. The owner's saved
  garage state was restored after the update. Rollback files and the repeated fixture are under
  MafiaModLoader/build/wheel-support-20261111. See the loader journal for the corrected test ABI,
  native address witnesses, restart details and save-file verification.

### 2026-11-11 — Preserve the pedestrian pose at native car contact

- The dodge correction's sweep rollback also undid positions committed by its hit handler. Each
  sweep now owns its swaps, nested hit scopes mark all matching records committed, and restoration
  preserves native position changes and model replacement. Exception cleanup restores untouched swaps.
- Reproduced a separate 1.500 m start discontinuity in 1.0: moving the visible pelvis and invoking
  native car contact in one game-thread command still initialized physics from the previous tick.
  Capture the skeleton before the native hit, and use it only when the model and all 20 frame identities
  still match. A rebuilt skeleton uses the fresh current pose with no historical motion. Same-clip
  history uses the latest previous sample for relative limb velocity, preserving native bulk velocity.
- Added pedestrian_position regression executable. It and pedestrian_motion passed. The SDK fixture
  in MafiaModLoader/tests/game/test_pedestrian_handover.py passes four checks in each game build;
  ten existing native contact/get-up checks also passed per build with real traffic pedestrians enabled.
  Tests ensure the player is seated and the camera is within the native contact distance.
- ragdoll.state.car_handovers retains 16 bounded transitions: tick/serial, pre-hit and initial pelvis,
  maximum initial skeleton gap, chosen pre-hit pose, motion history, native sweep commitment and first
  solver-step displacement. Large pose/first-step changes alone emit concise log messages.
- Installed port 5A0DA231... / PDB E5FA51EB... in both labs and hot-reloaded Mafia10Fixes PID 25528
  generation 2. All 206 native save hashes were preserved. Source/runtime backups and the owner's four
  pre-update hits are under MafiaModLoader/build/pedestrian-handover-20261111. Those hits lack a pre-hit
  skeleton sample, so the owner's exact rare teleport has not been conclusively assigned to either bug.

### 2026-11-11 — Follow the reported body anomaly beyond initial creation

- The owner saw another unnatural fall and teleport after the previous update. Captured contacts 8/9
  have zero initial skeleton gap and first solver movements of 0.0414/0.0017 m. These do not explain the
  visible anomaly. The second native car trace is retained for 3.5 s; the first hit is outside that window.
- Retain up to 128 pose samples for each of the 16 recent car contacts, through age 3.5 s. Stages 1/2
  sample before/after tick pose application, 3/4 before/after render-begin application. Samples include
  physical/drawn pelvis, maximum skeletal origin error, fastest segment speed and native element position.
  Sample the first transitions densely, then at 0.1 s intervals; paused renders do not fill the buffer.
  Drop invalidates the tracked body pointer. Diagnostic faults are guarded separately from pose writes.
- ragdoll.state stays compact (sample count and maximum post-apply render error). ragdoll.handover reads
  a retained actor's samples after recovery. Physics settings and solver behavior were not changed in
  this follow-up. Seven real contact/render checks passed in each build, including actual pelvis fit.
- The SDK fixture explicitly probes the native car-hit distance gate rather than treating native success
  as proof a hit happened. Both builds bypass the 20 m distance test when element byte +0x50 is nonzero;
  its broader meaning is left unnamed. Distant ineligible contacts can merely deactivate the element.
- Port 3542AF0C... / PDB 7E294708... installed in both labs and Mafia10Fixes, with loader FC5257CB...
  The owner's mise15-mesto checkpoint was restored in new PID 26876. The intermittent visual report is
  still open; the new trace is for distinguishing physics displacement from model application on recurrence.
  Evidence and preimages: MafiaModLoader/build/pedestrian-handover-20261111 and build/cursor-return-20261111.

### 2026-11-12 — Observe the reported corner-to-corner pedestrian transfer

- The owner described right-corner impacts followed by bodies appearing at the left corner in
  Mafia10Fixes PID 21276. Added last post-world skeleton snapshots, identity/freshness validation,
  and stage-5 body samples, without changing initialization or collision response. After observer
  E4CA19BB... generation 2, the owner confirmed a recurrence. All six retained starts have zero
  previous-world/pre-hit and initialization gaps; pelvis matrices after world drawing follow physics.
  The last body accelerates around age 0.048 s. Matrices alone do not establish actual skin pixels.
- ragdoll.handover now also retains up to 192 car-contact records per transition for 3.5 s, matched
  by the body's opaque owner identity: car hull part, body id, normal, depth, sensor state, callback
  velocity, car origin/right/forward. Both per-transition and global contact losses are reported.
  Paused steps clear old contact diagnostics. An old unstepped row cannot claim a recycled body's step.
- Nine handover/render checks pass per build, including separate same-command matrix edits and
  post-world samples. The first 1.0 fixture failed with accepted=false/no transition after a successful
  camera probe; the next complete run passes nine. Native Bolt-Thrower corner drives record contacts
  in both labs; no exact owner transfer was reproduced. Some targets were missed; one crowded trace
  records global contact truncation. Those limitations are retained, not counted as clean reproductions.
- Port 2F791D64... / PDB 4B161450... installed in both labs and owner PID 21276 generation 3.
  Binary preimages and receipts are in MafiaModLoader/build/pedestrian-teleport-20261112, alongside
  the owner's six full recurrence traces. Another owner recurrence is requested. This is diagnostic
  progress; the intermittent visual report is not fixed or explained yet.
- Test games were closed and their original water manifests restored bytewise; owner PID 21276 was
  left running. Actual car axes, rather than the intended approach offset, identify the contact side.

### 2026-11-14 — Record pedestrian movement even without a physical body

- The owner's second `civil31.I3d!` incident also had three hits without body creation. See
  MafiaModLoader/build/pedestrian-trajectory-20261114/owner-incident.json for the old-process data.
- `ragdoll.trajectory` returns the latest contact or the latest for a named pedestrian. The observer
  keeps the last drawn pose if available, pre-hit data, the native hit event, handler return, native
  tick completion, physics step, ragdoll tick completion, render preparation and world draw.
  Samples track element, model, pelvis and physical positions, health, animation, and velocity.
  The hit records the car basis, damage and disposition of any body creation attempt.
- The budget is 16 contacts, up to 256 samples per contact for 1.6 s. The later phase rate is
  capped, missed samples are counted, and model/pelvis destruction closes the record. A position
  change of at least .75 m between samples within 200 ms syncs the first candidate to disk. The
  latest completed native contact is also saved. `pedestrian-jump.json` and `pedestrian-contact.json`
  in mod data are atomically replaced. The threshold selects evidence and does not diagnose a bug.
- Both labs used a controlled pelvis displacement followed by native vehicle contact, with
  physics disabled and with it enabled. All four traces captured the injected pre-hit change
  and all expected stages, saved a matching jump file and reported zero dropped samples.
  This validates the observer. It did not reproduce the owner's unforced visual jump.
- Deployed observer with SDK model alias fix to owner PID 17684, loaded profile 1 save 500 on
  pause. EAX verified. Backups and verification logs are in MafiaModLoader/build/pedestrian-trajectory-20261114.

### 2026-11-14 — Keep physical cars under the bumper camera

- The owner reported pass-through bodies and similar fallback pushes. In the owner process the
  TruckBx00 model root was hidden (0x2000003C), actor_meshes returned zero, and the recent handover
  had zero car contacts. The mod depended on rendering visibility when initially collecting a car.
- Vehicle collection now uses appended actor_meshes_ex with MML_ACTOR_MESH_HIDDEN_ROOT when the
  host supports it. Hidden child visuals remain excluded. The fallback keeps compatibility with
  older hosts. Shape input is the loaded model's finest-LOD native triangles, transformed to the
  car frame and assembled as per-visual convex hulls with native wheel identities. No box substitute.
  `ragdoll.vehicle` now reports the current or named car's actual physical compound for diagnostics.
- C SDK hidden-root fixture passes four checks per build. An isolated truck-driving fixture starts
  physics under camera 12 with the root already hidden. Both builds acquire 18 visible-child meshes
  (1375 triangles), produce measured car/corpse contacts and move an already physical corpse >1 m
  with >7 m/s peak part speed. Return to external camera preserves the physical car. Initial fixture
  mistakes and failed receipts are archived, alongside the corrected results.
- Installed port 97B7E11E8EB7ED5E4C3A5CB54FC1DD67633E940766B5C7DBF67DC3A6297C56B4
  with matching PDB in both labs and owner, on host 1BEB2DD9.... Owner's full native restore point was
  reloaded. PID 19940, camera 12, actual 3000 kg car compound registered, root hidden, 20 source meshes /
  1379 triangles in the current live car. Full EAX retained, ordinary time scale 1, native pause menu.
  The owner briefly saw the deliberate zero-time restoration hold as a hang. The process responded.
- The recent trajectory also captures native animation moving the model 5.202 m while the physical
  body moves 0.095 m before pose restoration. This remains separate from the missing car collider.
  Evidence and preimages: MafiaModLoader/build/vehicle-hidden-root-20261114. Both labs are closed.

### 2026-11-14 — Sweep only the current part of a traffic dodge

- The latest matching draw and handover records put the native 2.388 m jump before pedCarHit. The
  first physical pelvis step moves about 3 mm. The post-world CPU sample already contains the new root.
- Reproduced the native rewind in both builds by playing actual dodge 323 and inserting a native car
  across the already travelled path. The removed heuristic inferred a moving car by proximity. Replace
  that behavior with the normal collision sweep from the previously accepted element root, leaving the
  proposed animation endpoint, hit classification and correction to the game's own function.
- Both builds pass the native versus fixed dodge A/B, including a car ahead stopping movement, and
  all nine checks in the existing same-tick pre-hit handover case. The executable changes are confined
  to the sibling ragdoll DLL. Records and rollback files are in MafiaModLoader/build/pedestrian-teleport-20261114-2100.
- DLL 87379006... is installed in both labs and staged for the next Mafia10Fixes launch. The owner
  process still uses the previous DLL until restart to preserve the paused scene and its current bodies.
- Follow-up hot-loaded that exact DLL into Mafia10Fixes PID 27624, generation 2, once no physical
  bodies remained. Verified the live artifact hash. Sixteen checks pass per build in the final sequence.
  The intermittent get-up test error used a target killed by the synthetic hit. The fixture now checks
  that the same pedestrian model survives as a physical knockdown before dropping it into GetUp.
  No additional production change was needed for that test error. Owner replay remains pending.

### 2026-11-15 — Use collision forces for every vehicle-impact body

- The owner had a current native vehicle with 15 model pieces / 2442 triangles but no Jolt body.
  The latest contact showed an upwards/forwards velocity before physics and no car contacts. The
  loader now discovers and validates the occupied/last vehicle even outside the native general lists.
- Removed the artificial car-impact impulse, Ragdoll::Impulse and car_push. Vehicle-impact bodies
  have no animation-driven muscle targets. This covers both living knockdowns and lethal hits.
  The striking vehicle's actual model mover is registered at body creation and retained near its
  victim until the vehicle leaves or is destroyed. Empty hull extraction retries.
- The final live block passes 29 checks per build, including real bumper-camera truck/corpse
  contact, detached-car contact validation, absence of the launch impulse and pose continuity.
  Evidence is in MafiaModLoader/build/car-impact-20261114. The full offline suite has three
  outstanding failures detailed in MafiaModLoader/knowledge/notes/car-impact-20261115.md.
- Installed DLL 5505A21D... with host 34B23E81... in both labs and Mafia10Fixes. The owner scene
  mise07b-saliery was restored through the native restore point and left in the pause menu.

## 2026-11-18 — Clark's Motel, live settings and simulation costs

- The first 15 seconds of 04-03 previously produced four artificial vehicle hits on cinematic
  doubles in each game build. Supplemental car sweeps now apply to traffic pedestrians. Mission
  characters and Tommy still respond to native vehicle hits. The same save now has zero
  ragdolls in the opening on both builds.
- The master switch releases living actors and physics resources. The bullet switch also clears
  pending impulses and registered corpse shapes. Animated doors and changes to car-part membership
  update the hull while unchanged parts reuse their geometry. Existing cabin constraints survive.
- Solver, car motion and timers now use the full mission interval. Water exclusions follow each
  mesh placement. Pose snapshots sample each bone matrix once. Removed unused car reaction and
  splash outputs while keeping vehicle contact forces and buoyancy.
- Both labs passed 70 freeride checks and 13 pedestrian checks each. Four standalone x86 tests
  passed, including six 100 ms steps in Jolt and a shared terrain mesh moved across the waterline.
  In one 600-frame sample per DLL with 13 tracked actors, mod time went from 0.264 to 0.255 ms/frame.
  Installed in both labs and packaged locally as mafia-ragdoll-0.1.0-2026-11-18.zip, DLL EB850044….
  MafiaModLoader's knowledge/notes/ragdoll-audit-20261118.md records full hashes and limitations.
- The owner subsequently requested installation in Mafia10Fixes. The installed and live DLLs match
  EB850044… in PID 29072, generation 2. The previous DLL and PDB were backed up in the host checkout
  under `build/owner-ragdoll-20261118-201637-054203/`. The mission remained paused.

## 2026-11-18 — Player ragdoll hotkey

Press J on foot to start a living puppet and press it again to use the native get-up. The Bodies tab
exposes `player_hotkey` as a rebindable key. A seated or dead player, a paused mission and the master
switch guard the action. The key leaves a native knockdown on its normal recovery path.
Both labs passed all 15 ragdoll audit assertions (8 for the new bind) and the settings layout checks.
The owner's running Mafia10Fixes registered J after hot reload, with live DLL 8286544B….
The previous binaries and mod settings were saved in MafiaModLoader/build/ragdoll-player-key-20261118-202115-528752/.

## 2026-11-18 — Railway passengers

Get-up now checks the native walking floor in the selected carriage. The player returns to a
clear first-person view while the carriage moves. A descending passenger who has lost that
floor and fallen below it leaves through the engine's railway exit. If the street or a platform
has already been crossed, the mod restores the feet to it and clears the underground fall speed.
Open-air falls retain their position and speed.

Both labs exercised the streetcar, the elevated train and its trailing carriage, including
recovery after a mod reload. In the owner's paused 1.0 game the new DLL (3ADCB929…) returned
Tommy to the street with the same horizontal position and health. The host checkout records
the native functions, scenarios and before/after measurements in
`knowledge/notes/ragdoll-audit-20261118.md`.

## 2026-11-18 — Ready-to-install player archive

Removed the temporary J hotkey and its settings row. Bridge commands still exercise the same
gameplay transitions. The with-loader archive can be installed beside Game.exe on either build.
It includes the regular x86 host, core, wrapper, FFmpeg and x64 sidecar. Jolt is linked into the mod.
Both extracted installations passed 38 checks apiece. A separate hello_side mod installed afterwards
counted 64-bit frames and queried the player in each lab. The archive contains only core and ragdoll
under mml/mods. Mafia10Fixes hot-loaded the release DLL A20F181B… as generation 5, with zero ragdoll keys.
The archive is `build/dist/mafia-ragdoll-0.1.0-2026-11-18-with-loader.zip` (14,701,707 bytes).

## 2026-11-18 — Two player downloads

The mod-only ZIP writes exclusively to mml/mods/ragdoll, including its Jolt notice. The
clean-install download includes the same payload and a nested MML 2026.11.18-71d2377a base.
Extracting the outer ZIP preserves any installed host. The packer pins all base file contents.
Both labs passed 38 assertions with just the base and ragdoll. The host repository builds
the full runtime and optional video package. Both downloads are under
`build/dist/mafia-ragdoll-0.1.0-player/`.

Repacked the clean-install ZIP for one extraction next to Game.exe. All pinned base files
and the ragdoll DLL match the previously tested payloads. The mod-only archive remains
confined to mml/mods/ragdoll. Both installation layouts were checked by extracting them.

### Public release 0.1.0

The README links to MafiaModLoader and the player downloads. The requested backlog covers
dogs, tram drivers and passengers, and dragging a dead driver out when stealing a car.
The release assets use the tested one-extraction layout for both supported game builds.

The November 18 download labels put the bundle with MML 0.1 first. The second ZIP is
marked for players who already have the loader. Both retain the 0.1.0 ragdoll DLL.

### 2026-11-18 - Pulling a dead driver out of a car (0.1.1)

The native carjack starts paired animation tracks. The victim's track ends before Tommy lets go. Keep the pelvis attached to its seat during the reach, then apply a damped, bounded pull to the chest when Tommy's hands meet the torso and start to withdraw. The legs follow through their joints. The body still collides with the cabin and the street. The player's longer animation governs release.

The real-game regression uses the normal Action command, follows the physical pelvis through the door, checks Tommy's seat and moves the car away from the body. Both labs ran the scenario. The 1.2 lab also ran it with external Mafia Fusion Fix. The existing-ASI bundle adds MML under `scripts/` while preserving the player's proxy DLLs. Local 0.1.1 packages use the MML 0.1 host. The published 0.1.0 assets predate this change.

### 2026-11-18 - Holding a driver through the doorway (0.1.2)

An impulse on the chest left the dead driver behind Tommy's hands. The new grip joins a point on the torso to a collisionless kinematic anchor. The anchor follows both hands through every physics substep. The initial 0.18 s takes up the gap to a slumped driver. The joint lets the chest rotate and carries the pelvis and legs through the ordinary anatomical joints. The mod drops the grip on interruption, teleport, body removal and world teardown.

The Bolt-Thrower exposed a second issue: the moving body registered an impact against its own holder and knocked Tommy out of his animation. Contacts between this body and its holder now skip the walking push and knockdown response during the grip. The regression measures hand-to-coat distance and checks where the former driver lands in both game builds. Added the car explosion response of already-dead occupants to the backlog.

The final two-vehicle lab run passed 16 checks per build. Maximum sampled hand-to-coat error after take-up was 2 mm on the Bolt-Thrower and 35 mm on the freeride car. The owner's installed Community Modpack 1.0 also passed its eight checks with the four original ASIs still loaded. Adding the external Fusion Fix ASI reproduced a separate startup failure.

### 2026-11-18 - Falconer driver extraction (0.1.3)

On the Falconer the hands reach the belt while the previous constraint attaches to the upper chest, roughly 45 cm above the hands. Drawing that gap closed in 0.18 s caused the neck to reverse sharply. At grasp, rays from the hands now locate the surface of the nearest trunk segment. The joint stays on that segment until the hands open during the outward stroke. The body keeps its velocity on release. While the driver slides out, the mod reduces friction only against the occupied cabin.

The mod saves the latest extraction to `mml/data/ragdoll/driver-pull.json`. Run `python tools/analyze_driver_pull.py <capture>` for hand tracking, clearance relative to the outside tyre edge, settling and three-tick head motion. The log includes physical head position and neck rotation relative to the chest. The drawn head and nose points come from the engine's neck matrix after world drawing. Time is in simulation seconds.

At normal speed on the standard Falconer, the 95th-percentile angular deviation of the neck from its neighbours during the grip was 7.43° and 7.54° with the chest attachment (1.0 and 1.2). With the new attachment two runs per build measured 0.40–1.24°. In the final Falconer and Falconer Yellowcar runs on both builds, hand-to-coat separation after take-up peaked below 2 mm. Both feet settled at least 66 cm beyond the outer tyre line. All four scenarios passed the street landing, driver-seat entry and moving-car checks. Captures in the local `build/driver-audit/falconer-final/` directory contain 255 ticks each.

The Bolt Ace Pickup remains a separate geometry case. Its coat point briefly lagged by 11–15 cm in the wider test, despite completing the pull. An earlier Yellowcar run missed the follow-up seat interaction on 1.0. The final Yellowcar runs passed in both labs. The owner's Mafia10Fixes process loaded the final DLL (SHA-256 `bd73fc988e4911bfc27c06e580facbb305ccc3ab6103cfde774093c3e74d4397`).

### 2026-11-19 - Bolt V8 Touring traffic driver remains inside the car

The owner's 23:39:38 capture on November 18 contains all 255 simulation ticks. The game log and live actor lookup identified the vehicle as `Models/forvto11.i3d` and its driver as `civil02`. The grip lasted 0.72 s. After take-up, the coat followed 63.3% of hand travel and the worst separation reached 0.439 m. The pelvis moved 0.180 m outwards and settled 0.581 m inside the outer tyre. The earlier Falconer validation did not cover this model. The preserved owner capture is `build/driver-audit/owner-6e6dcd25b2bf.json`.

Driver-pull captures in the labs now include the source vehicle filename and, per tick, the deepest reported contact for each body segment against its own cabin mesh (triangle, source visual, world point, normal from cabin to body and penetration depth). The analyzer counts contacts pointing against outward motion while the settled grip lags by at least 5 cm. Missing contacts in this sample do not rule out other contacts or other colliders. Earlier captures can still be analyzed.

Spawning the exact vehicle model and driver in both windowed labs and using the normal Action key reproduced the failure. In the named-contact run on 1.0 and 1.2 the pelvis settled 0.49–0.50 m inside the tyre. The game allowed Tommy to enter the driver seat even with the former driver still in the cabin. The abdomen and lower chest repeatedly contacted the vehicle's `Box02` mesh with normals opposing extraction. The steering wheel also had contacts. The trace identifies the model mesh but does not establish a particular interior feature as the sole obstruction. Raw traces are in `build/driver-audit/touring-named/`. This is a failing gameplay regression and a working diagnostic on both builds.

### 2026-11-19 - Fit the driver to the seat back before extraction (0.1.4)

On the Bolt V8 Touring traffic model `forvto11.4ds`, the main mesh `Box02` includes the front bench. Triangles 16 and 17 form its broad sloping backrest face, 22 and 23 the narrow top, and 31 the side joining those surfaces. All use material 11 (`2BHA.BMP`). The local vertices and contact points in the two game builds agree. The pelvis and abdomen started across the backrest. The torso caught its top while the hands moved toward the open door. A secondary head contact was with the roof liner (triangle 108).

When the cabin becomes available, the mod probes both sides of the seat mesh at pelvis, abdomen and lower chest height. A correction requires at least two intersecting segments and a centre behind the front surface. The mod measures the clearance along the car's longitudinal axis and moves the articulated body together toward the knees before attaching the pelvis to its seat. This preserves joint angles and velocities. During the pull, the measured hand separation limits the turning moment on the coat. Jolt's internal-edge removal lets the body slide over triangle seams in the cabin.

Reducing the physics step during extraction still produced neck oscillations at the roof liner and was discarded. The final approach uses the existing neck joint's damped spring once the dying driver's active muscles have relaxed. Its target is the head angle at grasp, torque is bounded, and the spring is released with the coat.

The capture regression runs as `python -B tests/check_touring_pull.py <driver-pull.json>`. It checks the specific model, completed grip, tick spacing, hand tracking, clearance of the torso and both feet, release height and travel, jitter of the trunk and feet, final settling, neck rotation and the face point sampled after drawing. The original captures fail the check. Both labs also exercise the native Action interaction, a ground query on the pelvis, Tommy's driver seat and the corpse's position after the car moves.

The final six Bolt V8 Touring pulls (three per build, including two per build with the owner's type 5 hit and 101 damage) each produced 255 normal-speed ticks and passed the capture regression. The coat followed 99.31–99.97% of hand travel; maximum separation after take-up was 42.9 mm. Across individual pulls, the 95th percentile three-tick neck residual ranged from 0.87 to 3.00 degrees, and the drawn face residual stayed below 7.1 mm at that percentile. The nearest measured foot ended 0.52 m beyond the tyre. Neither build recorded a rise of the pelvis after release. Raw captures are in `build/driver-audit/touring-014-neck/`.

The two additional Falconer pulls passed the gameplay checks, giving 12 of 12 checks on each build. The DLL installed in the owner's `W:/Mafia10Fixes` matches both lab copies (SHA-256 `934a7b10ea89b79c5d3609530c41ad5ad9373a802cbd7b63ca153b8c3128ba64`). The previous files are in `build/driver-audit/owner-before-014-20261119-191656/`.

### 2026-11-19 - Match the physical driver grip to the outward stroke (0.1.5)

The owner's Falconer capture (`Models\\blackha11.i3d`, civilian `civil52`, hit type 5, damage 101) showed that the old grip engaged at 0.86 s. Tommy's hands were still reaching inward until 1.41 s. The head moved 0.696 m toward the open door during that reach. The grip released at 2.13 s during the fast outward stroke. Over the next 80 ms the pelvis travelled horizontally at 2.71 m/s and it moved another 0.736 m out from the car before settling. The capture field `coat` is a material point on one rigid trunk segment. The mod has no independent clothing simulation.

Measure hand movement across the door in the model frame, take up the grip after an inward reach turns outward, and support the driver as the outward stroke slows and reverses. On release, contact with Tommy stays filtered until a physics step finds no overlap between his collision shapes and the driver's body. Subsequent contact with him is handled normally. The frame comes from the vehicle model and also works before its collision mesh is ready. The regression in `tests/check_falconer_pull.py` checks head movement before the hands withdraw, grip proximity, horizontal release speed, outward travel and the final position of the torso and feet. The owner's capture fails its inward-reach check.

Validation: `build/driver-audit/falconer-015-contact/` contains four Falconer and four Bolt V8 Touring captures split equally between 1.0 and 1.2. All eight passed their capture checks and the two labs each passed 12 gameplay assertions. On the Falconer the head moved 4–5 mm outward during the reach. Horizontal pelvis speed over the first 80 ms after release was 0.37–0.70 m/s; remaining outward travel was 0.19–0.30 m. The separate world-step regression passed. The owner installation in `W:/Mafia10Fixes` matches the tested DLL (SHA-256 `e9b84929d2f6bf1871fcae39373a0f225813c6edc22275db3cf43e0d04b41f83`). Previous files are in `build/driver-audit/owner-before-015-20261119-210331/`. The running game was left open on its existing generation.

### 2026-11-19 - Traffic occupant death before carjacking

The owner's 1.0 session (PID 14512, ragdoll generation 1, version 0.1.4) distinguishes two events. The traffic car at `0x2506FDB8` received bullet hits at session times 1587.4, 1592.3 and 1593.0 s. Its live traffic record still held a driver model and the car's native dead-driver flag (+0x2071) was 1. No human-hit event for this occupant appeared in that interval. In an earlier extraction, bullet hits on car `0x2511A3A0` at 1549.5 s preceded the human's type 5, damage 101 event at 1550.6 s; the ragdoll started when the human event arrived.

The native traffic occupant hit handler (1.0 0x506C00, 1.2 0x4BADD0) selects the occupant model, assigns `smrt auto ridic01.I3D`, advances the animation to its end and marks the driver dead on the car. It does not call the normal human hit handler there. The 0.1.5 pull tests use a seated human actor and cover the later extraction; they do not exercise shooting a traffic occupant before conversion. No runtime change was made for this finding.

### 2026-11-19 - Transport occupants at death (0.1.6)

Capture the native traffic model before the occupant death handler advances its animation. A hit on the car frame alone leaves the occupant alone. The driver's native death flag also matters when traffic recycles a car and model in place. If the driver becomes a human actor with the same model during extraction, transfer the physical body and its seat constraint.

On both builds S_rail_human::Kill (1.2 0x484230, 1.0 0x4D5160) supplies the person and the railway. Its inline driver is also in the passenger roster. Keep the model through the death clip (state 17) and its completion (passenger 18, driver 20). The stock tram driver stands. Use the actual thigh pose to distinguish standing from sitting. Each stock carriage has a private walking grid of oriented boxes, including the rear car of the elevated train. Use those boxes in the carriage's own frame and constrain only seated pelvises. The solver creates the seat before its first collision step and filters out other vehicles' cabins.

Both windowed city freeride labs passed `tests/check_transport_occupants.py` on the installed binary. The test exercises the two native kill handlers and samples the neck after the world is drawn. All four bodies (traffic driver, tram driver, tram passenger and rear-car train passenger) were still simulated more than three seconds after death. Separate shot-queue trials registered kills on traffic drivers, tram drivers, tram passengers and rear-car passengers on both builds. On 1.2 the traffic fixture fired through a visible driver model in a traffic-controlled vehicle. Moving-carriage sampling included a rear car travelling over 40 m with the passenger aboard. Visual assessment is the owner's.

Both labs passed the Falconer and Bolt V8 Touring extraction checks, including Tommy taking the seat and driving away. Each pull recorded 255 ticks. Both capture analyzers passed, as did the world-step regression. Captures are in `build/driver-audit/transport-016-pull-regression/`. The owner's `W:/Mafia10Fixes` installation matches the tested DLL (SHA-256 `939182cd0a5530d26e47b5cf86c691b7278a231530bfa43dadbf07e2e1eb1389`). Its previous files are in `build/driver-audit/owner-before-016-20261119-213201/`.

### 2026-11-19 - Rail cabin surfaces and bullet contacts (0.1.7)

The owner's 1.0 tram had four simulated bodies. Three passengers still held their heads almost directly above their pelvises after 25 seconds. The walking grid places tall invisible side boxes along the benches, and the car seat constraint kept seated pelvises in place. Rail bodies now use the posed carriage model with free pelvises and passive joints. For a rear carriage, read that carriage's own model and update the transforms and visibility of its visual parts. In a four-person tram trial on each build the standing driver's hip fell from 1.65 m to 0.73 m above the carriage origin; some passengers settled on the bench and others fell toward the floor.

During a fatal shot the native vehicle Hit identifies the occupant model. For dead riders the native dynamic ray can return empty even through the centre of a physical body. TickShoot's return from collRayTest (1.2 0x5A935C, 1.0 0x5E4CAC) supplies the travelled segment. Limit it to the static impact and check the carriage mesh from the bullet's origin so the floor can block a shot across successive frames. Impulses go to the first intersected body part.

Both windowed freeride labs passed `tests/check_transport_occupants.py`. In each, shots from the aisle registered one local impulse per shot on the tram driver, a passenger and a rear-car train passenger. Shots at those targets from under the floor registered zero. Separate native shot-queue trials confirmed a lethal shot and impulse on all three roles in both builds. All pose measurements come from the game and Jolt; visual assessment is the owner's.

The DLL in both labs and `W:/Mafia10Fixes` has SHA-256 `739b119e6f4a389ae81a07421649b6361b587f141a79985a03a147c180883ba1`. Owner rollback files are in `build/driver-audit/owner-before-017-20261119-220307/`.

### 2026-11-19 - Shared passive joint resistance (0.1.8)

The owner's 1.0 log showed the tram driver and three passengers on the same carriage in 0.1.7. The reported bench pose left the head unsupported. Rail occupants had zero friction torque in every joint. All bodies still had angular drag in world space. A car driver's neck also had a separate position motor while Tommy held the driver.

Every passive ragdoll now uses the same anatomical settings for each joint. Jolt resists the child body's rotation relative to its parent. Hinges use rotation about their allowed axis. The neck has very little dry friction and a velocity-dependent torque. The spine and hips have higher dry friction to support the trunk. A hand grip applies its load to the driver's trunk, with the neck following the ordinary joint law.

In the isolated Jolt regression the torso support is fixed and the 6 kg head hangs under gravity. Give it a single 2 N s transverse impulse and disable sleeping. With the old rail resistance, maximum deflection between three and four seconds was 12.09 degrees. The shared neck settings gave 0.20 degrees at both 30 and 120 Hz. The initial excursion was 7.41 and 5.73 degrees respectively. These are solver measurements, not an angle measured from the owner's screen.

Both builds passed the Falconer and Bolt V8 Touring extraction captures in `build/driver-audit/passive-joints-018-anatomy/`. The first tuning allowed the head to move 15 to 17 cm towards the door before Tommy grasped the driver in the Falconer. With the shared spine and hip settings the same reach measured 1.1 to 1.3 cm. Both vehicles passed their capture analyzers and Tommy could take the seat and drive away in both builds.

The final DLL passed the transport occupant regression and all 34 lying-pose anatomy checks on each game build. After five seconds in separate stationary tram trials, the passengers' necks measured 0.77 to 1.40 m above the carriage origin, from starting heights of 1.78 to 1.79 m. Both standing drivers stayed near the controls in those trials. The pendulum test isolates the bench-edge condition; the exact pose in the owner's session was not replayed.

The owner installation in `W:/Mafia10Fixes` and both lab DLLs match SHA-256 `c94188de3059bae58baa532a3f8e214d28f35e5bd76bfb64c34fea849993cd93`. Previous owner files are in `build/driver-audit/owner-before-018-20261119-222127/`.

### 2026-11-19 - Passenger ejection and street contact (0.1.9)

In the owner's running 1.0 game on 0.1.8, passenger `0x1996E200` from tram `0x0F7682B0` had a pelvis at Y = -3955.105 m. The other four ragdolls in the same tram were at Y = -4.62 to -4.87 m. Both the broad-phase filter and the layer-pair filter excluded static scenery for every vehicle occupant. A loose passenger also kept the original carriage association after leaving through a door.

Occupant parts now contact static scenery throughout ejection. A hand or foot can reach the pavement while the pelvis remains inside. Before updating the carriage pose, a loose body leaves the cabin collision layer once all 15 part bounds clear the visual mesh bounds in carriage coordinates. The transition keeps the existing body transforms and velocities. The original roster still owns its visual model. A carriage jump only carries bodies still associated with that carriage.

Both windowed freeride labs reproduced the fall on 0.1.8 with a native passenger initially outside the tram. On 0.1.9 the doorway fixture starts the native passenger overlapping the carriage and fires live projectiles. Six hits (1.0) and seven hits (1.2) pushed the tested passengers clear. After five seconds their pelvises were 0.47 m and 0.21 m above the street respectively. Both bodies accepted a subsequent hit. Bodies still inside travelled 12 m with a moving carriage on each build. The static-floor regression in `tests/world_step.cpp` keeps the test limb on the occupant layer for the entire fall and checks the resting height.

The repeatable live regression `tests/check_rail_exit.py` aligns the hips at the tram edge for either seated pose before shooting. It passed twice per build, with the pelvis resting 0.14 to 0.23 m above the street. The native transport occupant regression passed on both builds. The Falconer and Bolt V8 Touring passed their gameplay and capture checks (`build/driver-audit/rail-exit-019/`).

The installed DLLs in both labs and `W:/Mafia10Fixes` match SHA-256 `8a966101479d27be6827bb21f5389b19f768e82072007ee723dc7f1397b0a624`. Previous owner files are in `build/driver-audit/owner-before-019-20261119-223359/`.

### 2026-11-20 - Driver footwell placement and two-hand support (0.1.10)

The [0.1.10 release notes](RELEASE-0.1.10.md) cover changes since the previous public release, 0.1.0. All eight saved extraction captures passed again at packaging time. Both archives carry the lab-tested DLL. The clean bundle matches the public MML 0.1 base file for file, with the host moved to `scripts/`. The smaller archive owns five files under `mml/mods/ragdoll/`.

The owner's ambulance capture showed a foot 0.46 m inside the outer wheel line after release. On the same vehicle in both labs the old seat-back correction shifted the whole skeleton forward by 0.16 m. Before Tommy even reached in, the shin penetrated the cabin by up to 0.074 m. During the pull the shin and foot reported opposite normals on the same panel. Moving the grip from the pelvis to the abdomen still left a foot inside on both builds.

When fitting a trunk to the backrest, test the displaced leg shapes against the actual cabin. Keep the original leg pose if it fits within the solver's contact tolerance. An obstructed leg uses a two-link hip-to-ankle solve in its existing knee bend plane. Evaluate the thigh, shin and shoe together against the seat and footwell, choosing the smallest combined overlap and shoe displacement. This runs when attaching the seat. At grasp the mod records the line between Tommy's hands in the torso's local frame. Changes to that line apply bounded restoring torque through the hand span. The body can roll around it and retains its momentum at release.

A separate capture with the mod unloaded confirmed that the native victim track turns the legs and carries the pelvis about 2.5 m out from the ambulance seat. Guiding the shins from that track did not resolve an initially embedded foot. The installed implementation keeps the legs dynamic throughout the pull. The regression in `tests/check_ambulance_pull.py` checks shin penetration before grasp, hand tracking, both feet clearing the wheel line, release height and head motion after drawing.

The final windowed 1.0 and 1.2 runs in `build/driver-audit/pull-footwell-validated/` each passed 12 gameplay assertions. All eight normal-speed captures (two ambulance pulls plus a Falconer and a Bolt V8 Touring per build) passed their vehicle-specific regression. In the four ambulance pulls the closest foot or torso ended 0.22 to 0.48 m beyond the outer wheel line. The fitted leg shapes began with 0.085 and 0.055 m of overlap on the two sides, reduced below 0.001 m. During the grip the 95th percentile of the drawn face's three-tick residual was 0.0024 to 0.0025 m. The original owner capture fails the shin-penetration check. These lab runs used spawned seated people and the native Action interaction.

The tested DLL matches the installed file in `W:/Mafia10Fixes` (SHA-256 `500a76cdfdfa5b93a290fd95f53894461a734ccbe4710ffe772d9462d80349d1`). Previous owner files are in `build/driver-audit/owner-before-01010-20261120-202558/`.

Additional live-traffic captures are in `build/driver-audit/pull-traffic-01010b/`. Both native dead drivers transferred into the carjack and both pulls completed. On the 1.0 roadster (`forVro12`) one foot rested against the car, 0.037 m inboard of the tyre line. On the 1.2 truck (`fordtru10`) the feet cleared the truck, but the torso lagged up to 0.143 m behind the hands and the chest briefly reached 10.09 m/s. Fresh truck fixtures reproduced 0.15–0.16 m lag on both versions. In the geometry capture the pelvis encountered the fixed side of `Object01`, triangles 96 and 221, just aft of the opening. A higher initial seat fit and a grasp farther up the trunk did not remove the obstruction. A laboratory route shifted the grip forward 0.24–0.28 m through the opening and reduced lag, but separated it from the stock wrists and failed the landing check on 1.2. Those experiments were left out of 0.1.10. Further work on the truck needs to fit Tommy’s reach and the driver’s path to the same opening and measure the drawn wrists against the physical contact.

### 2026-11-21 - Where Tommy takes hold in 0.1.10

At the first recorded grip sample in the eight validated pulls, the wrist midpoint on the Falconer was 46–47 cm below the driver's physical chest centre. The constraint point was 7 cm above the pelvis centre on both builds. The wrists were 31 cm below the chest centre on the ambulance and 25 cm below it on the Bolt V8 Touring. These are vertical differences from `pull-footwell-validated`, measured before the gap to the hands was taken up. The capture does not record which trunk segment won the nearest-surface test.

For a preliminary reach estimate, compare the Falconer grip sample on each build to the native paired-animation sample in `native-pair-probe-1` with the smallest maximum left/right wrist position difference (about 4 cm). Both arms measured 58 cm from shoulder through elbow to wrist. The left shoulder was 64 cm from the ragdoll chest centre and the right shoulder 45–46 cm. These samples come from separate runs. A chest-grip prototype should test two reachable points on the near chest surface in the same frame, preserve arm lengths and elbow bend, and measure the drawn wrists against the physical contacts through release.

### 2026-11-21 - Traffic driver identity and the chest-grip prototype (0.1.11)

The owner's 1.0 log showed a traffic driver dying at 08:24:13 and another ragdoll starting from rest on a human in the same car at 08:24:14.923. Earlier traffic tests established that the pull finished. They never checked whether the original body survived. The ordinary seated-human fixture skips this transition entirely.

In both executables, `C_human::Do_ThrowCocotFromCar` duplicates the model returned by `C_traffic_car::Corovacka`. Immediately after duplication (1.2 at 0x5882B9, 1.0 at 0x4A8829), record the exact source, clone and car. Match the clone and its seat car during the synthetic lethal hit in that tick. Rebind the existing Jolt ragdoll to the clone's skeleton before drawing. Its rigid bodies, velocities, joints, seat and serial survive.

`tests/check_traffic_pull.py` takes a live traffic driver, invokes the native occupant death handler, waits for the actual car to stop, reads its driver-door approach track, and uses the normal Action interaction. It compares the original serial, pelvis, head and drawn head with the first extraction frame. Both pulls completed. The physical shift was at most 1 mm. Drawn head displacement was 6 mm on 1.0 (`forade11`) and 17 mm on 1.2 (`fordtru10`). Captures are in `build/driver-audit/chest-ik-traffic/*-action.json`.

Tommy now aims at two points on the driver's physical upper chest, using a spine and clavicle adjustment and a two-link solve for each arm. The draw pass records the wrists and restores the native animation afterward. The grip uses the same chest surface and accounts for reach and overhead cabin clearance. In six stationary seated-human captures (Falconer, ambulance and Bolt V8 Touring on both builds), maximum wrist-to-contact error while holding was 0.4 mm. All 18 gameplay assertions passed. In the 1.0 ambulance capture one shoe still rested 0.175 m inside the wheel line. See `build/driver-audit/chest-ik-validated/`.

The traffic traces distinguish model replacement from a later lurch at grasp. The left wrist was still 0.26 m short immediately before grasp on the 1.0 car and 0.47 m short on the 1.2 truck. The truck chest then moved 0.23 m in the first held frame as the target was projected into reach. The 1.0 traffic car also retained a foot in the doorway. Further work on the pull should use measured hand contact to engage the grip, adapt Tommy's approach to the seated pose and keep the anchor continuous at engagement. Check both traffic and seated-human captures.

A second Action run with `foraro13` on both builds preserved both drivers across promotion. The 1.2 driver was extracted (3 mm physical shift and 9 mm drawn head shift at promotion). On 1.0 the driver had settled across the cabin before the pull. At 1.2 seconds into the animation the chest was 1.05 m on the opposite side of the car's centreline and the native wrist midpoint was 0.30 m on Tommy's side. No grip formed. This is a separate failure from recreating the body. The two repeat traces are in `build/driver-audit/chest-ik-traffic/*-action-repeat.json`.

Installed this local test build in `W:/Mafia10Fixes`. The installed and both lab DLLs have SHA-256 `ba979b52a85c163ab5171232a95d872279f3b723b7ded7e27b00488307886062`. The previous owner files are in `build/driver-audit/owner-before-chest-ik-0111/`.

## 2026-11-21 — Two-handed driver grips (0.1.12)

In the owner's `pontfor11` capture the chest jumped 0.416 m at engagement. The pelvis was 0.49 m closer to the driver door. In `chemafor12` both hands had already arrived when the pull began, but chest travel later reached 0.067 m in 16 ms. The two captures are `build/driver-audit/owner-pull-2186.json` and `owner-pull-5586.json`.

Use the same spine, clavicle and arm reach calculation for drawing and for physical contact. Corrected the rotation direction for LS3D row vectors. Only ribcage, waist, hips and the upper half of each thigh supply paired contact points. The load starts on the existing surface after both hands reach it, stays on those material points and follows the pull with limited acceleration. The trace records each wrist, its own contact point and the selected part.

On 1.0 and 1.2, the cross-seat regression reproduced the owner's pose with the chest 0.49 m farther into the cabin. Both runs selected the pelvis. Chest displacement was under 0.4 mm in the first held frame, and the chest, pelvis and both feet cleared the driver's wheel line. Eight more runs covered the Falconer, `chemafor12`, ambulance and Bolt V8 Touring. All 24 gameplay checks passed. The ten captures also passed the wrist, grip spacing and grip travel checks. See `build/driver-audit/adaptive-grip-a/c-cross-*.json` and `adaptive-grip-c/`.

Traffic still has two distinct limits. On 1.0 the best simultaneous wrist contact in the `forade11` trace was 9 cm short, so no grip formed. On 1.2 the `fordtru10` driver retained the original body and was held at the pelvis for 0.62 s, but the final chest, pelvis and feet were inside the wheel line. A successful two-hand grip alone does not prove extraction. Keep these traces for work on Tommy's approach and the loaded portion of the pull.

Installed DLL SHA-256 `6372f34295b9894261df930c34daf8bf4a8b9e7f77e70e8a26f702dee9a42945` in `W:/Mafia10Fixes`. The previous files are in `build/driver-audit/owner-before-adaptive-grip-0112/`.

## 2026-11-21 — Bullet impact (0.1.13)

The default impulse per damage point rose from 1 to 2 N s. The per-shot cap rose from 120 to 240 N s and the local speed allowance from 8 to 16 m/s. The native hit handler distinguishes bullets from other blows, including when carrying recent hits into a new ragdoll. Traffic and railway occupants use the same shot settings. A native projectile to each palm in both windowed labs gave a measured local forearm velocity change of 8.0 m/s on 0.1.12 and 16.0 m/s on 0.1.13. All four shots hit the matching forearm on each build pair. The owner game loaded 0.1.13 as generation 4 in the running process.

## 2026-11-21 — Shots at exposed railway occupants (0.1.14)

On 0.1.13 a real shot through a driver's head hit a second target placed 0.65 m behind it in both builds. The rail-body test skipped the entire stretch whenever the native moving-object ray returned a primitive. At the existing TickShoot hook (1.2 0x5A935C, 1.0 0x5E4CAC), both executables pass the native hit position at ESP+B0. Clip the rail-body test to that point and the static impact, then terminate a body-first shot through the game's range check. The frame caches the endpoint at ESP+BC and the squared range at ESP+14C. This prevents a second physical hit farther down the same shot.

In both windowed builds, `tests/check_rail_exit.py --driver-window` checked both target orders. Only the nearer target received the impulse. A separate shot from the street towards the tram struck the driver's head 1.53–1.54 m from the carriage centreline. The ejection test also hit a passenger's head on the street after it separated from the tram. The transport suite confirmed that the carriage floor still blocks shots at a driver, a tram passenger and a rear train passenger. Its first run in the small city stopped at a traffic driver's drawn-neck tolerance before testing shots. The full run in the standard city passed on both builds.

The owner game hot-loaded 0.1.14 as generation 5 in PID 6896. DLL SHA-256 is `91db8e0a989f62caea2ca188bd467d04457de6f8922702b3b7dee15df79c668a`.

### Owner report of unusual elbow travel during a pull

The 09:48:34 traffic carjack used `ForAcou13.i3d` and kept body 5800. The original 255-sample trace is archived locally in `build/driver-audit/owner-elbow-5800-5e74adfa.json`. All 57 held samples use the pelvis, with maximum wrist-to-contact distance below 0.4 mm. At 2.368 s the grip was already open while the IK weight was still 1.0. Over the next 176 ms the drawn right wrist travelled 0.620 m along its path. The recorded native right wrist travelled 0.166 m in that interval. The maximum separation was 0.426 m at 2.400 s. These are wrist measurements. This trace lacks Tommy's shoulders and elbows, so it cannot establish which elbow deviated or its trajectory. A future capture for this question should record the native and drawn shoulder, elbow and wrist together on the same frames of the actual interaction.

## 2026-11-21 — Arm recovery and detachable headwear (0.1.15)

The arm solver transports the animated bend plane with the shoulder-to-wrist axis. After contact ends, it applies the last local joint corrections to the continuing native animation and decays them over a duration derived from the reach distance. Both arms and the upper spine recover together, independently of the released driver. Each rendered sample records shoulders, elbows and wrists before and after IK. The analysis reports the time and speed of the largest elbow correction by phase and side. The latest capture and its two predecessors use `driver-pull.json`, `.prev` and `.prev2`.

In both windowed labs an intermediate fixed-duration crossfade moved the right wrist 0.68–0.69 m over the first 176 ms of recovery and the peak speed of the elbow correction relative to the shoulder reached 2.91 m/s. In the final ForAcou13 runs the right wrist travelled 0.333–0.334 m over that window, with peak elbow correction speed 1.17 m/s. Each capture contained 255 ticks. Both held wrists stayed within 0.5 mm of the contact points. The driver landed at the door, Tommy took the seat and the driver stayed on the street when the car moved. These captures used a chest grip, while the owner's archived interaction used a pelvis grip.

The mod reads installed 4DS character files, identifies headwear by material and checks the full-detail faces against the neck's rigid weights. Some distant models weight the same material to the chest. The derived character uses the original rigid hat as a separate neck visual at all distances, and the detached prop uses the same triangles, UVs and materials. The mod registers generated files from its private data directory with the loader. Existing file overrides from other mods take precedence at preparation. Unsupported layouts and models without identifiable headwear keep their original geometry. A model Mafia cached before a hot load gains the new visual when its file is next loaded.

On both builds 114 stock models passed conversion. `tests/check_headwear.py` checked two civil15 instances, two TommyHAT instances and bareheaded Tommy in each lab. Only the selected wearer's visual switched off. Its matching physical world prop travelled 1.53–1.61 m and the bare model produced no prop. A zero-velocity physical prop initially slept in the air. A stationary hat now starts with the first gravity step. Moving hats inherit the head velocity at the hat centre, including the contribution from head rotation. The owner DLL is `5df5b51ead3ead66fb4f2ed90dd71f546eaaf755f4d121f652f21b858e9d325f`.

The final blackha11 and forvto11 runs also passed all three gameplay checks in both labs. Together with ForAcou13 these are six extractions. The three on-disk trace generations were present after consecutive pulls. The owner's existing 1.0 process (PID 6896) hot-loaded generation 6 at 10:23:50, prepared 114 models and loaded the same DLL hash as the labs.

### Open crown after a hat falls

The owner's screenshot was saved at 10:25:47. The preceding detachments were freegang01 at 10:25:10 and civil19 at 10:25:26. The screenshot does not identify an actor. Compared the original A2.dta meshes with the generated wearer and prop in the owner's installation. At full detail all 62 civil19 face triangles (COBL10.BMP) remain in the wearer with identical indices and vertices. Only the 54 CKLB05M.BMP triangles move to the prop. All eight vertices of the open upper face boundary coincide with the original hat boundary. A vertical ray through the centre of that opening crosses only the outside of the hat. Freegang01 also has an open crown and retains all 279 ENEMY02K.BMP triangles. On civil19 a possible repair is to transplant the missing scalp surface from a bareheaded stock model. Civil34, civil42 and civil56 use the same face texture and have 84 face triangles and no open upper boundary. Matching the seam and UVs requires a separate implementation and validation.

## 2026-11-21 — Closed crowns under detachable hats (0.1.16)

The model converter finds the open skin boundary beneath the brim, follows the wearer's outline and builds a crown using its face material. The added surface is fitted inside the corresponding stock hat, with the seam attached to the original vertices. It reuses spare rigid vertices on the same bone and excludes vertices touched by facial morphs. The worn hat keeps its native distance levels. A cached 0.1.15 wearer keeps its hat until Mafia reloads the character model. The world prop still uses the full-detail hat.

Both windowed labs registered 114 wearers. The independent skin test found upward-facing skin at five points inside the crown on all 212 native detail levels in each build. A comparison with each build's original assets found 187 repaired levels and 25 already closed. All surviving body triangles retained their original indices and vertex data. Eight models shared headwear materials with distant hands. All 112 hand triangles in those groups were preserved. Tests also compare the distant hand geometry against the game's own files.

In each lab two copies of each of civil19, freegang01, TommyHAT, pauliecoathat and czena08 were spawned together. Only the struck wearer's hat detached, and each physical hat moved at least 1.50 m in the final run. Bareheaded Tommy produced no hat. These are geometry and gameplay checks. They do not assess the appearance of hair on every face atlas.

The owner's 1.0 process hot-loaded 0.1.16 at 10:49:19 and prepared all 114 models. The installed DLL and the live DLL in the subsequent freeride session both have SHA-256 `dcaf20de80e7c5cb97074af7c6f871e34a981653aae5058e45b314b467a34f36`.

## 2026-11-21 — Hat retention and lining (0.1.17)

The fit tracks the simulated crown's point velocity. Gravity relative to crown acceleration loads the hat sideways or lifts it. Pressure into the head increases lateral grip. A damped fit returns toward its seat during small disturbances. Four centimetres of accumulated slip releases the hat. Its world placement comes from the current physical neck and the worn visual's local transform. The prop inherits head point velocity plus relative slip. Each release records ragdoll age, head orientation, slip, position and velocity.

The detached prop has an inward-facing shell with reversed normals and stitched open edges. A shared 64 × 64 opaque cloth texture includes a dark sweatband. The original exterior and character models are unchanged from 0.1.16. Versioned prop names bypass old cached geometry and work with already loaded 0.1.16 wearers.

In both windowed labs all 114 registered props passed the interior triangle test and all 212 wearer detail levels passed the crown tests. Civil19, freegang01, TommyHAT, pauliecoathat and czena08 kept their hats on activation and released during the fall after 0.544–0.624 s across ten trials. Matching bystanders kept their hats and bareheaded Tommy produced no prop. From below, civil19, freegang01 and TommyHAT covered the sky through the opening in each build. The archived civil19 screenshot shows the brim, inner wall and crown. The fit regression passed at 20, 30, 60 and 144 Hz for upright travel, a steady 30-degree lean, free fall, inversion and a lateral jolt.

The owner's 1.0 game hot-loaded generation 2 of 0.1.17 at 11:01:16, prepared 114 wearers and logged a civil17 hat releasing after 0.197 s. The installed and live DLL hash is `39c3d80d57749c9ad8bf59390c28dd36f9fafe8cc897bbbaf4fd5c7cc721fbc1`.
