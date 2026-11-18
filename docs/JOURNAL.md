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
