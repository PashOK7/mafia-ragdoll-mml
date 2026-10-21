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

