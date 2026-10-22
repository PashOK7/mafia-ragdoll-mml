# mafia-ragdoll-mml: guide for AI agents

Physics ragdolls for **Mafia: The City of Lost Heaven** (2002; builds 1.2 and 1.0) as a **MafiaModLoader mod**
(`ragdoll`). A private repository. Its reference is mafia-ragdoll (`../mafia-ragdoll`, read only: a host of its own,
its PhysX smoke and water are not a part of this); its physics (`ragdoll`, `jolt_ragdoll`, `jolt_world`, `knock`,
`collision`, `tunables`) is ported as it was, its binding to the game is MafiaModLoader's services.

Read MafiaModLoader's `AGENTS.md` first (the labs, the rules, the MCP tools: they are this mod's too), then this file
and the end of `docs/JOURNAL.md`.

## The loop

```bat
cmake --preset x86                       :: once (MafiaModLoader beside this repository, or -DMML_ROOT=...)
cmake --build --preset x86-release       :: build/x86/out/mml/mods/ragdoll/
python tools/deploy.py                   :: into both labs, reloaded in a running game (--game 1.2 for one)
```

Then through MafiaModLoader's MCP (`mafia`): `game_launch`, `call ragdoll.state`, `human.hit` (a death, a knock-down),
`hits.take`, `draw.world`, `screenshot`, `log_tail`. A crash: `crash_triage`; a fault at `mod_ragdoll_*.dll+0x...`:
the function is in `build/x86/ragdoll.map` (the linker's map).

## Map

| Path | What |
|---|---|
| `src/main.cpp` | the entry: the build detected (`game::Detect`), the events, the commands |
| `src/core.cpp` | who becomes a ragdoll and when, the tick, the bones written, the hits, the knock-downs, getting up, the trams, the dodge fix |
| `src/ragdoll.cpp` | a ragdoll's binding to the skeleton (15 parts, the bones written through `bone_set`) |
| `src/jolt_ragdoll.cpp` | the bodies, joints and muscles in Jolt |
| `src/jolt_world.cpp` | the rigid-body world: the static scenery, the cars, the people near |
| `src/scenery.cpp` | the static world from the loader's `scene_pieces` (water faces left out) |
| `src/vehicles.cpp` | the cars' bodies from the loader's `actor_meshes` |
| `src/knock.cpp` | bodies knocking living people over |
| `src/ragcam.cpp` | the ragdoll camera: the player's body circled by the mouse while he lies and gets up (`camera_override`) |
| `src/fmt.h` | printf into a string of any length |
| `mod/mod.json`, `mod/config.json` | the manifest with the settings page (Options > Mods: Bodies, Forces, Camera); the defaults |
| `tools/deploy.py`, `tools/check_settings.py` | into the labs; the settings page laid out and judged by the loader's own check |

## Rules

- Everything here is English; talk to the owner in their language.
- Both builds: what changes the mod is seen working in both labs before it is called done.
- What the mod needs from the game and the loader does not give: a service of the loader (in MafiaModLoader, with its
  tests and docs), not a raw hook here, unless it is the mod's own logic (the dodge fix is).
- Commit by block; never push without the owner's word.
