# Ragdolls for Mafia

Physical bodies for **Mafia: The City of Lost Heaven** (2002), builds 1.2 and 1.0, as a
[MafiaModLoader](https://github.com/PashOK7/MafiaModLoader) mod. The dead fall where and how they were struck, the
knocked down tumble and get up the game's way, and the bodies stay in the world: bullets push them, cars feel them,
people trip over a body thrown at them.

The physics is [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.6, the one MafiaModLoader pins.

## Download

For a clean installation, use the bundle with MafiaModLoader. If the game already
has a compatible loader, use the smaller mod-only archive. Extract either ZIP once
into the folder containing Game.exe. [Download the latest release](https://github.com/PashOK7/mafia-ragdoll-mml/releases/latest).

## What it does

- **Death.** Whoever dies near the player (a person of a mission, a traffic pedestrian, the player) becomes a ragdoll
  from the pose he had a tick before the game reacted, moving as his animation moved, pushed by what killed him: a
  bullet where it struck, a car at its speed, an explosion. The game's death animation, while it plays, is the
  muscles' target for a moment; then the body goes limp.
- **Explosions.** A grenade, a car blowing up or a mission's explosion throws every body within its reach away from
  it, each part by its own distance. Tommy can also ragdoll when the game gives him a heavy blast fall.
- **Knock-downs.** The game signals a vehicle hit on a mission character or the player. The mod also checks traffic
  pedestrians against moving cars when the native sweep misses. After a hard hit the victim falls and gets
  up with the game's own get-up, from the back or the front, blended from the pose his body was left in. A body
  thrown at someone knocks him over the same way. Nobody alive in a car's seat is knocked down: the game keeps him
  there.
- **Bodies in the world.** The street, the buildings, the kerbs and the cars around are in the physics (the loader
  gives the scene's meshes and the cars' models, including changes to door positions); a body never falls through a street that is not loaded yet (it
  waits for it). Bullets that hit a body lying on the ground push it and are taken from the game's own test, so a
  shot body moves where it was struck. Cars driven over or into bodies are slowed by them.
- **In cars.** Someone killed in his seat slumps there and stays in the cabin while the car drives, turns, brakes or
  crashes; a car teleported by a mission takes him along.
- **The ragdoll camera.** While the player himself is a ragdoll and while he gets up, the camera circles his body:
  the mouse turns it round him, the wheel brings it nearer or further, and it keeps out of walls and cars; it blends
  back into the game's camera once he is up.
- **The dodge fix.** The original game's pedestrians who jump out of a car's way were hit where they had stood, not
  where they landed; they are tested where they are.

## Install

For a clean game, extract **with-MafiaModLoader** into the folder containing Game.exe
and start Mafia. This installs both the ragdoll and MML 2026.11.18 (host 71d2377a).

If a compatible loader is already installed, extract **mod-only** into the same game
folder. This updates only `mml/mods/ragdoll/` and preserves the installed loader.
The mod is on by default. Its settings are in **Options > Mods > Ragdolls**.
Other mods can be installed alongside it in `mml/mods/`.

Getting up on a tram or elevated train uses the walking floor of the carriage under Tommy's feet.
In the first-person passenger view his model is hidden. If he falls through a doorway, the game
returns him to world coordinates. A fall over open air keeps its downward speed.

## Settings (Options > Mods > Ragdolls)

| Tab | Setting | What |
|---|---|---|
| Bodies | Ragdolls | off releases living actors and stops the physics world. On resumes tracking and adopts nearby dead bodies |
| | Cars knock down | cars knock living people over as ragdolls (the player too) |
| | Cars feel bodies | cars are slowed by the bodies they push |
| | Bullets push bodies | bullets strike the bodies on the ground |
| | Blasts knock Tommy down | a native heavy blast fall uses ragdoll physics for the player |
| | Watched within | how far from the player people's poses are kept (a death starts from them) |
| | Bodies drawn | the bodies' capsules drawn over the world (to see what the physics sees) |
| Forces | Bullet's push, Whole body share, Spread of a blow, Fastest part | how hard a bullet pushes, how much of it moves the whole body, how far around the hit it spreads, the fastest it makes one part |
| | Strongest shot | the most momentum one hit gives a body (a script's huge damage stays a shot) |
| | Last shots kept | a living man's last bullets given to his body when he dies within this time |
| | Blast's push | how hard an explosion throws the bodies within its reach |
| Camera | Camera on the body | the ragdoll camera on or off |
| | Mouse turn, Nearest, Furthest, Softness, Back to the game's, Starts further by | how it turns, how near and far the wheel brings it, how softly it follows, how long it blends back, where it starts |

The same keys are in `mod/config.json` (the defaults); what is changed in the game is kept in
`mml\data\ragdoll\config.json`.

## To do

- Add ragdoll physics for dogs.
- Give the tram driver and passengers ragdoll reactions.
- Let Tommy pull a dead driver out of the car when stealing it.

## Cost

About 0.3 ms a frame with six bodies lying near the player and a dozen people around, 0.6 ms with twenty-two
(measured with MafiaModLoader's `profile`); bodies at rest sleep and cost little.

## Building

MafiaModLoader's repository beside this one (or `-DMML_ROOT=<its path>`), Visual Studio 2022 with the x86 tools:

```bat
cmake --preset x86
cmake --build --preset x86-release
```

The mod is in `build\x86\out\mml\mods\ragdoll\`. `python tools\deploy.py` copies it into the labs named in
MafiaModLoader's `mml.config.json` and reloads it in a running game; `python tools\pack.py` makes the players'
downloads in `build/dist/mafia-ragdoll-<version>-player/`. The default builds both.
Use `--mod-only` or `--with-loader` to select one. Agents: read `AGENTS.md`.

## Credits

The ragdoll's physics comes from mafia-ragdoll (its own host before MafiaModLoader); Jolt Physics by Jorrit Rouwe and
its contributors (MIT).
