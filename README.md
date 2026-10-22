# Ragdolls for Mafia

Physical bodies for **Mafia: The City of Lost Heaven** (2002), builds 1.2 and 1.0, as a
[MafiaModLoader](https://github.com/PashOK7/MafiaModLoader) mod. The dead fall where and how they were struck, the
knocked down tumble and get up the game's way, and the bodies stay in the world: bullets push them, cars feel them,
people trip over a body thrown at them.

The physics is [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.6, the one MafiaModLoader pins.

## What it does

- **Death.** Whoever dies near the player (a person of a mission, a traffic pedestrian, the player) becomes a ragdoll
  from the pose he had a tick before the game reacted, moving as his animation moved, pushed by what killed him: a
  bullet where it struck, a car at its speed, an explosion. The game's death animation, while it plays, is the
  muscles' target for a moment; then the body goes limp.
- **Explosions.** A grenade, a car blowing up or a mission's explosion throws every body within its reach away from
  it, each part by its own distance, so a body turns over as it flies; the living near it are knocked down.
- **Knock-downs.** A car (at about 7 m/s or more) knocks a living man over as a ragdoll; once he lies still he gets
  up with the game's own get-up, from the back or the front, blended from the pose his body was left in. A body
  thrown at someone knocks him over the same way. Nobody alive in a car's seat is knocked down: the game keeps him
  there.
- **Bodies in the world.** The street, the buildings, the kerbs and the cars around are in the physics (the loader
  gives the scene's meshes and the cars' models); a body never falls through a street that is not loaded yet (it
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

1. Install MafiaModLoader (its release archive) into the game's folder.
2. Unpack this mod's archive into the game's folder (it holds `mml\mods\ragdoll\`).
3. Start the game. The mod is on by default; its settings are in the game's **Options > Mods > Ragdolls**.

## Settings (Options > Mods > Ragdolls)

| Tab | Setting | What |
|---|---|---|
| Bodies | Ragdolls | the whole mod on or off |
| | Cars knock down | cars knock living people over as ragdolls (the player too) |
| | Cars feel bodies | cars are slowed by the bodies they push |
| | Bullets push bodies | bullets strike the bodies on the ground |
| | Blasts knock down | an explosion knocks the living near it over as ragdolls |
| | Watched within | how far from the player people's poses are kept (a death starts from them) |
| | Bodies drawn | the bodies' capsules drawn over the world (to see what the physics sees) |
| Forces | Car's push | the share of a car's speed given to a body where it strikes it |
| | Bullet's push, Whole body share, Spread of a blow, Fastest part | how hard a bullet pushes, how much of it moves the whole body, how far around the hit it spreads, the fastest it makes one part |
| | Strongest shot | the most momentum one hit gives a body (a script's huge damage stays a shot) |
| | Last shots kept | a living man's last bullets given to his body when he dies within this time |
| | Blast's push | how hard an explosion throws the bodies within its reach |
| Camera | Camera on the body | the ragdoll camera on or off |
| | Mouse turn, Nearest, Furthest, Softness, Back to the game's, Starts further by | how it turns, how near and far the wheel brings it, how softly it follows, how long it blends back, where it starts |

The same keys are in `mod\ragdoll\config.json` (the defaults); what is changed in the game is kept in
`mml\data\ragdoll\config.json`.

## Cost

About 0.3 ms a frame with six bodies lying near the player and a dozen people around (measured in both builds with
MafiaModLoader's `profile`); bodies at rest sleep and cost almost nothing.

## Building

MafiaModLoader's repository beside this one (or `-DMML_ROOT=<its path>`), Visual Studio 2022 with the x86 tools:

```bat
cmake --preset x86
cmake --build --preset x86-release
```

The mod is in `build\x86\out\mml\mods\ragdoll\`. `python tools\deploy.py` copies it into the labs named in
MafiaModLoader's `mml.config.json` and reloads it in a running game; `python tools\pack.py` makes the players'
archive (`build\dist\mafia-ragdoll-<version>-<date>.zip`). Agents: read `AGENTS.md`.

## Credits

The ragdoll's physics comes from mafia-ragdoll (its own host before MafiaModLoader); Jolt Physics by Jorrit Rouwe and
its contributors (MIT).
