### Changes since v0.1.10

- Tommy pulls a body clear before taking an obstructed seat, including when it lies across both front seats. If the front passenger door is clear but the driver's seat is blocked, the Action menu offers only the passenger seat. If the passenger alone blocks the path to the wheel, Tommy pulls the passenger out before moving across to the driver's seat.
- A body that moves into the doorway during the entry animation can cancel the seat reservation and trigger the pull. Updated door collision follows the moving door.
- Bodies can slide out of open convertibles or settle on a seat after falling in. Dead seated occupants keep their physical pose when a car explosion replaces their model with the charred version.
- Dogs now have articulated ragdolls. Hats can come loose on impact and interact with the world. The update also refines rail collisions, animation handover and vehicle reactions to bodies on the road.

### Installation

Close Mafia and extract one ZIP into the folder containing `Game.exe`. Both downloads support Mafia 1.0 and 1.2.

1. **Ragdoll + MML 0.1** for a clean game. If your modpack already loads ASI plugins from `scripts/` and you are adding MML, extract only `scripts/`, `mml/` and `licenses/` to keep its ASI loader and graphics wrapper.
2. **Ragdoll only** for an existing MafiaModLoader installation. This replaces the files under `mml/mods/ragdoll/` and keeps the installed loader and settings. Compatibility was verified with the loader included in the v0.1.10 ragdoll bundle.

Settings are under **Options > Mods > Ragdolls**.

### Validation and known limit

The clean bundle and the mod-only upgrade each passed 16 car extraction checks on Mafia 1.0 and another 16 on Mafia 1.2. The Falconer replay covered an obstruction already in place and a body arriving during entry. The late-arrival case also passed on both game versions with the older loader. In one Mafia 1.0 Wright Coupe Gangster run, a foot stayed caught in the bodywork after the pull and Tommy remained outside.

### Building from source

Use the SDK from MafiaModLoader commit `d420ee118f5a164fca7aec5231dcdea61983be25` with the three-header SDK patch from this tag. In a checkout of that loader commit beside the mod, apply the patch with `git apply ../mafia-ragdoll-mml/docs/mml-sdk-0.1.59.patch`. Then run the ragdoll project's standard CMake build. [SDK patch](https://github.com/PashOK7/mafia-ragdoll-mml/blob/v0.1.59/docs/mml-sdk-0.1.59.patch)
