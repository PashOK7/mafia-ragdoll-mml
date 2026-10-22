// The ragdoll camera: while the player is a ragdoll and while he gets up, the frame is drawn from a camera that
// circles his body (the mouse turns it round him, the wheel brings it nearer or further), kept out of walls, and
// blends back into the game's camera once he is up. mafia-ragdoll's ragcam on the loader's camera_override (the
// drawing only: the game's camera logic, the aim and the sound stay the game's) and its actions (the mouse's axes
// and wheel read and kept from the game meanwhile: Tommy does not turn, nor cycle his weapons).
#pragma once
#include <string>
#include "mathx.h"

namespace ragcam {

void Settings();                    // ragcam.* from config.json (the game's Options > Mods)
void Follow(const V3 *target);      // each tick: the point to circle (the body's chest), or nullptr: the game's camera
void Stop();                        // the mod's unload, the mission's close: the camera given back at once
std::string State();                // json

} // namespace ragcam
