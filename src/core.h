// The ragdolls: who becomes one and when, their life in the rigid-body world, the bones written, getting up, the
// bullets and blows they take. mafia-ragdoll's core.cpp on the loader's services.
#pragma once
#include <string>
#include "mml/mml.h"

namespace core {

void Start();                         // (the mod's load) the physics modules, the hooks, the settings
void Stop();                          // (its unload) every ragdoll dropped, the world stopped
void Tick(float dt);                  // MISSION_TICK: after the game's world moved
void Render();                        // RENDER_BEGIN: the bones written over the game's animation
void Rendered();                      // RENDER_WORLD: observe the pose after the actual world drawing
void RenderEnd();                     // RENDER_END: return the railway passengers' frames to the game
void OnHit(const MmlEvent &ev);       // HUMAN_HIT, PEDESTRIAN_HIT
void OnActorDestroyed(void *actor);
void OnExplosion(const MmlExplosion &x);   // EXPLOSION: the blast throws the bodies near it
void OnCarExploded(void *car);              // the native wreck and charred occupant models are ready
void OnWorldReset(const char *reason); // mission close or a save load that can reuse the mission
void Settings();                      // the settings read again (config.json, the game's Options > Mods)
std::string State();                  // the `state` command's reply (json)
std::string Exec(const std::string &cmd, const std::string &args, bool *ok);   // the developer's commands

} // namespace core
