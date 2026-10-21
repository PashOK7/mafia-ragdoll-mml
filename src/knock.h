// knock: a body (or anything loose in the Jolt world) that strikes a living person hard enough knocks him over, as a
// car does; the person's own body may then knock over the next one: a chain.
//
// Jolt gives the blows of each step (jolt_world::PersonBlows: the momentum a body brought into a person, by its own
// approach only, so walking into a body is no blow). A person's blows within `window` add up (a body strikes with
// several parts over a few steps); when they would move a person of `mass` kg by `speed` m/s or more, he falls. The
// caller knocks him over the game's own way (its knockdown, as from a car, with no damage) and gives the blow to his
// new ragdoll. People seated in cars are the caller's to leave out; Tommy falls as anyone does.
#pragma once
#include <string>
#include <vector>
#include "jolt_world.h"
#include "mathx.h"
#include "tunables.h"

namespace knock {
using LogFn = void (*)(const char *fmt, ...);

struct Knock { void *who; V3 J, at; float dv; };

void Init(LogFn log);
// This step's blows in; the people to knock over now out (one each; not again for a second).
void Tick(float dt, const std::vector<jolt_world::PersonBlow> &blows, std::vector<Knock> &out);
void Forget(const void *who);   // its actor is gone
void Clear();
void Note(const Knock &k, const char *result);   // what the caller did with it (for `knock`)
void Tunes(std::vector<tunables::Info> &out);
bool SetTune(const char *name, float value, std::string *out);
bool Exec(const std::string &cmd, const std::string &args, std::string *out);   // `knock`, `knock set <name> <v>`
}
