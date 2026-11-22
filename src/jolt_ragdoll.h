// Ragdolls on Jolt Physics: the Ragdoll class keeps its binding to the game (the skeleton it reads and writes,
// src/ragdoll.*) and its 15 body parts; their motion comes from Jolt (jolt_world::).
//
// - Bodies: the same 15 parts, with an adult's masses (78 kg); boxes for the pelvis, the trunk and the feet (they
//   lie flat instead of rolling), capsules for the limbs, a sphere for the head.
// - Joints: swing-twist joints with anatomical limits around each joint's neutral direction (no twist past what a
//   joint allows, no impossible poses), hinges for the elbows and knees, friction in every joint.
// - Muscles: motors drive every joint towards a target pose with a strength that fades out (a dying body sags
//   before it goes limp). For a death the target is the game's own death animation while the engine plays it, so
//   the body falls the way the animation does but with its real mass, against the real world; for a body knocked
//   over by a car it is the pose at the impact (a brief tensing, then limp).
// - Continuous collision for every part (no tunnelling through a car or a thin wall), sleeping when at rest.
#pragma once
#include <string>
#include <vector>
#include "ragdoll.h"
#include "smoke.h"
#include "tunables.h"

namespace jolt_ragdoll {
using LogFn = void (*)(const char *fmt, ...);

void Init(LogFn log);
// The Jolt bodies for a ragdoll set up by Ragdoll::Init (its start pose and velocities). False: it stays without.
// `seatedIn`: the car it dies sitting in. Its body then lives in the car's cabin (jolt_world kCabin: the car's own
// triangles), held to its seat by the pelvis (a few centimetres and degrees of play), so it slumps against the
// seat, the door and the wheel and is thrown about as the car moves.
bool Add(Ragdoll *r, std::string *err, const void *seatedIn = nullptr, const void *struckBy = nullptr);
// Move a seated corpse with the player's grip during the native extraction track.
void PullFromSeat(Ragdoll *r, const void *holder, const V3 *pose, float dt);
float PullContacts(const Ragdoll *r, V3 (&contacts)[2], const char **part = nullptr);
void FinishPull(Ragdoll *r);
const void *SeatedCar(const Ragdoll *r);
// Preserve a seated body's cabin-relative pose, motion and seat anchor when the engine replaces its carrier.
bool TransferSeat(Ragdoll *r, void *car);
void ForgetCarrier(const void *car);
float GripDistance(const Ragdoll *r, V3 *contact = nullptr); // physical grip error in metres, or -1 when released
// Nearest physical segment of this body on the shot ray, in world coordinates.
int ShotPart(const Ragdoll *r, const V3 &from, const V3 &delta, V3 *point);
bool CabinBlocksShot(const void *key, const V3 &from, const V3 &point, bool requireCabin = true);
void Remove(Ragdoll *r);
bool Has(const Ragdoll *r);
bool WaitingForGround(const Ragdoll *r);
uint32_t CollisionGroupOf(const Ragdoll *r);
bool OwnsBody(uint32_t body);   // a Jolt body (its index and sequence number) is a part of a ragdoll
void Clear();   // every ragdoll's bodies (before the world stops)

// One tick for all of them: muscles and edited velocities into Jolt, the world step (with `movers`, see
// jolt_world::Step), then the bodies' states back into each Ragdoll.
void Step(const std::vector<Ragdoll *> &all, float dt, const std::vector<smoke::Mover> &movers, const V3 *player);

bool Exec(const std::string &cmd, const std::string &args, std::string *out);   // `jolt ...`
std::string Describe();
void Tunes(std::vector<tunables::Info> &out);                       // the live settings (`jolt set`)
bool SetTune(const char *name, float value, std::string *out);
bool Enabled();   // the ragdolls can be made (false only when the settings table is out of order)
}
