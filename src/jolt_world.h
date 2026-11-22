// The mod's rigid-body world: Jolt Physics 5.6 (third_party/jolt) on the game thread. It holds:
//
// - the static world, from the visual models the game draws (scenery::, the same pieces the water and the smoke
//   use), one static body per piece;
// - kinematic bodies for what the game moves: cars as compounds of convex hulls (one per model part, from the
//   cars' own models) and living people as 15 capsules each, moved to their pose every tick, so a body the game
//   moves pushes simulated ones with its real velocity;
// - the dynamic bodies other modules add (the ragdolls: jolt_ragdoll::).
//
// Positions inside Jolt are relative to Origin() (world coordinates in Lost Heaven reach ~2000 m; floats near zero
// keep contacts exact). The world starts on first use and is rebuilt around the player when he has gone far from
// its origin while nothing dynamic exists.
#pragma once
#include <string>
#include <vector>
#include "mathx.h"
#include "smoke.h"

namespace JPH { class PhysicsSystem; class BodyInterface; class BodyID; class Vec3; class Quat; }

namespace jolt_world {
using LogFn = void (*)(const char *fmt, ...);

// Object layers: what collides with what.
// kOccupant: bodies inside a vehicle meet the static world, other occupants, dynamic bodies and their cabin.
// The vehicle's outer convex hull would fill its interior. kCabin: its triangles, both sides, moved with
// it, for the people in it to slump against (seats, doors, the dashboard, the wheel).
// kCarBody: a car the game drives, as a body of its real mass that meets dynamic bodies and living-body sensors.
// kLiving is separate from other kinematics, so car/car and car/tram collisions remain native, without doubling.
enum Layer : unsigned short { kStatic = 0, kKinematic = 1, kDynamic = 2, kOccupant = 3, kCabin = 4, kCarBody = 5, kLiving = 6, kLayerCount = 7 };

bool Start(LogFn log, const V3 &near, std::string *err);   // idempotent
void Stop();               // every body gone; before the core unloads
bool Running();
JPH::PhysicsSystem *System();
JPH::BodyInterface &Bodies();   // no-lock body interface (everything runs on the game thread)
const char *StaticName(const JPH::BodyID &body);

V3 Origin();
inline V3 ToJolt(const V3 &world) { return world - Origin(); }
inline V3 ToWorld(const V3 &local) { return local + Origin(); }

// What the world should hold this tick, then one step of `dt` (split so no collision step exceeds 1/120 s):
// - the scenery service's newest pieces (static bodies made or dropped as they come and go, within a time budget);
// - `movers`: the cars (HULL movers with parts) and people (CAPSULE movers, keyed per body part) the game moves:
//   kinematic bodies moved to these poses over the step; those not given again are removed.
// - `player`: where Tommy is (null: no player): the static world is kept ready around him, and the origin follows
//   him when nothing dynamic exists.
// Dynamic bodies are the callers'; they read their bodies after this.
void Step(float dt, const std::vector<smoke::Mover> &movers, const V3 *player, void (*attachSeats)() = nullptr);

// A body that started inside a car's hull (a convex hull fills the cabin and the open door's gap: a person killed in
// the doorway is born inside it). `SetPassThrough(body, car)`: its contacts with that car's hull bodies are rejected,
// and it meets the car's cabin (RequestCabin) like the people in it; null clears. `HullAround`: the car whose hull
// the body overlaps deeper than `depth` m now (only `onlyCar`'s when given), or null.
void SetPassThrough(const JPH::BodyID &body, const void *car);
// Strongest penetration for each segment touching its own cabin during the last step.
struct PullContact { unsigned part, triangle; float depth; V3 point, normal; };
void WatchDriver(const void *ragdoll, const void *car);
const std::vector<PullContact> &DriverContacts();
unsigned CabinVersion(const void *car);
const void *HullAround(const JPH::BodyID &body, float depth, const void *onlyCar = nullptr);

// The static world in [lo, hi] (world) is all there: the scenery's last scan took the box whole and every piece of it
// is a body (none waiting for the next tick's budget). A body dropped where it is not yet would fall through the
// street (it did after a core reload: the scene walk, the scan and the pieces take a few seconds to come).
bool StaticReady(const V3 &lo, const V3 &hi);
std::string ReadyReport(const V3 &lo, const V3 &hi);   // why StaticReady says what it says (for the log)

// How a living person (his body-part capsules) meets dynamic bodies: not with hard contacts (a kinematic body has
// infinite strength). Segment masses match the ragdoll, capped by `mass` kg. All walker-powered impacts and
// overlap pushes together are at most `maxForce` N per person; incoming dynamic momentum is separate.
// maxForce <= 0: hard contacts.
void SetPersonPush(float maxForce, float mass);
// A body being carried cannot knock its own holder over with the motion the holder gives it.
void SetHolder(const void *ragdoll, const void *person);
void ReleaseHolder(const void *ragdoll); // restore holder contacts after the pair separates
// Aggregate contact diagnostics since world reset; locomotion excludes the body's own incoming reaction.
std::string PersonPushState();
// The blows dynamic bodies dealt living people in the last step: the momentum a body brought into a person (N s,
// world, pointing into him), where, and the body (its id's value). Incoming assembly translation is sampled
// before pushes; work previously supplied by this person is excluded conservatively from the incoming energy.
// Walking into a resting body or its self-induced rebound is no independent blow. Part effective mass is retained.
struct PersonBlow { const void *who; V3 J, at; unsigned body; };
const std::vector<PersonBlow> &PersonBlows();
// Measured car/living-body contacts. Sensors only: native gameplay receives them after the solver returns;
// they never push a car as an infinitely strong animated body. at/velocity are world space, m/s.
struct VehicleContact { const void *who, *car; V3 at, velocity; float closing; };
const std::vector<VehicleContact> &VehicleContacts();


// The game object a kinematic body stands for (a car), or null; whether a car (mover key) has a body now.
const void *KinematicOwner(const JPH::BodyID &id);
bool HasKinematic(const void *key);
// Register a known vehicle before its first impact. Does not advance the world or retire other movers.
bool EnsureVehicle(const smoke::Mover &mover);

// A car someone sits in: its cabin body (layer kCabin) is kept while asked for every tick, from the next Step on.
void RequestCabin(const void *car, const void *occupant = nullptr, bool railway = false);
JPH::BodyID CabinBody(const void *car);   // invalid while there is none
struct RailwayStep {
    V3 cabinPosition{}, cabinVelocity{}, cabinAngular{}, pointVelocity{};
    unsigned contacts{}, endContacts{}, outsideContacts{}, barriersRejected{};
    unsigned outsideBody{}, outsideLayer{};
    const void *outsideVisual{};
    float endApproach{}, endPointZ{}, outsideDepth{};
};
// Carriage dynamics and the last step's contact summary for one rail rider.
bool RailwaySample(const void *car, const void *occupant, V3 at, RailwayStep *step);
// Told before a cabin's body goes (the car left the world, or nobody asks for it): constraints to it must go first.
using CabinGoneFn = void (*)(const void *car);
void SetCabinGoneListener(CabinGoneFn fn);
// Told when a cabin jumped (the car teleported: a script, a respawn) from one pose to another (Jolt's space), after
// its body was put there: whatever sits in it is to be carried the same way.
using CabinJumpFn = void (*)(const void *car, const JPH::Vec3 &fromPos, const JPH::Quat &fromRot, const JPH::Vec3 &toPos,
                             const JPH::Quat &toRot);
void SetCabinJumpListener(CabinJumpFn fn);

// What the bodies did to each car in the last step. A car with a known mass (Mover::mass) is a dynamic body of that
// mass and inertia: every step it is put where the game had it and given the velocity that brings it to where the
// game has it now (no gravity, no damping: the game keeps it on the road); the solver meets it with the bodies both
// ways, by their masses (a wheel against a body on the ground cannot press it into the street and rides up; a body
// struck takes its share of the car's momentum). `dv`, `dw`: how its velocity and its turning (Jolt's: a point moves
// at v + w x r) changed in the step, at Jolt's centre of mass `com` (world): what the game's car must take.
struct CarKick { const void *car; V3 dv, dw, com; };
const std::vector<CarKick> &CarKicks();
// `jolt car <car>`: a car's body here: dynamic or kinematic, mass, its origin's height, and each part of its hull
// with its bounds in the car's frame (m): where the wheels are, how low the body comes down.
std::string DescribeCar(const void *car);
// Which part of a car's hull a collision's sub-shape id (its value) is (its index), or -1.
int CarPartOf(const JPH::BodyID &car, unsigned subShapeId);
// The last step's contacts of driven cars with bodies: the car, the part of its hull, the point (world), the normal
// from the car to the body, the depth. For `carfeel`.
struct CarContactInfo {
    const void *car; int part; V3 at, n; float depth;
    uintptr_t owner; uint32_t body; bool sensor;
    V3 carOrigin, carRight, carForward, velocity;
    unsigned wheel; // native wheel index plus one, or zero for other sub-shapes
};
const std::vector<CarContactInfo> &CarContacts();
unsigned CarContactsDropped();   // diagnostic limit reached in the last step; absence is then inconclusive
// Native wheel-index mask supported by physical bodies below the tyres during the last completed step.
uint32_t WheelSupport(const void *car);

// The scenery's focus for the static world: around the player, and around every box callers add each tick.
void AddFocus(const V3 &lo, const V3 &hi);

// A dynamic body struck something (a new contact): the speed it came in at along the contact normal (m/s), where
// (world), and the normal of what it hit (world, pointing at the body). Called inside Step, on the game thread.
using ImpactFn = void (*)(const JPH::BodyID &body, float speed, const V3 &point, const V3 &normal);
void SetImpactListener(ImpactFn fn);

// Water: the surface under or over a point (world; y the body's: a street over the city's river plane is no water for
// what lies on it): its height, its normal (up), the water's flow (m/s); false where there is no open water. Given
// by the mod (core: the loader's world_water); none: nothing floats.
using WaterFn = bool (*)(float x, float y, float z, float *surfaceY, V3 *normal, V3 *flow);
void SetWaterQuery(WaterFn fn);
bool WaterAt(float x, float y, float z, float *surfaceY, V3 *normal, V3 *flow);
// A dynamic body in water: buoyancy 1 is neutral, below sinks, above floats; Jolt's drag factors (linear ~0.5,
// angular ~0.01). Applied before every step while the body exists.
void SetFloat(const JPH::BodyID &id, float buoyancy, float linearDrag, float angularDrag);
void ClearFloat(const JPH::BodyID &id);
unsigned BodiesInWater();   // in the last step
bool IsCarBody(uint32_t body);   // a Jolt body (its index and sequence number) stands for a car the game moves

// The boxes (world) round every body that moves (the dynamic ones, and the kinematic ones the game moves: cars,
// people) whose middle is within `reach` m of (cx, cz) across: what the water lets the eye see into it.
struct Bounds { V3 lo, hi; };
void MovingBounds(float cx, float cz, float reach, std::vector<Bounds> &out);

std::string Describe();
}
