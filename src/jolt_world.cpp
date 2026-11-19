// The mod's rigid-body world on Jolt Physics (see jolt_world.h).
#include "fmt.h"
#include "jolt_world.h"
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <windows.h>
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include "scenery.h"

namespace jolt_world {
namespace {

LogFn g_log;
#define JLOG(...) (g_log ? g_log(__VA_ARGS__) : (void)0)


double Now() {
    LARGE_INTEGER f, t;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&t);
    return 1000.0 * (double)t.QuadPart / (double)f.QuadPart;
}

// ---- Jolt's plumbing
void Trace(const char *fmt, ...) {
    char b[1024]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    JLOG("jolt: %s", b);
}

namespace BP { constexpr JPH::BroadPhaseLayer kStill(0), kMoving(1); constexpr unsigned kCount = 2; }

class BroadPhaseLayers final : public JPH::BroadPhaseLayerInterface {
public:
    unsigned GetNumBroadPhaseLayers() const override { return BP::kCount; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer l) const override { return l == kStatic ? BP::kStill : BP::kMoving; }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer l) const override { return l == BP::kStill ? "still" : "moving"; }
#endif
};

// Dynamic bodies meet everything; static and kinematic ones only meet dynamic ones (the game moves them).
bool Meets(JPH::ObjectLayer a, JPH::ObjectLayer b) {
    auto either = [&](JPH::ObjectLayer x) { return a == x || b == x; };
    if (either(kCarBody)) return a == kDynamic || b == kDynamic || a == kLiving || b == kLiving;
    if (either(kOccupant)) return (a == kOccupant || a == kDynamic || a == kCabin) && (b == kOccupant || b == kDynamic || b == kCabin);
    // A cabin is for the people in it (kOccupant) and for bodies passing through its car's hull (a person killed in
    // the open door, SetPassThrough); it exists only while someone asks for it.
    if (either(kCabin)) return a == kDynamic || b == kDynamic;
    return a == kDynamic || b == kDynamic;
}

class LayerPairs final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override { return Meets(a, b); }
};
class LayerVsBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::BroadPhaseLayer b) const override {
        if (a == kOccupant) return b == BP::kMoving;   // people in a car never meet the street
        return a == kDynamic || b == BP::kMoving;   // a static or kinematic body only looks for moving ones
    }
};

std::unordered_map<uint32_t, const void *> g_carOfBody;   // kinematic body id -> the car it stands for
std::unordered_map<uint32_t, const void *> g_passCar;     // dynamic body id -> the car whose hull it passes through
// Living people: their body-part capsules are kinematic (the game's animation moves them), so a hard contact would
// give them infinite strength: they bulldozed bodies as if weightless and forced pinned limbs past their joints.
// Their contacts with dynamic bodies are sensors instead; each tick the touched bodies get the impulse that stops
// them closing in on the person and lets them out of the overlap, at most `g_personForce` N per person.
std::unordered_map<const void *, const void *> g_holder;
struct PersonPart { const void *who; float mass; };
std::unordered_map<uint32_t, PersonPart> g_personOfBody;   // capsule -> its person and anatomical mass
struct PersonTouch { JPH::BodyID person, body; const void *who; JPH::Vec3 n; JPH::RVec3 point; float depth; };
std::unordered_map<uint64_t, PersonTouch> g_personTouches;   // (capsule, body) -> the latest contact of the last step
float g_personForce = 700.0f;                                 // <= 0: hard contacts (infinite strength)
float g_personMass = 20.0f;                                   // upper bound on a segment's impact mass (kg)
std::vector<PersonBlow> g_personBlows;                        // the last step's (PersonBlows)
struct PushStats {
    unsigned contacts = 0;
    double requested = 0, delivered = 0, reaction = 0;
    float peakForce = 0, maxPartMass = 0;
} g_pushStats;
// Cars and bodies: see CarKicks (jolt_world.h). A car the game drives is a dynamic body of its own mass here, so the
// solver itself settles every contact with the bodies both ways; what it did to the car goes back to the game.
std::vector<CarKick> g_carKicks;
std::vector<VehicleContact> g_vehicleContacts;
V3 F(JPH::Vec3Arg v);
std::vector<CarContactInfo> g_carContacts;   // this step's (cleared before each step)
unsigned g_carContactsDropped;
// A driven car's wheels: the parts of its hull made from the model's wheel frames (WHL0, WHR1...), a bit per part, by
// the car's body. Their hull does not turn, but a wheel rolls: its contacts with bodies get the rolling tyre's surface
// velocity and rubber's friction (WheelContact), so a wheel climbs onto what it meets as a tyre climbs a kerb.
std::unordered_map<uint32_t, uint64_t> g_wheelParts;
std::unordered_map<const void *, uint32_t> g_wheelSupport; // supporting wheel contacts from the completed step
int CarPartOfSub(const JPH::Body &car, const JPH::SubShapeID &sub) {
    const JPH::Shape *shape = car.GetShape();
    if (!shape || shape->GetSubType() != JPH::EShapeSubType::StaticCompound) return -1;
    JPH::SubShapeID rest;
    return (int)static_cast<const JPH::StaticCompoundShape *>(shape)->GetSubShapeIndexFromID(sub, rest);
}

const void *g_watchDriver = nullptr, *g_watchCabin = nullptr;
std::vector<PullContact> g_driverContacts;
ImpactFn g_impact;
V3 OriginNow();
void NoteCarReaction(const void *car);

class Contacts final : public JPH::ContactListener {
public:
    void OnContactAdded(const JPH::Body &a, const JPH::Body &b, const JPH::ContactManifold &m, JPH::ContactSettings &s) override {
        PersonContact(a, b, m, s);
        WheelContact(a, b, m, s);
        DrivenCarContact(a, b, m, s);
        SlidingCabinContact(a, b, s);
        TraceDriver(a, b, m);
        if (!g_impact || m.mRelativeContactPointsOn1.empty()) return;
        JPH::RVec3 p = m.GetWorldSpaceContactPointOn1(0);
        JPH::Vec3 n = m.mWorldSpaceNormal;   // from a towards b
        float approach = (a.GetPointVelocity(p) - b.GetPointVelocity(p)).Dot(n);   // > 0: closing
        if (approach <= 0) return;
        V3 wp = V3{p.GetX(), p.GetY(), p.GetZ()} + OriginNow();
        V3 nv{n.GetX(), n.GetY(), n.GetZ()};
        if (a.IsDynamic()) g_impact(a.GetID(), approach, wp, -nv);
        if (b.IsDynamic()) g_impact(b.GetID(), approach, wp, nv);
    }
    void OnContactPersisted(const JPH::Body &a, const JPH::Body &b, const JPH::ContactManifold &m, JPH::ContactSettings &s) override {
        PersonContact(a, b, m, s);
        WheelContact(a, b, m, s);
        DrivenCarContact(a, b, m, s);
        SlidingCabinContact(a, b, s);
        TraceDriver(a, b, m);
    }
    // A body passing through a car's hull (SetPassThrough) does not meet that car's hull bodies; it meets the car's
    // cabin, which is not one of them.
    JPH::ValidateResult OnContactValidate(const JPH::Body &a, const JPH::Body &b, JPH::RVec3Arg, const JPH::CollideShapeResult &) override {
        if (!g_passCar.empty() && (Passes(a, b) || Passes(b, a))) return JPH::ValidateResult::RejectAllContactsForThisBodyPair;
        return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
    }
private:
    // A driven car's wheel touching a body: the tyre rolls. A wheel rolling forward at v turns about up x forward at
    // v / R, so its surface where the contact's normal n (from the car, radial on a tyre) leaves it moves, against
    // the car, at v (up x forward) x n: down at its front. The body's friction on it is the other way: up, and the
    // wheel climbs. Jolt takes it as the bodies' relative surface velocity (body 2's minus body 1's).
    static void WheelContact(const JPH::Body &a, const JPH::Body &b, const JPH::ContactManifold &m, JPH::ContactSettings &s) {
        bool carA = a.GetObjectLayer() == kCarBody, carB = b.GetObjectLayer() == kCarBody;
        if (carA == carB) return;
        const JPH::Body &car = carA ? a : b;
        auto wheels = g_wheelParts.find(car.GetID().GetIndexAndSequenceNumber());
        if (wheels == g_wheelParts.end()) return;
        int part = CarPartOfSub(car, carA ? m.mSubShapeID1 : m.mSubShapeID2);
        if (part < 0 || part >= 64 || !(wheels->second >> part & 1)) return;
        JPH::Quat q = car.GetRotation();
        JPH::Vec3 up = q * JPH::Vec3::sAxisY(), fwd = q * JPH::Vec3::sAxisZ();
        float v = car.GetLinearVelocity().Dot(fwd);
        JPH::Vec3 n = carA ? m.mWorldSpaceNormal : -m.mWorldSpaceNormal;   // from the car to the body
        JPH::Vec3 tyre = up.Cross(fwd).Cross(n) * v;                        // the tyre's surface, against the car
        s.mRelativeLinearSurfaceVelocity = carA ? -tyre : tyre;
        s.mCombinedFriction = 0.8f;   // rubber on cloth

    }
    // A driven car (kCarBody) touching a body: noted for CarContacts.
    static void DrivenCarContact(const JPH::Body &a, const JPH::Body &b, const JPH::ContactManifold &m, const JPH::ContactSettings &s) {
        bool carA = a.GetObjectLayer() == kCarBody, carB = b.GetObjectLayer() == kCarBody;
        if (carA == carB) return;
        const JPH::Body &car = carA ? a : b;
        auto owner = g_carOfBody.find(car.GetID().GetIndexAndSequenceNumber());
        if (owner == g_carOfBody.end()) return;
        const JPH::Body &other = carA ? b : a;
        if (!s.mIsSensor && other.GetObjectLayer() == kDynamic) NoteCarReaction(owner->second);
        JPH::Vec3 n = carA ? m.mWorldSpaceNormal : -m.mWorldSpaceNormal;   // from the car to the body
        // Support is independent of diagnostic contact limits and compound part ordering.
        if (!s.mIsSensor && other.GetObjectLayer() == kDynamic && n.GetY() < -0.5f) {
            const auto sub = carA ? m.mSubShapeID1 : m.mSubShapeID2;
            const uint64_t wheel = car.GetShape()->GetSubShapeUserData(sub);
            if (wheel > 0 && wheel <= 32) g_wheelSupport[owner->second] |= 1u << (wheel - 1);
        }
        if (g_carContacts.size() >= 64) { ++g_carContactsDropped; return; }
        JPH::RVec3 p = m.mRelativeContactPointsOn1.empty() ? m.mBaseOffset : m.GetWorldSpaceContactPointOn1(0);
        int part = CarPartOfSub(car, carA ? m.mSubShapeID1 : m.mSubShapeID2);
        const JPH::RVec3 origin = car.GetPosition();
        const JPH::Quat rotation = car.GetRotation();
        g_carContacts.push_back(CarContactInfo{owner->second, part, V3{(float)p.GetX(), (float)p.GetY(), (float)p.GetZ()} + OriginNow(),
                                               V3{n.GetX(), n.GetY(), n.GetZ()}, m.mPenetrationDepth,
                                               (uintptr_t)other.GetUserData(), other.GetID().GetIndexAndSequenceNumber(), s.mIsSensor,
                                               V3{(float)origin.GetX(), (float)origin.GetY(), (float)origin.GetZ()} + OriginNow(),
                                               F(rotation * JPH::Vec3::sAxisX()), F(rotation * JPH::Vec3::sAxisZ()), F(other.GetLinearVelocity())});
    }
    // A living person's capsule touching a dynamic body: no hard contact, remembered for ApplyPersonPushes.
    static void PersonContact(const JPH::Body &a, const JPH::Body &b, const JPH::ContactManifold &m, JPH::ContactSettings &s) {
        if (g_personOfBody.empty()) return;
        auto pa = g_personOfBody.find(a.GetID().GetIndexAndSequenceNumber());
        auto pb = g_personOfBody.find(b.GetID().GetIndexAndSequenceNumber());
        const JPH::Body *person, *body;
        const void *who;
        JPH::Vec3 n = m.mWorldSpaceNormal;   // from a towards b
        if (pa != g_personOfBody.end() && b.IsDynamic()) { person = &a; body = &b; who = pa->second.who; }
        else if (pb != g_personOfBody.end() && a.IsDynamic()) { person = &b; body = &a; who = pb->second.who; n = -n; }
        else return;
        JPH::RVec3 p = m.mRelativeContactPointsOn2.empty() ? m.mBaseOffset
                       : body == &b ? m.GetWorldSpaceContactPointOn2(0) : m.GetWorldSpaceContactPointOn1(0);
        auto car = g_carOfBody.find(body->GetID().GetIndexAndSequenceNumber());
        if (car != g_carOfBody.end()) {
            s.mIsSensor = true;
            JPH::Vec3 carVelocity = body->GetPointVelocity(p);
            JPH::Vec3 relative = carVelocity - person->GetPointVelocity(p);
            // Only the car's own approach creates a vehicle blow. An animated hand sweeping into a parked
            // car can have a high relative speed; that must not turn the stationary car into an attacker.
            float closing = -carVelocity.Dot(n);
            if (closing <= 1.5f || carVelocity.LengthSq() <= 1.5f * 1.5f) return;
            auto old = std::find_if(g_vehicleContacts.begin(), g_vehicleContacts.end(), [&](const VehicleContact &c) {
                return c.who == who && c.car == car->second;
            });
            VehicleContact contact{who, car->second, F(JPH::Vec3(p)), F(relative), closing};
            if (old != g_vehicleContacts.end()) { if (closing > old->closing) *old = contact; }
            else if (g_vehicleContacts.size() < 256) g_vehicleContacts.push_back(contact);
            return;
        }
        if (!g_holder.empty()) {
            auto held = g_holder.find(reinterpret_cast<const void *>(static_cast<uintptr_t>(body->GetUserData())));
            if (held != g_holder.end() && held->second == who) {
                s.mIsSensor = true;
                return;
            }
        }
        if (g_personForce <= 0) return;
        s.mIsSensor = true;
        uint64_t key = (uint64_t)person->GetID().GetIndexAndSequenceNumber() << 32 | body->GetID().GetIndexAndSequenceNumber();
        g_personTouches[key] = PersonTouch{person->GetID(), body->GetID(), who, n, p, m.mPenetrationDepth};
    }
    // A coat and trousers sliding over the seat and sill during a carjack.
    static void SlidingCabinContact(const JPH::Body &a, const JPH::Body &b, JPH::ContactSettings &s) {
        if (g_holder.empty() || (a.GetObjectLayer() != kCabin && b.GetObjectLayer() != kCabin)) return;
        const bool cabinA = a.GetObjectLayer() == kCabin;
        const auto &body = cabinA ? b : a;
        if (!g_holder.count(reinterpret_cast<const void *>(static_cast<uintptr_t>(body.GetUserData())))) return;
        const auto pass = g_passCar.find(body.GetID().GetIndexAndSequenceNumber());
        if (pass != g_passCar.end() && CabinBody(pass->second) == (cabinA ? a : b).GetID())
            s.mCombinedFriction = .3f;
    }
    static void TraceDriver(const JPH::Body &a, const JPH::Body &b, const JPH::ContactManifold &m) {
        if (!g_watchDriver || m.mRelativeContactPointsOn1.empty()) return;
        const bool cabinA = a.GetObjectLayer() == kCabin;
        if (!cabinA && b.GetObjectLayer() != kCabin) return;
        const auto &body = cabinA ? b : a;
        const auto &cabin = cabinA ? a : b;
        if (body.GetUserData() != reinterpret_cast<uintptr_t>(g_watchDriver) || cabin.GetID() != CabinBody(g_watchCabin)) return;
        const unsigned part = body.GetCollisionGroup().GetSubGroupID();
        auto it = std::find_if(g_driverContacts.begin(), g_driverContacts.end(), [part](const auto &c){ return c.part == part; });
        if (it != g_driverContacts.end() && it->depth >= m.mPenetrationDepth) return;
        const auto *mesh = static_cast<const JPH::MeshShape *>(cabin.GetShape());
        const auto idx = mesh->GetTriangleUserData(cabinA ? m.mSubShapeID1 : m.mSubShapeID2);
        PullContact contact{part, idx, m.mPenetrationDepth,
            F(JPH::Vec3(cabinA ? m.GetWorldSpaceContactPointOn1(0) : m.GetWorldSpaceContactPointOn2(0))) + OriginNow(),
            F(cabinA ? m.mWorldSpaceNormal : -m.mWorldSpaceNormal)};
        if (it == g_driverContacts.end()) g_driverContacts.push_back(contact);
        else *it = contact;
    }
    static bool Passes(const JPH::Body &body, const JPH::Body &other) {
        auto it = g_passCar.find(body.GetID().GetIndexAndSequenceNumber());
        if (it == g_passCar.end()) return false;
        auto car = g_carOfBody.find(other.GetID().GetIndexAndSequenceNumber());
        return car != g_carOfBody.end() && car->second == it->second;
    }
};

struct World {
    BroadPhaseLayers bp;
    LayerPairs pairs;
    LayerVsBroadPhase vsBp;
    Contacts contacts;
    std::unique_ptr<JPH::TempAllocatorImpl> temp;
    std::unique_ptr<JPH::JobSystemSingleThreaded> jobs;
    std::unique_ptr<JPH::PhysicsSystem> system;
    V3 origin{};
};
// Explicitly owned by Start/Stop. During process detach the ragdoll entries in another translation unit still
// need this world for their destructors. Windows reclaims the world at process exit, regardless of CRT order.
World *g_w;
bool g_typesRegistered;
// The water bodies float in (SetWaterQuery) and the bodies that float (SetFloat).
WaterFn g_water;
struct Float { float buoyancy, linearDrag, angularDrag; bool wet = false; };
std::unordered_map<uint32_t, Float> g_floats;   // by body id
unsigned g_inWater;

JPH::Vec3 J(const V3 &v) { return JPH::Vec3(v.x, v.y, v.z); }
V3 F(JPH::Vec3Arg v) { return V3{v.GetX(), v.GetY(), v.GetZ()}; }

// Rows ax[i]: the world directions of the local axes (unit, orthogonal, right-handed).
JPH::Quat FromAxes(const V3 ax[3]) {
    JPH::Mat44 m(JPH::Vec4(ax[0].x, ax[0].y, ax[0].z, 0), JPH::Vec4(ax[1].x, ax[1].y, ax[1].z, 0),
                 JPH::Vec4(ax[2].x, ax[2].y, ax[2].z, 0), JPH::Vec4(0, 0, 0, 1));
    return m.GetQuaternion().Normalized();
}

// The rotation that turns +y onto `d` (a capsule's axis).
JPH::Quat YTo(const V3 &d) {
    JPH::Vec3 to = J(Norm(d, {0, 1, 0}));
    return JPH::Quat::sFromTo(JPH::Vec3::sAxisY(), to);
}

// ---- the static world: one body per scenery piece, one mesh shape per piece shape
struct Static { JPH::BodyID body; unsigned version = 0, used = 0; };
std::unordered_map<const void *, Static> g_statics;               // by piece key
std::unordered_map<const void *, JPH::RefConst<JPH::Shape>> g_meshes;   // by shape key
unsigned g_sceneryScans = ~0u, g_sceneryEpoch = ~0u, g_applyRound;
unsigned g_staticsPending, g_meshFailures;
// The last scan whose pieces are all bodies here (its number; ~0u: none since the statics were last made anew).
unsigned g_appliedScan = ~0u;
bool g_freshWorld = true;
double g_sceneryMs;
// The box the ragdolls asked for (the last focus given to scenery): the scenery has pieces for other consumers too
// (the water's shore lines want 500 m of quays), and only those in this box become bodies.
V3 g_keepLo{-1e9f, -1e9f, -1e9f}, g_keepHi{1e9f, 1e9f, 1e9f};
struct PieceBound { V3 centre; float radius; };
std::unordered_map<const void *, PieceBound> g_pieceBounds;   // by shape key, in the piece's frame

bool InKeep(const smoke::SceneryPiece &p) {
    auto it = g_pieceBounds.find(p.shapeKey);
    if (it == g_pieceBounds.end()) {
        V3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f};
        if (p.pos) for (const V3 &v : *p.pos) {
            lo = {std::min(lo.x, v.x), std::min(lo.y, v.y), std::min(lo.z, v.z)};
            hi = {std::max(hi.x, v.x), std::max(hi.y, v.y), std::max(hi.z, v.z)};
        }
        PieceBound b{(lo + hi) * 0.5f, p.pos && !p.pos->empty() ? Len(hi - lo) * 0.5f : 0.0f};
        it = g_pieceBounds.emplace(p.shapeKey, b).first;
    }
    const PieceBound &b = it->second;
    V3 c = p.t + p.ax[0] * b.centre.x + p.ax[1] * b.centre.y + p.ax[2] * b.centre.z;
    float r = b.radius + 10.0f;
    return c.x + r >= g_keepLo.x && c.x - r <= g_keepHi.x && c.y + r >= g_keepLo.y && c.y - r <= g_keepHi.y &&
           c.z + r >= g_keepLo.z && c.z - r <= g_keepHi.z;
}

JPH::RefConst<JPH::Shape> MeshOf(const smoke::SceneryPiece &p) {
    auto it = g_meshes.find(p.shapeKey);
    if (it != g_meshes.end()) return it->second;
    JPH::RefConst<JPH::Shape> shape;
    if (p.pos && p.tris && p.tris->size() >= 3) {
        JPH::VertexList verts;
        verts.reserve(p.pos->size());
        for (const V3 &v : *p.pos) verts.push_back(JPH::Float3(v.x, v.y, v.z));
        JPH::IndexedTriangleList tris;
        tris.reserve(p.tris->size() / 3);
        for (size_t t = 0; t + 2 < p.tris->size(); t += 3) tris.push_back(JPH::IndexedTriangle((*p.tris)[t], (*p.tris)[t + 1], (*p.tris)[t + 2], 0));
        JPH::MeshShapeSettings ms(std::move(verts), std::move(tris));   // drops the degenerate triangles
        // Nothing left (a piece of slivers, or only water, which scenery leaves out): no body, and nothing wrong.
        if (!ms.mIndexedTriangles.empty()) {
            JPH::ShapeSettings::ShapeResult r = ms.Create();
            if (r.IsValid()) shape = r.Get();
            else if (g_meshFailures++ < 10) JLOG("jolt: a scenery mesh was not made (%s)", r.GetError().c_str());
        }
    }
    g_meshes[p.shapeKey] = shape;   // a failure is remembered too: not tried again every scan
    return shape;
}

void ApplyScenery() {
    JPH::BodyInterface &bi = Bodies();
    if (scenery::Epoch() != g_sceneryEpoch) {   // the mission's world is gone: so are its pieces and meshes
        for (auto &kv : g_statics) { bi.RemoveBody(kv.second.body); bi.DestroyBody(kv.second.body); }
        g_statics.clear();
        g_pieceBounds.clear();
        g_meshes.clear();
        g_sceneryEpoch = scenery::Epoch();
        g_sceneryScans = ~0u;
        g_appliedScan = ~0u;
    }
    if (scenery::Scans() == g_sceneryScans && !g_staticsPending) return;
    g_sceneryScans = scenery::Scans();
    double t0 = Now();
    // All at once the first time (a body may be lying there already); a few milliseconds per tick after that.
    double budget = g_freshWorld || scenery::FirstScan() ? 400.0 : 3.0;
    unsigned round = ++g_applyRound, made = 0;
    g_staticsPending = 0;
    for (const smoke::SceneryPiece &p : scenery::Pieces()) {
        if (!InKeep(p)) continue;   // another consumer's: left out (removed below if it was a body)
        auto it = g_statics.find(p.key);
        if (it != g_statics.end() && it->second.version == p.version) { it->second.used = round; continue; }
        if (Now() - t0 > budget) { g_staticsPending++; if (it != g_statics.end()) it->second.used = round; continue; }
        JPH::RefConst<JPH::Shape> shape = MeshOf(p);
        if (it != g_statics.end()) { bi.RemoveBody(it->second.body); bi.DestroyBody(it->second.body); g_statics.erase(it); }
        if (!shape) continue;
        JPH::BodyCreationSettings s(shape, J(p.t - g_w->origin), FromAxes(p.ax), JPH::EMotionType::Static, kStatic);
        s.mFriction = 0.8f;   // clothes on asphalt, stone, wood
        s.mRestitution = 0.0f;
        JPH::BodyID id = bi.CreateAndAddBody(s, JPH::EActivation::DontActivate);
        if (id.IsInvalid()) continue;
        g_statics[p.key] = Static{id, p.version, round};
        made++;
    }
    for (auto it = g_statics.begin(); it != g_statics.end();) {
        if (it->second.used == round) { ++it; continue; }
        bi.RemoveBody(it->second.body);
        bi.DestroyBody(it->second.body);
        it = g_statics.erase(it);
    }
    if (!g_staticsPending) g_appliedScan = g_sceneryScans;
    // The meshes of shapes scenery no longer has (it lets go of what the region left behind) go too; a body keeps its
    // own reference while it stands. (Every 40 rounds: 10 s of scenery scans.)
    if (round % 40 == 0) {
        std::unordered_set<const void *> live;
        for (const smoke::SceneryPiece &p : scenery::Pieces()) live.insert(p.shapeKey);
        for (auto it = g_meshes.begin(); it != g_meshes.end();) it = live.count(it->first) ? std::next(it) : g_meshes.erase(it);
        for (auto it = g_pieceBounds.begin(); it != g_pieceBounds.end();) it = live.count(it->first) ? std::next(it) : g_pieceBounds.erase(it);
    }
    if (made > 16) g_w->system->OptimizeBroadPhase();
    g_sceneryMs = Now() - t0;
    if (g_freshWorld && !scenery::Pieces().empty()) {
        g_freshWorld = false;
        JLOG("jolt: static world of %u pieces (%u meshes) in %.1f ms", (unsigned)g_statics.size(), (unsigned)g_meshes.size(), g_sceneryMs);
    }
}

// ---- kinematic bodies: what the game moves
// A body the game moves. `dynamic`: a car with its mass (kCarBody), driven by velocity; vSet/wSet: what it was given
// for the step (CollectCarKicks compares).
struct Kinematic {
    JPH::BodyID body; unsigned version = 0, seen = 0; V3 pos{};
    unsigned reactionTick = 0;
    JPH::Quat rot = JPH::Quat::sIdentity();
    bool dynamic = false;
    bool still = false;   // a kinematic part left where it is (ApplyKinematics): its velocity zeroed once
    JPH::Vec3 vSet = JPH::Vec3::sZero(), wSet = JPH::Vec3::sZero();
};
std::unordered_map<const void *, Kinematic> g_kinematic;   // by mover key
unsigned g_tick;

void NoteCarReaction(const void *car) {
    auto it = g_kinematic.find(car);
    if (it != g_kinematic.end()) it->second.reactionTick = g_tick;
}

// A mover's triangles (`hull`, three points each), both windings: seen from inside too. A car's cabin for the people
// in it, and the whole of a vehicle with no body of its own mass (Mover::hullAsMesh: a tram, hollow).
JPH::RefConst<JPH::Shape> TwoSidedMesh(const smoke::Mover &m);

using HullCache = std::unordered_map<uint64_t, JPH::RefConst<JPH::Shape>>;
std::unordered_map<const void *, HullCache> g_cookedHulls;

// `wheels`: which of the compound's sub-shapes are the mover's wheel parts (a part whose hull fails is not one).
JPH::RefConst<JPH::Shape> CarShape(const smoke::Mover &m, uint64_t *wheels = nullptr) {
    if (wheels) *wheels = 0;
    if (m.kind == smoke::Mover::HULL && m.hullAsMesh && m.mass <= 0) return TwoSidedMesh(m);
    if (m.kind == smoke::Mover::BOX) {
        V3 h{std::max(m.h.x, 0.05f), std::max(m.h.y, 0.05f), std::max(m.h.z, 0.05f)};
        return new JPH::BoxShape(J(h), 0.02f);
    }
    if ((!m.hullParts || m.hullParts->empty()) && m.hull && !m.hullIsTriangles && m.hull->size() >= 4) {
        // Moving scenery (a door, a prop): one convex hull of its points.
        JPH::Array<JPH::Vec3> pts;
        for (const V3 &p : *m.hull) pts.push_back(J(p));
        JPH::ConvexHullShapeSettings hs(pts, 0.01f);
        JPH::ShapeSettings::ShapeResult r = hs.Create();
        return r.IsValid() ? r.Get() : nullptr;
    }
    if (!m.hullParts || m.hullParts->empty()) return nullptr;
    HullCache next;
    HullCache &previous = g_cookedHulls[m.key];
    JPH::StaticCompoundShapeSettings cs;
    for (size_t i = 0; i < m.hullParts->size(); i++) {
        const std::vector<V3> &part = (*m.hullParts)[i];
        const uint64_t revision = m.hullRevisions ? (*m.hullRevisions)[i] : 0;
        JPH::RefConst<JPH::Shape> hull;
        auto cached = previous.find(revision);
        if (revision && cached != previous.end()) hull = cached->second;
        else {
            JPH::Array<JPH::Vec3> pts;
            pts.reserve(part.size());
            for (const V3 &p : part) pts.push_back(J(p));
            JPH::ConvexHullShapeSettings hs(pts, 0.01f);
            if (m.nativeWheels && i < m.nativeWheels->size()) hs.mUserData = (*m.nativeWheels)[i];
            auto result = hs.Create();
            if (result.IsValid()) hull = result.Get();
        }
        if (revision) next.emplace(revision, hull);
        if (!hull) continue;
        if (wheels && i < 64 && (m.wheelParts >> i & 1) && cs.mSubShapes.size() < 64) *wheels |= 1ull << cs.mSubShapes.size();
        cs.AddShape(JPH::Vec3::sZero(), JPH::Quat::sIdentity(), hull);
    }
    previous.swap(next);
    if (cs.mSubShapes.empty()) return nullptr;
    JPH::ShapeSettings::ShapeResult r = cs.Create();
    return r.IsValid() ? r.Get() : nullptr;
}

// A pose for a mover: position (Jolt space) and rotation, and the shape it needs (null: keep the one it has).
bool PoseOf(const smoke::Mover &m, V3 *pos, JPH::Quat *rot, unsigned *version) {
    if (m.kind == smoke::Mover::CAPSULE) {
        V3 d = m.p1 - m.p0;
        *pos = (m.p0 + m.p1) * 0.5f - g_w->origin;
        *rot = YTo(d);
        // The shape changes only if the limb does (a key reused by another human): its size, to the millimetre.
        *version = (unsigned)std::lround(Len(d) * 1000.0f) * 1000u + (unsigned)std::lround(m.r * 1000.0f);
        return true;
    }
    if (m.kind == smoke::Mover::HULL || m.kind == smoke::Mover::BOX) {
        *pos = m.c - g_w->origin;
        *rot = FromAxes(m.ax);
        *version = m.kind == smoke::Mover::HULL ? m.shapeVersion * 2 + (m.hullAsMesh ? 1 : 0) : 1;
        return true;
    }
    return false;
}

Kinematic *KinematicFor(const smoke::Mover &m, const V3 &pos, const JPH::Quat &rot, unsigned version, bool *created) {
    JPH::BodyInterface &bi = Bodies();
    *created = false;
    bool car = m.kind != smoke::Mover::CAPSULE;
    auto it = g_kinematic.find(m.key);
    if (car && it != g_kinematic.end() && it->second.version != version && it->second.dynamic == (m.mass > 0)) {
        uint64_t wheels = 0;
        auto shape = CarShape(m, &wheels);
        if (!shape) return nullptr;
        Kinematic &k = it->second;
        bi.SetShape(k.body, shape, false, JPH::EActivation::Activate);
        if (k.dynamic) {
            // SetShape's default density would overwrite the game's vehicle mass.
            auto properties = shape->GetMassProperties();
            properties.ScaleToMass(m.mass);
            JPH::BodyLockWrite lock(g_w->system->GetBodyLockInterfaceNoLock(), k.body);
            auto *motion = lock.GetBody().GetMotionProperties();
            motion->SetMassProperties(motion->GetAllowedDOFs(), properties);
            g_wheelParts[k.body.GetIndexAndSequenceNumber()] = wheels;
        }
        k.version = version;
    }
    if (it != g_kinematic.end() && it->second.version != version) {
        g_carOfBody.erase(it->second.body.GetIndexAndSequenceNumber());
        g_personOfBody.erase(it->second.body.GetIndexAndSequenceNumber());
        g_wheelParts.erase(it->second.body.GetIndexAndSequenceNumber());
        bi.RemoveBody(it->second.body);
        bi.DestroyBody(it->second.body);
        g_kinematic.erase(it);
        it = g_kinematic.end();
    }
    if (it != g_kinematic.end()) return &it->second;
    JPH::RefConst<JPH::Shape> shape;
    uint64_t wheels = 0;
    if (car) shape = CarShape(m, &wheels);
    else shape = new JPH::CapsuleShape(std::max(0.5f * Len(m.p1 - m.p0), 0.01f), std::max(m.r, 0.01f));
    if (!shape) return nullptr;
    bool dyn = car && m.mass > 0;
    JPH::BodyCreationSettings s(shape, J(pos), rot, dyn ? JPH::EMotionType::Dynamic : JPH::EMotionType::Kinematic,
                                car ? (dyn ? kCarBody : kKinematic) : kLiving);
    s.mFriction = car ? 0.5f : 0.7f;
    s.mRestitution = 0.0f;
    s.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    s.mMassPropertiesOverride.mMass = dyn ? m.mass : car ? 1200.0f : 5.0f;
    if (dyn) {
        s.mGravityFactor = 0.0f;   // the game holds it up
        s.mLinearDamping = 0.0f;
        s.mAngularDamping = 0.0f;
        s.mAllowSleeping = false;
        s.mMotionQuality = JPH::EMotionQuality::LinearCast;
    }
    JPH::BodyID id = bi.CreateAndAddBody(s, JPH::EActivation::Activate);
    if (id.IsInvalid()) return nullptr;
    if (car) g_carOfBody[id.GetIndexAndSequenceNumber()] = m.key;
    else g_personOfBody[id.GetIndexAndSequenceNumber()] = {m.owner ? m.owner : m.key, m.partMass};
    if (dyn && wheels) g_wheelParts[id.GetIndexAndSequenceNumber()] = wheels;
    Kinematic nk{id, version, g_tick, pos};
    nk.rot = rot;
    nk.dynamic = dyn;
    *created = true;
    return &g_kinematic.emplace(m.key, nk).first->second;
}

void ApplyKinematics(float dt, const std::vector<smoke::Mover> &movers) {
    JPH::BodyInterface &bi = Bodies();
    g_tick++;
    for (const smoke::Mover &m : movers) {
        V3 pos; JPH::Quat rot; unsigned version;
        if (!m.key || !PoseOf(m, &pos, &rot, &version)) continue;
        bool created;
        Kinematic *entry = KinematicFor(m, pos, rot, version, &created);
        if (!entry || created) continue;
        Kinematic &k = *entry;
        // A jump the game made (a teleport, a respawn) is not a motion: no velocity from it.
        bool jump = dt <= 0 || Len(pos - k.pos) > 30.0f * dt + 2.0f;
        if (k.dynamic) {
            // Where the game had it, and the velocity (of its centre of mass) and turning that bring it to where the
            // game has it now over the step.
            JPH::Vec3 v = JPH::Vec3::sZero(), w = JPH::Vec3::sZero();
            if (jump) {
                bi.SetPositionAndRotation(k.body, J(pos), rot, JPH::EActivation::Activate);
            } else {
                bi.SetPositionAndRotation(k.body, J(k.pos), k.rot, JPH::EActivation::Activate);
                JPH::Quat dq = (rot * k.rot.Conjugated()).Normalized();
                if (dq.GetW() < 0) dq = -dq;
                JPH::Vec3 axis;
                float angle;
                dq.GetAxisAngle(axis, angle);
                if (angle > 1e-6f) w = axis * (angle / dt);
                JPH::Vec3 comOffset = k.rot * bi.GetShape(k.body)->GetCenterOfMass();
                v = J(pos - k.pos) * (1.0f / dt) + w.Cross(comOffset);
            }
            bi.SetLinearAndAngularVelocity(k.body, v, w);
            // Jolt clamps the input here. Compare the solver result to what it actually received.
            bi.GetLinearAndAngularVelocity(k.body, k.vSet, k.wSet);
        } else if (jump) {
            bi.SetPositionAndRotation(k.body, J(pos), rot, JPH::EActivation::Activate);
            k.still = false;
        } else if (Len(pos - k.pos) < 0.01f && std::fabs(rot.Dot(k.rot)) > 0.99996f) {
            // Within a centimetre and a degree of where the part is (a person standing: his idle sway is
            // millimetres): left there. A part moved every step stays awake, and so does every body it touches: the
            // corpses at a standing man's feet never slept. Its velocity is zeroed once (a kinematic body keeps
            // moving at the last one) and it sleeps; it is moved again once the pose has gone further.
            if (!k.still) bi.SetLinearAndAngularVelocity(k.body, JPH::Vec3::sZero(), JPH::Vec3::sZero());
            k.still = true;
            k.seen = g_tick;
            continue;   // (k.pos, k.rot: where the body is)
        } else {
            bi.MoveKinematic(k.body, J(pos), rot, dt);
            k.still = false;
        }
        k.pos = pos;
        k.rot = rot;
        k.seen = g_tick;
    }
    for (auto it = g_kinematic.begin(); it != g_kinematic.end();) {
        if (it->second.seen == g_tick) { ++it; continue; }
        g_cookedHulls.erase(it->first);
        g_carOfBody.erase(it->second.body.GetIndexAndSequenceNumber());
        g_personOfBody.erase(it->second.body.GetIndexAndSequenceNumber());
        g_wheelParts.erase(it->second.body.GetIndexAndSequenceNumber());
        bi.RemoveBody(it->second.body);
        bi.DestroyBody(it->second.body);
        it = g_kinematic.erase(it);
    }
}

// ---- cabins: a car's triangles, both windings (TwoSidedMesh), for the people in it
std::unordered_map<const void *, Kinematic> g_cabins;
std::unordered_set<const void *> g_cabinWanted;
CabinGoneFn g_cabinGone;
CabinJumpFn g_cabinJump;

void DropCabin(std::unordered_map<const void *, Kinematic>::iterator it) {
    if (g_cabinGone) g_cabinGone(it->first);
    JPH::BodyInterface &bi = Bodies();
    bi.RemoveBody(it->second.body);
    bi.DestroyBody(it->second.body);
    g_cabins.erase(it);
}

JPH::RefConst<JPH::Shape> TwoSidedMesh(const smoke::Mover &m) {
    if (!m.hull || !m.hullIsTriangles || m.hull->size() < 3) return nullptr;
    JPH::TriangleList tris;
    tris.reserve(m.hull->size() / 3 * 2);
    for (size_t t = 0; t + 2 < m.hull->size(); t += 3) {
        JPH::Float3 a((*m.hull)[t].x, (*m.hull)[t].y, (*m.hull)[t].z), b((*m.hull)[t + 1].x, (*m.hull)[t + 1].y, (*m.hull)[t + 1].z),
            c((*m.hull)[t + 2].x, (*m.hull)[t + 2].y, (*m.hull)[t + 2].z);
        const auto triangle = static_cast<unsigned>(t / 3);
        tris.emplace_back(a, b, c, 0, triangle);
        tris.emplace_back(a, c, b, 0, triangle);   // seen from inside too
    }
    JPH::MeshShapeSettings ms(tris);
    ms.mPerTriangleUserData = true;
    JPH::ShapeSettings::ShapeResult r = ms.Create();
    return r.IsValid() ? r.Get() : nullptr;
}

void ApplyCabins(float dt, const std::vector<smoke::Mover> &movers) {
    JPH::BodyInterface &bi = Bodies();
    for (const smoke::Mover &m : movers) {
        if (m.kind != smoke::Mover::HULL || !g_cabinWanted.count(m.key)) continue;
        V3 pos; JPH::Quat rot; unsigned version;
        if (!PoseOf(m, &pos, &rot, &version)) continue;
        version = m.shapeVersion; // Cabin geometry is always a mesh. Keep its source triangle revision.
        auto it = g_cabins.find(m.key);
        if (it != g_cabins.end() && it->second.version != version) {
            // Keep the seat constraint attached to the same cabin body as a door opens.
            if (auto shape = TwoSidedMesh(m)) {
                bi.SetShape(it->second.body, shape, false, JPH::EActivation::Activate);
                it->second.version = version;
            }
        }
        if (it == g_cabins.end()) {
            JPH::RefConst<JPH::Shape> shape = TwoSidedMesh(m);
            if (!shape) continue;
            JPH::BodyCreationSettings s(shape, J(pos), rot, JPH::EMotionType::Kinematic, kCabin);
            // Seats and roof liners are triangulated. Slide across their internal
            // edges without generating an extra contact normal at each seam.
            s.mEnhancedInternalEdgeRemoval = true;
            s.mFriction = 0.8f;   // cloth on upholstery and leather
            s.mRestitution = 0.0f;
            s.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
            s.mMassPropertiesOverride.mMass = 1200.0f;
            s.mMassPropertiesOverride.mInertia = JPH::Mat44::sScale(1000.0f);
            JPH::BodyID id = bi.CreateAndAddBody(s, JPH::EActivation::Activate);
            if (id.IsInvalid()) continue;
            g_cabins[m.key] = Kinematic{id, version, g_tick, pos};
            continue;
        }
        Kinematic &k = it->second;
        if (dt <= 0 || Len(pos - k.pos) > 30.0f * dt + 2.0f) {
            JPH::RVec3 fromPos;
            JPH::Quat fromRot;
            bi.GetPositionAndRotation(k.body, fromPos, fromRot);
            bi.SetPositionAndRotation(k.body, J(pos), rot, JPH::EActivation::Activate);
            JLOG("jolt: car %p's cabin jumped %.2f m (dt %.3f): its seated carried with it", m.key, Len(pos - k.pos), dt);
            if (g_cabinJump) g_cabinJump(m.key, fromPos, fromRot, JPH::Vec3(J(pos)), rot);
        } else {
            bi.MoveKinematic(k.body, J(pos), rot, dt);
        }
        k.pos = pos;
        k.seen = g_tick;
    }
    for (auto it = g_cabins.begin(); it != g_cabins.end();) {
        auto next = std::next(it);
        if (it->second.seen != g_tick || !g_cabinWanted.count(it->first)) DropCabin(it);
        it = next;
    }
    g_cabinWanted.clear();
}

struct IncomingMotion { JPH::Vec3 linear, angular; JPH::RVec3 centre; JPH::Vec3 assembly; uint64_t group; };
std::unordered_map<uint32_t,IncomingMotion> g_incoming;
// Work supplied by a kinematic walker is not energy an independently flying body brought into that walker.
// Carry this attribution through joint redistribution/rebound; dissipate it in proportion to measured kinetic
// energy loss. A later external impulse increases measured energy without increasing the walker's contribution.
struct AssemblyAccount {
    float energy=0, incomingEnergy=0;
    std::unordered_map<const void *,float> work, incomingWork;
};
std::unordered_map<uint64_t,AssemblyAccount> g_accounts;

void RememberVelocities() {
    g_incoming.clear();
    JPH::BodyIDVector bodies;
    g_w->system->GetBodies(bodies);
    struct Assembly { JPH::Vec3 momentum=JPH::Vec3::sZero(); float mass=0, energy=0; };
    std::unordered_map<uint64_t,Assembly> assemblies;
    const JPH::BodyLockInterfaceNoLock &locks = g_w->system->GetBodyLockInterfaceNoLock();
    for (const JPH::BodyID &id : bodies) {
        JPH::BodyLockRead lock(locks, id);
        if (!lock.Succeeded()) continue;
        const JPH::Body &b = lock.GetBody();
        if (!b.IsDynamic()) continue;
        float im = b.GetMotionProperties()->GetInverseMass();
        float mass=im>0 ? 1.0f/im : 0;
        // Jolt assigns the same owner to every part of a ragdoll. Other loose bodies form their own assembly.
        uint64_t group=b.GetUserData() ? (b.GetUserData()<<32)|b.GetCollisionGroup().GetGroupID()
                                       : (uint64_t(1)<<63)|id.GetIndexAndSequenceNumber();
        auto &assembly=assemblies[group];
        assembly.momentum+=b.GetLinearVelocity()*mass; assembly.mass+=mass;
        const auto *properties=b.GetMotionProperties();
        JPH::Vec3 omega=(b.GetRotation()*properties->GetInertiaRotation()).Conjugated()*b.GetAngularVelocity();
        JPH::Vec3 inverse=properties->GetInverseInertiaDiagonal();
        float rotational=0;
        for (int k=0;k<3;k++) if (inverse[k]>1e-8f) rotational+=omega[k]*omega[k]/inverse[k];
        assembly.energy+=.5f*(mass*b.GetLinearVelocity().LengthSq()+rotational);
        g_incoming.emplace(id.GetIndexAndSequenceNumber(),IncomingMotion{b.GetLinearVelocity(),b.GetAngularVelocity(),
                           b.GetCenterOfMassPosition(),JPH::Vec3::sZero(),group});
    }
    for (auto &kv:g_incoming) {
        const auto &a=assemblies.at(kv.second.group);
        kv.second.assembly=a.mass>0 ? a.momentum/a.mass : JPH::Vec3::sZero();
    }
    for (auto it=g_accounts.begin();it!=g_accounts.end();) {
        if (!assemblies.count(it->first)) it=g_accounts.erase(it); else ++it;
    }
    for (const auto &kv:assemblies) {
        auto &account=g_accounts[kv.first];
        float retained=account.energy>1e-6f ? std::min(1.f,kv.second.energy/account.energy) : 0;
        for (auto it=account.work.begin();it!=account.work.end();) {
            it->second*=retained;
            if (it->second<1e-4f) it=account.work.erase(it); else ++it;
        }
        account.energy=account.incomingEnergy=kv.second.energy;
        account.incomingWork=account.work;
    }
}

// What the step did to each driven car (see CarKicks): its velocity and turning now against what it was given.
void CollectCarKicks() {
    JPH::BodyInterface &bi = Bodies();
    for (auto &kv : g_kinematic) {
        const Kinematic &k = kv.second;
        if (!k.dynamic || k.reactionTick != g_tick || !bi.IsAdded(k.body)) continue;
        JPH::Vec3 dv = bi.GetLinearVelocity(k.body) - k.vSet, dw = bi.GetAngularVelocity(k.body) - k.wSet;
        if (dv.LengthSq() < 1e-6f && dw.LengthSq() < 1e-6f) continue;
        JPH::RVec3 c = bi.GetCenterOfMassPosition(k.body);
        g_carKicks.push_back(CarKick{kv.first, V3{dv.GetX(), dv.GetY(), dv.GetZ()}, V3{dw.GetX(), dw.GetY(), dw.GetZ()},
                                     V3{(float)c.GetX(), (float)c.GetY(), (float)c.GetZ()} + g_w->origin});
    }
}

// ---- focus and re-centring
V3 g_focusLo{1e9f, 1e9f, 1e9f}, g_focusHi{-1e9f, -1e9f, -1e9f};
bool g_haveFocus;
const V3 kAroundPlayer{30, 10, 30};   // m around Tommy that is always ready (a car may hit someone there)

void SetSceneryFocus(const V3 *player) {
    if (player) AddFocus(*player - kAroundPlayer, *player + kAroundPlayer + V3{0, 5, 0});
    if (g_haveFocus) {
        scenery::SetFocus(scenery::kRagdoll, g_focusLo, g_focusHi);
        g_keepLo = g_focusLo;
        g_keepHi = g_focusHi;
    } else {
        scenery::ClearFocus(scenery::kRagdoll);
    }
    g_focusLo = {1e9f, 1e9f, 1e9f};
    g_focusHi = {-1e9f, -1e9f, -1e9f};
    g_haveFocus = false;
}

unsigned g_steps;
double g_stepMs;
V3 OriginNow() { return g_w ? g_w->origin : V3{}; }

void RemoveAll() {
    while (!g_cabins.empty()) DropCabin(g_cabins.begin());
    JPH::BodyInterface &bi = Bodies();
    for (auto &kv : g_statics) { bi.RemoveBody(kv.second.body); bi.DestroyBody(kv.second.body); }
    for (auto &kv : g_kinematic) { bi.RemoveBody(kv.second.body); bi.DestroyBody(kv.second.body); }
    g_statics.clear();
    g_kinematic.clear();
    g_cookedHulls.clear();
    g_carOfBody.clear();
    WatchDriver(nullptr, nullptr);
    g_passCar.clear();
    g_holder.clear();
    g_personOfBody.clear();
    g_personTouches.clear();
    g_pushStats = {};
    g_incoming.clear();
    g_accounts.clear();
    g_carKicks.clear();
    g_wheelParts.clear();
    g_wheelSupport.clear();
    g_vehicleContacts.clear();
    g_meshes.clear();
}

V3 RoundOrigin(const V3 &p) {
    return V3{std::round(p.x / 64.0f) * 64.0f, std::round(p.y / 64.0f) * 64.0f, std::round(p.z / 64.0f) * 64.0f};
}

// Tommy far from the origin and nothing dynamic in the world: everything is made again around him.
void Recentre(const V3 *player) {
    if (!player) return;
    V3 p = *player;
    if (Len(p - g_w->origin) < 500.0f) return;
    unsigned dynamic = g_w->system->GetNumBodies() - (unsigned)g_statics.size() - (unsigned)g_kinematic.size();
    if (dynamic) return;
    RemoveAll();
    g_w->origin = RoundOrigin(p);
    g_freshWorld = true;
    g_sceneryScans = ~0u;
    g_appliedScan = ~0u;
    JLOG("jolt: origin moved to %.0f %.0f %.0f", g_w->origin.x, g_w->origin.y, g_w->origin.z);
}

} // namespace

bool Start(LogFn log, const V3 &around, std::string *err) {
    if (g_w) return true;
    g_log = log;
    if (!g_typesRegistered) {
        JPH::RegisterDefaultAllocator();
        JPH::Trace = Trace;
        if (!JPH::VerifyJoltVersionID()) { *err = "Jolt headers and library differ (rebuild third_party/jolt)"; return false; }
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
        g_typesRegistered = true;
    }
    auto w = std::make_unique<World>();
    w->temp = std::make_unique<JPH::TempAllocatorImpl>(16 * 1024 * 1024);
    w->jobs = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);
    w->system = std::make_unique<JPH::PhysicsSystem>();
    w->system->Init(16384, 0, 16384, 8192, w->bp, w->vsBp, w->pairs);
    JPH::PhysicsSettings ps = w->system->GetPhysicsSettings();
    ps.mNumVelocitySteps = 12;   // jointed bodies: more iterations than the default 10
    ps.mNumPositionSteps = 4;
    w->system->SetPhysicsSettings(ps);
    w->system->SetGravity(JPH::Vec3(0, -9.81f, 0));
    w->system->SetContactListener(&w->contacts);
    // A round origin near the player: coordinates inside Jolt stay within a few hundred metres.
    w->origin = RoundOrigin(around);
    g_w = w.release();
    g_freshWorld = true;
    g_sceneryScans = g_sceneryEpoch = ~0u;
    g_appliedScan = ~0u;
    JLOG("jolt: world started (Jolt %d.%d.%d), origin %.0f %.0f %.0f", JPH_VERSION_MAJOR, JPH_VERSION_MINOR, JPH_VERSION_PATCH,
         g_w->origin.x, g_w->origin.y, g_w->origin.z);
    return true;
}

void Stop() {
    g_floats.clear();
    g_inWater = 0;
    if (g_w) {
        RemoveAll();
        delete g_w;
        g_w = nullptr;
    }
    scenery::ClearFocus(scenery::kRagdoll);
    if (g_typesRegistered) {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
        g_typesRegistered = false;
    }
}

bool Running() { return g_w != nullptr; }
JPH::PhysicsSystem *System() { return g_w ? g_w->system.get() : nullptr; }
JPH::BodyInterface &Bodies() { return g_w->system->GetBodyInterfaceNoLock(); }
V3 Origin() { return g_w ? g_w->origin : V3{}; }

void AddFocus(const V3 &lo, const V3 &hi) {
    g_focusLo = {std::min(g_focusLo.x, lo.x), std::min(g_focusLo.y, lo.y), std::min(g_focusLo.z, lo.z)};
    g_focusHi = {std::max(g_focusHi.x, hi.x), std::max(g_focusHi.y, hi.y), std::max(g_focusHi.z, hi.z)};
    g_haveFocus = true;
}

// ---- water
// Buoyancy and drag for every floating body with water under it: the surface over its centre of mass, tilted with
// the waves (Jolt cuts the body's shape with that plane for the submerged volume).
bool IsCarBody(uint32_t body) { return g_carOfBody.count(body) != 0; }

void ApplyWater(float dt) {
    g_inWater = 0;
    if (!g_water || g_floats.empty()) return;
    JPH::BodyInterface &bi = Bodies();
    JPH::Vec3 gravity = g_w->system->GetGravity();
    for (auto it = g_floats.begin(); it != g_floats.end();) {
        JPH::BodyID id(it->first);
        if (!bi.IsAdded(id)) { it = g_floats.erase(it); continue; }
        if (!it->second.wet && !bi.IsActive(id)) { ++it; continue; }   // (asleep on dry land: nothing to ask)
        V3 c = ToWorld(F(bi.GetCenterOfMassPosition(id)));
        float y;
        V3 n, flow;
        Float &f = it->second;
        bool wet = false;
        if (g_water(c.x, c.y, c.z, &y, &n, &flow)) {
            JPH::AABox box = bi.GetTransformedShape(id).GetWorldSpaceBounds();
            float lo = box.mMin.GetY() + g_w->origin.y;
            if (lo < y) {
                wet = true;
                if (bi.ApplyBuoyancyImpulse(id, J(V3{c.x, y, c.z} - g_w->origin), J(n), f.buoyancy, f.linearDrag, f.angularDrag, J(flow), gravity, dt))
                    g_inWater++;
            }
        }
        f.wet = wet;
        ++it;
    }
}

// Living animation is kinematic: it cannot lose momentum to a corpse. Resolve its finite segment mass locally,
// and share ONE locomotion impulse allowance across every contact of that person, impacts included. Previously
// each touching part supplied a fresh 20 kg impact every frame outside the force budget: an FPS-dependent motor.
// A dynamic body's own incoming momentum is a separate, dissipative reaction, not walker-supplied energy.
void ApplyPersonPushes(float dt) {
    g_personBlows.clear();
    if (g_personTouches.empty()) return;
    JPH::BodyInterface &bi = Bodies();
    const JPH::BodyLockInterfaceNoLock &locks = g_w->system->GetBodyLockInterfaceNoLock();
    std::unordered_map<const void *, float> left;   // N s each person may still give this tick
    for (auto &kv : g_personTouches) {
        const PersonTouch &t = kv.second;
        if (!bi.IsAdded(t.body) || !bi.IsAdded(t.person)) continue;
        float invM;
        JPH::Vec3 r, inr;
        {
            JPH::BodyLockRead lock(locks, t.body);
            if (!lock.Succeeded() || !lock.GetBody().IsDynamic()) continue;
            const JPH::Body &body = lock.GetBody();
            invM = body.GetMotionProperties()->GetInverseMass();
            r = JPH::Vec3(t.point - body.GetCenterOfMassPosition());
            inr = body.GetInverseInertia().Multiply3x3(r.Cross(t.n)).Cross(r);
        }
        float w = invM + t.n.Dot(inr);
        if (w <= 1e-6f) continue;
        auto incoming=g_incoming.find(t.body.GetIndexAndSequenceNumber());
        if (incoming==g_incoming.end()) continue;
        const IncomingMotion &motion=incoming->second;
        JPH::Vec3 bodyV=motion.linear+motion.angular.Cross(JPH::Vec3(t.point-motion.centre));
        float currentNormal = bi.GetPointVelocity(t.body,t.point).Dot(t.n);
        float closing = bi.GetPointVelocity(t.person, t.point).Dot(t.n) - currentNormal;
        float depth = std::min(std::max(t.depth, 0.0f), 0.1f);
        auto part = g_personOfBody.find(t.person.GetIndexAndSequenceNumber());
        if (part == g_personOfBody.end()) continue;
        float mass = std::min(part->second.mass, g_personMass);
        float inverseMass = mass > 0 ? w + 1.f / mass : 0;
        float impact = inverseMass > 0 ? std::max(closing, 0.f) / inverseMass : 0.f;
        // the blow into the person: the same impact with the body's own approach only (t.n points at the body), from a
        // body that moves as a thrown one does (1.5 m/s and more where it touches): a body lying in a walker's way is
        // shoved slowly by his own feet and bounces back at him (seen 2026-10-22: two of five people walking past
        // corpses fell over them, 487 such blows in four minutes)
        // A limb spinning after a footstep is not an incoming thrown body. Attribute a blow to the assembly's
        // incoming translation, sampled before any person push; contact iteration must not manufacture attacks.
        auto &account=g_accounts.at(motion.group);
        auto supplied=account.incomingWork.find(t.who);
        float ownWork=supplied==account.incomingWork.end() ? 0 : supplied->second;
        float independent=account.incomingEnergy>1e-6f ? std::sqrt(std::max(0.f,1.f-ownWork/account.incomingEnergy)) : 0;
        JPH::Vec3 incomingVelocity=motion.assembly*independent;
        float approach = std::min(-bodyV.Dot(t.n),-incomingVelocity.Dot(t.n));
        if (inverseMass > 0 && approach > 0 && incomingVelocity.LengthSq() > 1.5f * 1.5f && g_personBlows.size() < 256) {
            float Jin = approach / inverseMass;
            g_personBlows.push_back(PersonBlow{t.who, F(-t.n * Jin), F(JPH::Vec3(t.point)) + g_w->origin,
                                               t.body.GetIndexAndSequenceNumber()});
        }
        auto budget = left.try_emplace(t.who, g_personForce * dt).first;
        float reaction = inverseMass > 0 ? std::min(impact, std::max(-currentNormal, 0.f) / inverseMass) : 0.f;
        float requested = impact - reaction + 0.2f * depth / dt / w;
        float locomotion = std::min(requested, std::max(budget->second, 0.f));
        budget->second -= locomotion;
        g_pushStats.contacts++;
        g_pushStats.requested += requested;
        g_pushStats.delivered += locomotion;
        g_pushStats.reaction += reaction;
        g_pushStats.maxPartMass = std::max(g_pushStats.maxPartMass, mass);
        g_pushStats.peakForce = std::max(g_pushStats.peakForce, (g_personForce * dt - budget->second) / dt);
        float J = reaction + locomotion;
        if (J <= 0) continue;
        // Exact kinetic work of this impulse, including rotation at the contact point (w is inverse effective mass).
        float work=J*bi.GetPointVelocity(t.body,t.point).Dot(t.n)+.5f*J*J*w;
        float after=std::max(0.f,account.energy+work);
        if (work>0) account.work[t.who]+=work;
        else if (account.energy>1e-6f) for (auto &entry:account.work) entry.second*=after/account.energy;
        account.energy=after;
        bi.AddImpulse(t.body, t.n * J, t.point);
    }
    g_personTouches.clear();
}

void WatchDriver(const void *ragdoll, const void *car) {
    g_watchDriver = ragdoll;
    g_watchCabin = car;
    g_driverContacts.clear();
}
const std::vector<PullContact> &DriverContacts() { return g_driverContacts; }

void SetHolder(const void *ragdoll, const void *person) {
    if (person) g_holder[ragdoll] = person;
    else g_holder.erase(ragdoll);
}

void SetPersonPush(float maxForce, float mass) { g_personForce = maxForce; g_personMass = mass; }

std::string PersonPushState() {
    return Fmt("{\"contacts\":%u,\"requested_ns\":%.4f,\"delivered_ns\":%.4f,\"reaction_ns\":%.4f,"
               "\"peak_force_n\":%.3f,\"force_limit_n\":%.3f,\"max_segment_kg\":%.3f}",
               g_pushStats.contacts, g_pushStats.requested, g_pushStats.delivered, g_pushStats.reaction,
               g_pushStats.peakForce, g_personForce, g_pushStats.maxPartMass);
}

void Step(float dt, const std::vector<smoke::Mover> &movers, const V3 *player) {
    g_driverContacts.clear();
    g_carContacts.clear();
    g_carContactsDropped = 0;
    g_carKicks.clear();       // a paused or failed step has no reaction to deliver
    g_wheelSupport.clear();
    g_vehicleContacts.clear();
    g_personBlows.clear();   // (a paused tick has none)
    if (!g_w) return;
    Recentre(player);
    SetSceneryFocus(player);
    ApplyScenery();
    ApplyKinematics(dt, movers);
    ApplyCabins(dt, movers);
    if (dt <= 0) return;   // the game is paused: the bodies wait
    // Integrate the same mission interval used for mover velocities and recovery timers.
    // The host bounds mission ticks below 200 ms. Keep each collision substep at most 1/120 s.
    int collisionSteps = std::max(1, (int)std::ceil(dt * 120.0f - 1e-3f));
    double t0 = Now();
    RememberVelocities();
    ApplyPersonPushes(dt);
    ApplyWater(dt);
    JPH::EPhysicsUpdateError e = g_w->system->Update(dt, collisionSteps, g_w->temp.get(), g_w->jobs.get());
    for (VehicleContact &c : g_vehicleContacts) c.at += g_w->origin;
    CollectCarKicks();
    g_stepMs = g_stepMs * 0.9 + (Now() - t0) * 0.1;
    g_steps++;
    if (e != JPH::EPhysicsUpdateError::None) {
        static unsigned told;
        if (told++ < 5) JLOG("jolt: step error %u (contact or body pair buffers full)", (unsigned)e);
    }
}

// The ground around a box is all bodies here: the scans know every visual around it (scenery::Covers), and the
// statics have been made from a scan at least as new as the complete scan that took it. (It used to ask for the
// last scan to be complete and made into bodies at this very moment: while Tommy walked or a scan ran that was
// often false for a moment, and a new body was held in the air for up to 5 s over ground that was there.)
bool StaticReady(const V3 &lo, const V3 &hi) {
    return Running() && g_sceneryEpoch == scenery::Epoch() && g_appliedScan != ~0u && g_appliedScan >= scenery::FullScan() &&
           scenery::Covers(lo, hi);
}

std::string ReadyReport(const V3 &lo, const V3 &hi) {
    return Fmt("statics from scan %d (%u pieces still to make), epoch %s; ", g_appliedScan == ~0u ? -1 : (int)g_appliedScan,
               g_staticsPending, g_sceneryEpoch == scenery::Epoch() ? "current" : "old") + scenery::CoverReport(lo, hi);
}

void SetPassThrough(const JPH::BodyID &body, const void *car) {
    if (car) g_passCar[body.GetIndexAndSequenceNumber()] = car;
    else g_passCar.erase(body.GetIndexAndSequenceNumber());
}

const void *HullAround(const JPH::BodyID &body, float depth, const void *onlyCar) {
    if (!Running()) return nullptr;
    JPH::BodyInterface &bi = Bodies();
    JPH::RefConst<JPH::Shape> shape = bi.GetShape(body);
    if (!shape) return nullptr;
    JPH::CollideShapeSettings cs;
    cs.mMaxSeparationDistance = 0.0f;
    JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> hits;
    JPH::IgnoreSingleBodyFilter self(body);
    System()->GetNarrowPhaseQueryNoLock().CollideShape(shape, JPH::Vec3::sReplicate(1.0f), bi.GetCenterOfMassTransform(body), cs,
                                                       JPH::RVec3::sZero(), hits, {}, {}, self);
    const void *best = nullptr;
    float deepest = depth;
    for (const JPH::CollideShapeResult &h : hits.mHits) {
        auto car = g_carOfBody.find(h.mBodyID2.GetIndexAndSequenceNumber());
        if (car == g_carOfBody.end() || (onlyCar && car->second != onlyCar) || h.mPenetrationDepth <= deepest) continue;
        deepest = h.mPenetrationDepth;
        best = car->second;
    }
    return best;
}

std::string DescribeCar(const void *car) {
    auto it = g_kinematic.find(car);
    if (it == g_kinematic.end() || !Running()) return Fmt("jolt car: %p has no body", car);
    JPH::BodyInterface &bi = Bodies();
    const Kinematic &k = it->second;
    JPH::RefConst<JPH::Shape> shape = bi.GetShape(k.body);
    JPH::RVec3 p = bi.GetPosition(k.body);
    float invM = 0;
    {
        JPH::BodyLockRead lock(g_w->system->GetBodyLockInterfaceNoLock(), k.body);
        if (lock.Succeeded() && lock.GetBody().IsDynamic()) invM = lock.GetBody().GetMotionProperties()->GetInverseMass();
    }
    std::string s = Fmt("jolt car %p: %s, mass %.0f kg, origin at height %.2f (world)", car, k.dynamic ? "dynamic" : "kinematic",
                        invM > 0 ? 1.0f / invM : 0.0f, (float)p.GetY() + g_w->origin.y);
    JPH::Vec3 com = shape->GetCenterOfMass();
    if (shape->GetSubType() != JPH::EShapeSubType::StaticCompound) {
        JPH::AABox b = shape->GetLocalBounds();
        return s + Fmt("\n  one shape: x %.2f..%.2f y %.2f..%.2f z %.2f..%.2f", b.mMin.GetX() + com.GetX(), b.mMax.GetX() + com.GetX(),
                       b.mMin.GetY() + com.GetY(), b.mMax.GetY() + com.GetY(), b.mMin.GetZ() + com.GetZ(), b.mMax.GetZ() + com.GetZ());
    }
    auto *cs = static_cast<const JPH::StaticCompoundShape *>(shape.GetPtr());
    for (JPH::uint i = 0; i < cs->GetNumSubShapes(); i++) {
        const JPH::CompoundShape::SubShape &sub = cs->GetSubShape(i);
        JPH::AABox b = sub.mShape->GetLocalBounds().Transformed(sub.GetLocalTransformNoScale(JPH::Vec3::sReplicate(1.0f)));
        auto wheels = g_wheelParts.find(k.body.GetIndexAndSequenceNumber());
        bool wheel = wheels != g_wheelParts.end() && i < 64 && (wheels->second >> i & 1);
        s += Fmt("\n  part %2u: x %5.2f..%5.2f  y %5.2f..%5.2f  z %5.2f..%5.2f%s", i, b.mMin.GetX() + com.GetX(), b.mMax.GetX() + com.GetX(),
                 b.mMin.GetY() + com.GetY(), b.mMax.GetY() + com.GetY(), b.mMin.GetZ() + com.GetZ(), b.mMax.GetZ() + com.GetZ(),
                 wheel ? "  wheel" : "");
    }
    return s;
}

const std::vector<CarContactInfo> &CarContacts() { return g_carContacts; }
unsigned CarContactsDropped() { return g_carContactsDropped; }
uint32_t WheelSupport(const void *car) {
    const auto found = g_wheelSupport.find(car);
    return found != g_wheelSupport.end() ? found->second : 0;
}

int CarPartOf(const JPH::BodyID &car, unsigned subShapeId) {
    if (!Running()) return -1;
    JPH::SubShapeID sub;
    sub.SetValue(subShapeId);
    JPH::RefConst<JPH::Shape> shape = Bodies().GetShape(car);
    if (!shape || shape->GetSubType() != JPH::EShapeSubType::StaticCompound) return -1;
    JPH::SubShapeID rest;
    return (int)static_cast<const JPH::StaticCompoundShape *>(shape.GetPtr())->GetSubShapeIndexFromID(sub, rest);
}

const void *KinematicOwner(const JPH::BodyID &id) {
    auto it = g_carOfBody.find(id.GetIndexAndSequenceNumber());
    return it != g_carOfBody.end() ? it->second : nullptr;
}

bool HasKinematic(const void *key) { return g_kinematic.count(key) != 0; }

bool EnsureVehicle(const smoke::Mover &m) {
    if (!Running() || !m.key || m.kind != smoke::Mover::HULL) return false;
    V3 pos;
    JPH::Quat rot;
    unsigned version;
    if (!PoseOf(m, &pos, &rot, &version)) return false;
    bool created;
    return KinematicFor(m, pos, rot, version, &created) != nullptr;
}

void SetImpactListener(ImpactFn fn) { g_impact = fn; }

void RequestCabin(const void *car) { if (car) g_cabinWanted.insert(car); }
JPH::BodyID CabinBody(const void *car) {
    auto it = g_cabins.find(car);
    return it != g_cabins.end() ? it->second.body : JPH::BodyID();
}
unsigned CabinVersion(const void *car) {
    auto it = g_cabins.find(car);
    return it != g_cabins.end() ? it->second.version : 0;
}
void SetCabinGoneListener(CabinGoneFn fn) { g_cabinGone = fn; }
void SetCabinJumpListener(CabinJumpFn fn) { g_cabinJump = fn; }

const std::vector<CarKick> &CarKicks() { return g_carKicks; }
const std::vector<PersonBlow> &PersonBlows() { return g_personBlows; }
const std::vector<VehicleContact> &VehicleContacts() { return g_vehicleContacts; }



void SetWaterQuery(WaterFn fn) { g_water = fn; }
bool WaterAt(float x, float y, float z, float *surfaceY, V3 *normal, V3 *flow) {
    V3 n{0, 1, 0}, f{};
    return g_water && g_water(x, y, z, surfaceY, normal ? normal : &n, flow ? flow : &f);
}
void SetFloat(const JPH::BodyID &id, float buoyancy, float linearDrag, float angularDrag) {
    if (!id.IsInvalid()) g_floats[id.GetIndexAndSequenceNumber()] = Float{buoyancy, linearDrag, angularDrag};
}
void ClearFloat(const JPH::BodyID &id) { g_floats.erase(id.GetIndexAndSequenceNumber()); }
unsigned BodiesInWater() { return g_inWater; }

void MovingBounds(float cx, float cz, float reach, std::vector<Bounds> &out) {
    out.clear();
    if (!g_w) return;
    JPH::BodyIDVector all;
    g_w->system->GetBodies(all);
    JPH::BodyInterface &bi = Bodies();
    for (const JPH::BodyID &id : all) {
        if (bi.GetMotionType(id) == JPH::EMotionType::Static) continue;
        JPH::AABox box = bi.GetTransformedShape(id).GetWorldSpaceBounds();
        V3 lo = ToWorld(F(box.mMin)), hi = ToWorld(F(box.mMax));
        if (std::fabs(0.5f * (lo.x + hi.x) - cx) > reach || std::fabs(0.5f * (lo.z + hi.z) - cz) > reach) continue;
        out.push_back(Bounds{lo, hi});
    }
}

std::string Describe() {
    if (!g_w) return "jolt: off";
    unsigned cars = (unsigned)g_carOfBody.size(), people = (unsigned)g_kinematic.size() - cars, still = 0;
    for (const auto &kv : g_kinematic) still += kv.second.still;
    return Fmt("jolt: origin %.0f %.0f %.0f; %u bodies, %u awake (%u static pieces, %u meshes, %u waiting; %u car and %u "
               "body-part kinematics, %u of them still); step %.2f ms; scenery %.1f ms",
               g_w->origin.x, g_w->origin.y, g_w->origin.z, g_w->system->GetNumBodies(),
               g_w->system->GetNumActiveBodies(JPH::EBodyType::RigidBody), (unsigned)g_statics.size(),
               (unsigned)g_meshes.size(), g_staticsPending, cars, people, still, g_stepMs, g_sceneryMs) +
           Fmt("; %u floating bodies, %u in water%s", (unsigned)g_floats.size(), g_inWater, g_water ? "" : " (no water known)");
}

}
