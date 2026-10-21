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
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
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
    if (either(kCarBody)) return a == kDynamic || b == kDynamic;   // a driven car meets the loose bodies only
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

// Which dynamic bodies touched which car during a step (for the cars' reaction).
std::vector<std::pair<JPH::BodyID, const void *>> g_carTouches;
std::unordered_map<uint32_t, const void *> g_carOfBody;   // kinematic body id -> the car it stands for
std::unordered_map<uint32_t, const void *> g_passCar;     // dynamic body id -> the car whose hull it passes through
// Living people: their body-part capsules are kinematic (the game's animation moves them), so a hard contact would
// give them infinite strength: they bulldozed bodies as if weightless and forced pinned limbs past their joints.
// Their contacts with dynamic bodies are sensors instead; each tick the touched bodies get the impulse that stops
// them closing in on the person and lets them out of the overlap, at most `g_personForce` N per person.
std::unordered_map<uint32_t, const void *> g_personOfBody;   // body-part capsule id -> the person it belongs to
struct PersonTouch { JPH::BodyID person, body; const void *who; JPH::Vec3 n; JPH::RVec3 point; float depth; };
std::unordered_map<uint64_t, PersonTouch> g_personTouches;   // (capsule, body) -> the latest contact of the last step
float g_personForce = 700.0f;                                 // <= 0: hard contacts (infinite strength)
float g_personMass = 20.0f;                                   // kg behind each part of a person in an impact
std::vector<PersonBlow> g_personBlows;                        // the last step's (PersonBlows)
// Cars and bodies: see CarKicks (jolt_world.h). A car the game drives is a dynamic body of its own mass here, so the
// solver itself settles every contact with the bodies both ways; what it did to the car goes back to the game.
std::vector<CarKick> g_carKicks;
std::vector<CarContactInfo> g_carContacts;   // this step's (cleared before each step)
// A driven car's wheels: the parts of its hull made from the model's wheel frames (WHL0, WHR1...), a bit per part, by
// the car's body. Their hull does not turn, but a wheel rolls: its contacts with bodies get the rolling tyre's surface
// velocity and rubber's friction (WheelContact), so a wheel climbs onto what it meets as a tyre climbs a kerb.
std::unordered_map<uint32_t, uint64_t> g_wheelParts;
int CarPartOfSub(const JPH::Body &car, const JPH::SubShapeID &sub) {
    const JPH::Shape *shape = car.GetShape();
    if (!shape || shape->GetSubType() != JPH::EShapeSubType::StaticCompound) return -1;
    JPH::SubShapeID rest;
    return (int)static_cast<const JPH::StaticCompoundShape *>(shape)->GetSubShapeIndexFromID(sub, rest);
}

ImpactFn g_impact;
V3 OriginNow();

class Contacts final : public JPH::ContactListener {
public:
    void OnContactAdded(const JPH::Body &a, const JPH::Body &b, const JPH::ContactManifold &m, JPH::ContactSettings &s) override {
        Note(a, b);
        PersonContact(a, b, m, s);
        WheelContact(a, b, m, s);
        DrivenCarContact(a, b, m);
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
        Note(a, b);
        PersonContact(a, b, m, s);
        WheelContact(a, b, m, s);
        DrivenCarContact(a, b, m);
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
    static void DrivenCarContact(const JPH::Body &a, const JPH::Body &b, const JPH::ContactManifold &m) {
        bool carA = a.GetObjectLayer() == kCarBody, carB = b.GetObjectLayer() == kCarBody;
        if (carA == carB || g_carContacts.size() >= 64) return;
        const JPH::Body &car = carA ? a : b;
        auto owner = g_carOfBody.find(car.GetID().GetIndexAndSequenceNumber());
        if (owner == g_carOfBody.end()) return;
        JPH::Vec3 n = carA ? m.mWorldSpaceNormal : -m.mWorldSpaceNormal;   // from the car to the body
        JPH::RVec3 p = m.mRelativeContactPointsOn1.empty() ? m.mBaseOffset : m.GetWorldSpaceContactPointOn1(0);
        int part = CarPartOfSub(car, carA ? m.mSubShapeID1 : m.mSubShapeID2);
        g_carContacts.push_back(CarContactInfo{owner->second, part, V3{(float)p.GetX(), (float)p.GetY(), (float)p.GetZ()} + OriginNow(),
                                               V3{n.GetX(), n.GetY(), n.GetZ()}, m.mPenetrationDepth});
    }
    // A living person's capsule touching a dynamic body: no hard contact, remembered for ApplyPersonPushes.
    static void PersonContact(const JPH::Body &a, const JPH::Body &b, const JPH::ContactManifold &m, JPH::ContactSettings &s) {
        if (g_personForce <= 0 || g_personOfBody.empty()) return;
        auto pa = g_personOfBody.find(a.GetID().GetIndexAndSequenceNumber());
        auto pb = g_personOfBody.find(b.GetID().GetIndexAndSequenceNumber());
        const JPH::Body *person, *body;
        const void *who;
        JPH::Vec3 n = m.mWorldSpaceNormal;   // from a towards b
        if (pa != g_personOfBody.end() && b.IsDynamic()) { person = &a; body = &b; who = pa->second; }
        else if (pb != g_personOfBody.end() && a.IsDynamic()) { person = &b; body = &a; who = pb->second; n = -n; }
        else return;
        s.mIsSensor = true;
        JPH::RVec3 p = m.mRelativeContactPointsOn2.empty() ? m.mBaseOffset
                       : body == &b ? m.GetWorldSpaceContactPointOn2(0) : m.GetWorldSpaceContactPointOn1(0);
        uint64_t key = (uint64_t)person->GetID().GetIndexAndSequenceNumber() << 32 | body->GetID().GetIndexAndSequenceNumber();
        g_personTouches[key] = PersonTouch{person->GetID(), body->GetID(), who, n, p, m.mPenetrationDepth};
    }
    static bool Passes(const JPH::Body &body, const JPH::Body &other) {
        auto it = g_passCar.find(body.GetID().GetIndexAndSequenceNumber());
        if (it == g_passCar.end()) return false;
        auto car = g_carOfBody.find(other.GetID().GetIndexAndSequenceNumber());
        return car != g_carOfBody.end() && car->second == it->second;
    }
    static void Note(const JPH::Body &a, const JPH::Body &b) {
        const JPH::Body *dyn = a.IsDynamic() ? &a : b.IsDynamic() ? &b : nullptr;
        const JPH::Body *kin = a.IsKinematic() ? &a : b.IsKinematic() ? &b : nullptr;
        if (!dyn || !kin) return;
        auto it = g_carOfBody.find(kin->GetID().GetIndexAndSequenceNumber());
        if (it != g_carOfBody.end()) g_carTouches.push_back({dyn->GetID(), it->second});
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
std::unique_ptr<World> g_w;
bool g_typesRegistered;
// The water bodies float in (SetWaterQuery) and the bodies that float (SetFloat).
WaterFn g_water;
struct Float { float buoyancy, linearDrag, angularDrag; bool wet = false; float px = 0, pz = 0; };
std::vector<Splash> g_splashes;
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
    JPH::Quat rot = JPH::Quat::sIdentity();
    bool dynamic = false;
    bool still = false;   // a kinematic part left where it is (ApplyKinematics): its velocity zeroed once
    JPH::Vec3 vSet = JPH::Vec3::sZero(), wSet = JPH::Vec3::sZero();
};
std::unordered_map<const void *, Kinematic> g_kinematic;   // by mover key
unsigned g_tick;

// A mover's triangles (`hull`, three points each), both windings: seen from inside too. A car's cabin for the people
// in it, and the whole of a vehicle with no body of its own mass (Mover::hullAsMesh: a tram, hollow).
JPH::RefConst<JPH::Shape> TwoSidedMesh(const smoke::Mover &m);

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
    JPH::StaticCompoundShapeSettings cs;
    for (size_t i = 0; i < m.hullParts->size(); i++) {
        const std::vector<V3> &part = (*m.hullParts)[i];
        JPH::Array<JPH::Vec3> pts;
        pts.reserve(part.size());
        for (const V3 &p : part) pts.push_back(J(p));
        JPH::ConvexHullShapeSettings hs(pts, 0.01f);
        JPH::ShapeSettings::ShapeResult r = hs.Create();
        if (!r.IsValid()) continue;
        if (wheels && i < 64 && (m.wheelParts >> i & 1) && cs.mSubShapes.size() < 64) *wheels |= 1ull << cs.mSubShapes.size();
        cs.AddShape(JPH::Vec3::sZero(), JPH::Quat::sIdentity(), r.Get());
    }
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

void ApplyKinematics(float dt, const std::vector<smoke::Mover> &movers) {
    JPH::BodyInterface &bi = Bodies();
    g_tick++;
    for (const smoke::Mover &m : movers) {
        V3 pos; JPH::Quat rot; unsigned version;
        if (!m.key || !PoseOf(m, &pos, &rot, &version)) continue;
        bool car = m.kind != smoke::Mover::CAPSULE;
        auto it = g_kinematic.find(m.key);
        if (it != g_kinematic.end() && it->second.version != version) {   // new shape: made again
            g_carOfBody.erase(it->second.body.GetIndexAndSequenceNumber());
            g_personOfBody.erase(it->second.body.GetIndexAndSequenceNumber());
            g_wheelParts.erase(it->second.body.GetIndexAndSequenceNumber());
            bi.RemoveBody(it->second.body);
            bi.DestroyBody(it->second.body);
            g_kinematic.erase(it);
            it = g_kinematic.end();
        }
        if (it == g_kinematic.end()) {
            JPH::RefConst<JPH::Shape> shape;
            uint64_t wheels = 0;
            if (car) shape = CarShape(m, &wheels);
            else shape = new JPH::CapsuleShape(std::max(0.5f * Len(m.p1 - m.p0), 0.01f), std::max(m.r, 0.01f));
            if (!shape) continue;
            bool dyn = car && m.mass > 0;
            JPH::BodyCreationSettings s(shape, J(pos), rot, dyn ? JPH::EMotionType::Dynamic : JPH::EMotionType::Kinematic,
                                        dyn ? kCarBody : kKinematic);
            s.mFriction = car ? 0.5f : 0.7f;   // painted metal; clothes (a driven car's wheels: WheelContact)
            s.mRestitution = 0.0f;
            s.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            s.mMassPropertiesOverride.mMass = dyn ? m.mass : car ? 1200.0f : 5.0f;   // a kinematic body's is unused
            if (dyn) {
                s.mGravityFactor = 0.0f;   // the game holds it up
                s.mLinearDamping = 0.0f;
                s.mAngularDamping = 0.0f;
                s.mAllowSleeping = false;
                s.mMotionQuality = JPH::EMotionQuality::LinearCast;   // fast: no body passes through it
            }
            JPH::BodyID id = bi.CreateAndAddBody(s, JPH::EActivation::Activate);
            if (id.IsInvalid()) continue;
            if (car) g_carOfBody[id.GetIndexAndSequenceNumber()] = m.key;
            else g_personOfBody[id.GetIndexAndSequenceNumber()] = m.owner ? m.owner : m.key;
            if (dyn && wheels) g_wheelParts[id.GetIndexAndSequenceNumber()] = wheels;
            Kinematic nk{id, version, g_tick, pos};
            nk.rot = rot;
            nk.dynamic = dyn;
            g_kinematic[m.key] = nk;
            continue;
        }
        Kinematic &k = it->second;
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
            k.vSet = v;
            k.wSet = w;
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
        tris.push_back(JPH::Triangle(a, b, c));
        tris.push_back(JPH::Triangle(a, c, b));   // seen from inside too
    }
    JPH::MeshShapeSettings ms(tris);
    JPH::ShapeSettings::ShapeResult r = ms.Create();
    return r.IsValid() ? r.Get() : nullptr;
}

void ApplyCabins(float dt, const std::vector<smoke::Mover> &movers) {
    JPH::BodyInterface &bi = Bodies();
    for (const smoke::Mover &m : movers) {
        if (m.kind != smoke::Mover::HULL || !g_cabinWanted.count(m.key)) continue;
        V3 pos; JPH::Quat rot; unsigned version;
        if (!PoseOf(m, &pos, &rot, &version)) continue;
        auto it = g_cabins.find(m.key);
        if (it != g_cabins.end() && it->second.version != version) { DropCabin(it); it = g_cabins.end(); }
        if (it == g_cabins.end()) {
            JPH::RefConst<JPH::Shape> shape = TwoSidedMesh(m);
            if (!shape) continue;
            JPH::BodyCreationSettings s(shape, J(pos), rot, JPH::EMotionType::Kinematic, kCabin);
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
        if (dt <= 0 || Len(pos - k.pos) > 30.0f * dt + 2.0f) bi.SetPositionAndRotation(k.body, J(pos), rot, JPH::EActivation::Activate);
        else bi.MoveKinematic(k.body, J(pos), rot, dt);
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

// ---- the cars' reaction: the horizontal momentum the dynamic bodies touching a car gained during the step is
// what the car lost (estimated: the ground's friction on the same bodies is counted with it)
std::vector<Push> g_pushes;
struct Before { JPH::BodyID id; JPH::Vec3 v; float mass; };
std::vector<Before> g_before;

void RememberVelocities() {
    g_before.clear();
    JPH::BodyIDVector active;
    g_w->system->GetActiveBodies(JPH::EBodyType::RigidBody, active);
    const JPH::BodyLockInterfaceNoLock &locks = g_w->system->GetBodyLockInterfaceNoLock();
    for (const JPH::BodyID &id : active) {
        JPH::BodyLockRead lock(locks, id);
        if (!lock.Succeeded()) continue;
        const JPH::Body &b = lock.GetBody();
        if (!b.IsDynamic()) continue;
        float im = b.GetMotionProperties()->GetInverseMass();
        g_before.push_back({id, b.GetLinearVelocity(), im > 0 ? 1.0f / im : 0.0f});
    }
}

// What the step did to each driven car (see CarKicks): its velocity and turning now against what it was given.
void CollectCarKicks() {
    g_carKicks.clear();
    JPH::BodyInterface &bi = Bodies();
    for (auto &kv : g_kinematic) {
        const Kinematic &k = kv.second;
        if (!k.dynamic || !bi.IsAdded(k.body)) continue;
        JPH::Vec3 dv = bi.GetLinearVelocity(k.body) - k.vSet, dw = bi.GetAngularVelocity(k.body) - k.wSet;
        if (dv.LengthSq() < 1e-6f && dw.LengthSq() < 1e-6f) continue;
        JPH::RVec3 c = bi.GetCenterOfMassPosition(k.body);
        g_carKicks.push_back(CarKick{kv.first, V3{dv.GetX(), dv.GetY(), dv.GetZ()}, V3{dw.GetX(), dw.GetY(), dw.GetZ()},
                                     V3{(float)c.GetX(), (float)c.GetY(), (float)c.GetZ()} + g_w->origin});
    }
}

void CollectPushes() {
    g_pushes.clear();
    if (g_carTouches.empty()) return;
    std::sort(g_carTouches.begin(), g_carTouches.end(), [](const auto &a, const auto &b) {
        return a.first.GetIndexAndSequenceNumber() != b.first.GetIndexAndSequenceNumber()
                   ? a.first.GetIndexAndSequenceNumber() < b.first.GetIndexAndSequenceNumber()
                   : a.second < b.second;
    });
    g_carTouches.erase(std::unique(g_carTouches.begin(), g_carTouches.end()), g_carTouches.end());
    JPH::BodyInterface &bi = Bodies();
    for (const Before &b : g_before) {
        std::vector<const void *> cars;
        for (auto &t : g_carTouches) if (t.first == b.id) cars.push_back(t.second);
        if (cars.empty()) continue;
        JPH::Vec3 dv = bi.GetLinearVelocity(b.id) - b.v;
        V3 dp{dv.GetX() * b.mass, 0.0f, dv.GetZ() * b.mass};
        V3 share = dp * (-1.0f / (float)cars.size());
        for (const void *c : cars) {
            auto it = std::find_if(g_pushes.begin(), g_pushes.end(), [&](const Push &p) { return p.car == c; });
            if (it == g_pushes.end()) g_pushes.push_back({c, share});
            else it->impulse = it->impulse + share;
        }
    }
    g_carTouches.clear();
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
    g_carOfBody.clear();
    g_passCar.clear();
    g_personOfBody.clear();
    g_personTouches.clear();
    g_carKicks.clear();
    g_wheelParts.clear();
    g_meshes.clear();
    g_carTouches.clear();
    g_pushes.clear();
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
    g_w = std::move(w);
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
        g_w.reset();
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
// A car (the game moves it; it is kinematic here) in the water: its footprint along its motion, as deep as it sits,
// moving with it (a car driving into the river throws a wave; one sinking pushes the water aside).
std::unordered_map<uint32_t, V3> g_carWas;   // by body: where its middle was the last step
std::unordered_set<uint32_t> g_carWet;       // the cars whose underside was in the water the last step
void CarsInWater(float dt) {
    JPH::BodyInterface &bi = Bodies();
    std::unordered_map<uint32_t, V3> now;
    std::unordered_set<uint32_t> wet;
    for (auto &kv : g_carOfBody) {
        JPH::BodyID id(kv.first);
        if (!bi.IsAdded(id)) continue;
        JPH::AABox box = bi.GetTransformedShape(id).GetWorldSpaceBounds();
        V3 c = ToWorld(F(box.GetCenter()));
        now[kv.first] = c;
        float y;
        V3 n, flow;
        if (!g_water(c.x, c.z, &y, &n, &flow)) continue;
        float lo = box.mMin.GetY() + g_w->origin.y, hi = box.mMax.GetY() + g_w->origin.y;
        if (!(lo < y && hi > y - 0.2f)) continue;
        wet.insert(kv.first);
        auto was = g_carWas.find(kv.first);
        V3 p = was != g_carWas.end() ? was->second : c;
        // Falling in: a splash as hard as it came down (a car's mass: the game's own is not read here).
        float vy = dt > 0 && was != g_carWas.end() ? (c.y - p.y) / dt : 0;
        if (!g_carWet.count(kv.first) && vy < -0.5f) {
            float ex0 = box.GetExtent().GetX(), ez0 = box.GetExtent().GetZ();
            g_splashes.push_back({c.x, c.z, c.x, c.z, std::max(ex0, ez0), std::min(ex0, ez0), std::min(y - lo, 1.5f), 0, false, kv.first,
                                  -vy, 1300.0f});
        }
        V3 d{c.x - p.x, 0, c.z - p.z};
        float ex = box.GetExtent().GetX(), ez = box.GetExtent().GetZ();
        // Its size along its motion and across (the box is the world's: take the larger along the way it drives).
        float along = Len(d) > 1e-3f && std::fabs(d.x) > std::fabs(d.z) ? ex : ez, across = along == ex ? ez : ex;
        g_splashes.push_back({c.x, c.z, p.x, p.z, along, across, std::min(y - lo, 1.5f), dt, true, kv.first});
    }
    g_carWas.swap(now);
    g_carWet.swap(wet);
}

bool IsCarBody(uint32_t body) { return g_carOfBody.count(body) != 0; }

void ApplyWater(float dt) {
    g_inWater = 0;
    if (!g_water) return;
    CarsInWater(dt);
    if (g_floats.empty()) return;
    JPH::BodyInterface &bi = Bodies();
    JPH::Vec3 gravity = g_w->system->GetGravity();
    for (auto it = g_floats.begin(); it != g_floats.end();) {
        JPH::BodyID id(it->first);
        if (!bi.IsAdded(id)) { it = g_floats.erase(it); continue; }
        V3 c = ToWorld(F(bi.GetCenterOfMassPosition(id)));
        float y;
        V3 n, flow;
        Float &f = it->second;
        bool wet = false;
        if (g_water(c.x, c.z, &y, &n, &flow)) {
            JPH::AABox box = bi.GetTransformedShape(id).GetWorldSpaceBounds();
            float lo = box.mMin.GetY() + g_w->origin.y, hi = box.mMax.GetY() + g_w->origin.y;
            if (lo < y) {
                wet = true;
                V3 v = F(bi.GetLinearVelocity(id));
                float mass = 1.0f;
                {
                    JPH::BodyLockRead lock(g_w->system->GetBodyLockInterfaceNoLock(), id);
                    if (lock.Succeeded() && lock.GetBody().GetMotionPropertiesUnchecked())
                        mass = 1.0f / std::max(lock.GetBody().GetMotionPropertiesUnchecked()->GetInverseMass(), 1e-4f);
                }
                float ex = std::max(box.GetExtent().GetX(), 0.03f), ez = std::max(box.GetExtent().GetZ(), 0.03f);
                // Falling in: the surface pushed down by the body's momentum, over its size.
                if (!f.wet && v.y < -0.5f && hi > y - 1.0f)
                    g_splashes.push_back({c.x, c.z, c.x, c.z, std::max(ex, ez), std::max(ex, ez),
                                          std::min(std::max(0.012f * -v.y * std::sqrt(mass), 0.01f), 0.6f), 0, false, it->first, -v.y, mass});
                // In the surface: its footprint, as deep as it sits (capped), moving with it.
                if (f.wet && hi > y - 0.1f)
                    g_splashes.push_back({c.x, c.z, f.px, f.pz, ex, ez, std::min(y - lo, 0.6f), dt, true, it->first});
                f.px = c.x;
                f.pz = c.z;
                if (bi.ApplyBuoyancyImpulse(id, J(V3{c.x, y, c.z} - g_w->origin), J(n), f.buoyancy, f.linearDrag, f.angularDrag, J(flow), gravity, dt))
                    g_inWater++;
            }
        }
        f.wet = wet;
        ++it;
    }
}

// The pushes of living people (see g_personOfBody), from the contacts of the last step, per contact:
// - the impact: a collision of the body's part (its effective mass at that point, rotation included) with the
//   person's part, which has g_personMass behind it instead of an infinite one: a step kicks a body about;
// - the steady push: a fifth of the overlap out per step, all of one person's at most g_personForce * dt (a person
//   leaning on a body can push no harder than his feet hold on the ground).
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
        float closing = (bi.GetPointVelocity(t.person, t.point) - bi.GetPointVelocity(t.body, t.point)).Dot(t.n);
        float depth = std::min(std::max(t.depth, 0.0f), 0.1f);
        float impact = g_personMass > 0 && closing > 0 ? closing / (w + 1.0f / g_personMass) : 0.0f;   // N s
        // the blow into the person: the same impact with the body's own approach only (t.n points at the body), from a
        // body that moves as a thrown one does (1.5 m/s and more where it touches): a body lying in a walker's way is
        // shoved slowly by his own feet and bounces back at him (seen 2026-10-22: two of five people walking past
        // corpses fell over them, 487 such blows in four minutes)
        JPH::Vec3 bodyV = bi.GetPointVelocity(t.body, t.point);
        float approach = -bodyV.Dot(t.n);
        if (g_personMass > 0 && approach > 0 && bodyV.LengthSq() > 1.5f * 1.5f && g_personBlows.size() < 256) {
            float Jin = approach / (w + 1.0f / g_personMass);
            g_personBlows.push_back(PersonBlow{t.who, F(-t.n * Jin), F(JPH::Vec3(t.point)) + g_w->origin,
                                               t.body.GetIndexAndSequenceNumber()});
        }
        auto budget = left.try_emplace(t.who, g_personForce * dt).first;
        float steady = std::min(0.2f * depth / dt / w, budget->second);
        budget->second -= steady;
        float J = impact + std::max(steady, 0.0f);
        if (J <= 0) continue;
        bi.AddImpulse(t.body, t.n * J, t.point);
    }
    g_personTouches.clear();
}

void SetPersonPush(float maxForce, float mass) { g_personForce = maxForce; g_personMass = mass; }

void Step(float dt, const std::vector<smoke::Mover> &movers, const V3 *player) {
    g_personBlows.clear();   // (a paused tick has none)
    if (!g_w) return;
    Recentre(player);
    SetSceneryFocus(player);
    ApplyScenery();
    ApplyKinematics(dt, movers);
    ApplyCabins(dt, movers);
    if (dt <= 0) return;   // the game is paused: the bodies wait
    dt = std::min(dt, 1.0f / 20.0f);
    int collisionSteps = std::max(1, (int)std::ceil(dt * 120.0f - 1e-3f));
    double t0 = Now();
    RememberVelocities();
    g_carTouches.clear();
    ApplyPersonPushes(dt);
    ApplyWater(dt);
    g_carContacts.clear();
    JPH::EPhysicsUpdateError e = g_w->system->Update(dt, collisionSteps, g_w->temp.get(), g_w->jobs.get());
    CollectPushes();
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

void SetImpactListener(ImpactFn fn) { g_impact = fn; }

void RequestCabin(const void *car) { if (car) g_cabinWanted.insert(car); }
JPH::BodyID CabinBody(const void *car) {
    auto it = g_cabins.find(car);
    return it != g_cabins.end() ? it->second.body : JPH::BodyID();
}
void SetCabinGoneListener(CabinGoneFn fn) { g_cabinGone = fn; }

const std::vector<Push> &CarPushes() { return g_pushes; }
const std::vector<CarKick> &CarKicks() { return g_carKicks; }
const std::vector<PersonBlow> &PersonBlows() { return g_personBlows; }

// The vehicle whose body is the first solid thing (the street, a vehicle; not a body nor a person) along `way` from
// `at` (world), or null.
const void *FirstVehicle(const V3 &at, const V3 &way) {
    class Solid final : public JPH::ObjectLayerFilter {
    public:
        bool ShouldCollide(JPH::ObjectLayer l) const override { return l == kStatic || l == kKinematic || l == kCarBody; }
    } solid;
    JPH::RRayCast ray(J(at - g_w->origin), J(way));
    JPH::RayCastResult hit;
    if (!System()->GetNarrowPhaseQueryNoLock().CastRay(ray, hit, {}, solid)) return nullptr;
    auto car = g_carOfBody.find(hit.mBodyID.GetIndexAndSequenceNumber());
    return car == g_carOfBody.end() ? nullptr : car->second;
}

const void *VehicleAround(const V3 &at, float down, float up) {
    if (!Running() || down <= 0 || up <= 0) return nullptr;
    const void *below = FirstVehicle(at, {0, -down, 0});
    return below && FirstVehicle(at, {0, up, 0}) == below ? below : nullptr;
}

void SetWaterQuery(WaterFn fn) { g_water = fn; }
bool WaterAt(float x, float z, float *surfaceY, V3 *normal, V3 *flow) {
    V3 n{0, 1, 0}, f{};
    return g_water && g_water(x, z, surfaceY, normal ? normal : &n, flow ? flow : &f);
}
void SetFloat(const JPH::BodyID &id, float buoyancy, float linearDrag, float angularDrag) {
    if (!id.IsInvalid()) g_floats[id.GetIndexAndSequenceNumber()] = Float{buoyancy, linearDrag, angularDrag};
}
void ClearFloat(const JPH::BodyID &id) { g_floats.erase(id.GetIndexAndSequenceNumber()); }
void TakeSplashes(std::vector<Splash> &out) {
    out.insert(out.end(), g_splashes.begin(), g_splashes.end());
    g_splashes.clear();
}
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
