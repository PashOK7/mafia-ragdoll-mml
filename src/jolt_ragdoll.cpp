// Ragdolls on Jolt Physics (see jolt_ragdoll.h).
#include "fmt.h"
#include "jolt_ragdoll.h"
#include "pull_reach.h"
#include "anatomy.h"
#include "joint_resistance.h"
#include "articulation.h"
#include "vehicles.h"
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <unordered_map>
#include "jolt_world.h"
#include "vehicles.h"

// The Ragdoll's insides, for this file only (Ragdoll names it a friend).
struct JoltBridge {
    using Body = Ragdoll::Body;
    static Body &B(Ragdoll *r, int i) { return r->body[i]; }
    static const Body &B(const Ragdoll *r, int i) { return r->body[i]; }
    static const V3 *Points(const Ragdoll *r) { return r->initPos; }
    static V3 Facing(const Ragdoll *r) { return r->initFwd; }
    static V3 Origin(const Ragdoll *r) { return r->origin; }
    static const M3 *Rotations(const Ragdoll *r) { return r->initRot; }
    static V3 Rot(const Ragdoll::Quat &q, const V3 &v) { return Ragdoll::Rotate(q, v); }
    static V3 InvRot(const Ragdoll::Quat &q, const V3 &v) { return Ragdoll::InvRotate(q, v); }
};

namespace jolt_ragdoll {
namespace {

LogFn g_log;
#define RLOG(...) (g_log ? g_log(__VA_ARGS__) : (void)0)


constexpr float kDeg = 0.0174533f;

// ---- live tuning (`jolt set`, the effects panel's "Ragdolls" tab)
enum Tune { kFriction, kAngDamping, kLinDamping, kJointFriction, kDeathStrength, kDeathFade, kFollowAnim,
            kKnockStrength, kKnockFade, kMuscleHz, kVelocitySteps, kPositionSteps, kCcd, kBuoyancy, kWaterDrag, kPushForce,
            kPushMass, kTunes };
using Tunable = tunables::Row<Tune>;
Tunable g_tune[kTunes] = {   // each row names its id: Init refuses a table out of order
    // id, name, value, slider min, max, step, section, what

    {kFriction, "friction", 0.9f, 0, 1.5f, 0, "Body", "friction of the body on the world (clothes on asphalt: about 0.9)"},
    {kAngDamping, "angdamping", 0.5f, 0, 5, 0, "Body", "1/s: rotation lost to the soft body (stops limbs spinning like a propeller)"},
    {kLinDamping, "lindamping", 0.05f, 0, 1, 0, "Body", "1/s: motion lost to the air"},
    {kJointFriction, "jointfriction", 1, 0, 5, 0, "Joints", "scale of the friction in every joint (1: a limp human body)"},
    {kDeathStrength, "deathstrength", 0.3f, 0, 1, 0, "Muscles: death", "muscle strength at the moment of death (1: a living person's full strength)"},
    {kDeathFade, "deathfade", 1.0f, 0.05f, 4, 0, "Muscles: death", "s: until the muscles of a dying body are limp"},
    {kFollowAnim, "followanim", 1, 0, 1, 1, "Muscles: death",
     "1: the muscles follow the game's death animation while it plays; 0: they hold the pose of the moment of death"},
    {kKnockStrength, "knockstrength", 0.5f, 0, 1, 0, "Muscles: car", "muscle strength of a person a car knocks over (tensing at the impact)"},
    {kKnockFade, "knockfade", 0.4f, 0.05f, 4, 0, "Muscles: car", "s: until that tension is gone"},
    {kMuscleHz, "musclehz", 4, 0.5f, 15, 0, "Muscles", "Hz: how fast a muscle pulls its joint towards the target (spring frequency)"},
    {kVelocitySteps, "velocitysteps", 20, 4, 50, 1, "Solver", "velocity iterations for a ragdoll's island (stiffer joints; new ragdolls)"},
    {kPositionSteps, "positionsteps", 6, 1, 20, 1, "Solver", "position iterations for a ragdoll's island (less joint stretch; new ragdolls)"},
    {kCcd, "ccd", 1, 0, 1, 1, "Solver", "1: continuous collision for every part (no tunnelling through cars and thin walls; new ragdolls)"},
    {kBuoyancy, "buoyancy", 1, 0, 2, 0, "Water",
     "how a body floats (1: as a human body, the chest with air in it high, the limbs low; below 1 sinks; new ragdolls)"},
    {kWaterDrag, "waterdrag", 0.6f, 0, 3, 0, "Water", "how much the water slows a body and its limbs (new ragdolls)"},
    {kPushForce, "pushforce", 700, 0, 3000, 0, "Living people",
     "N: one living person's total effort across all contacts, including steps and overlap pushes: ground traction. "
     "0: as hard as the game moves him (infinite strength: the old way)"},
    {kPushMass, "pushmass", 20, 0, 100, 0, "Living people",
     "kg: upper bound on each anatomical segment's impact mass; actual masses match the ragdoll's segments. "
     "0: no impacts, the steady push only"},
};
// How each part floats against its own weight: the chest holds the lungs' air, bone and muscle in the limbs sink.
// By mass the body comes to 1.06: a human body with air in its lungs is a little lighter than water, so it floats
// low, the back up and the limbs hanging.
const float kFloat[Ragdoll::BCOUNT] = {1.02f, 1.08f, 1.25f, 1.35f, 1.0f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f};
float T(Tune t) { return g_tune[t].value; }

JPH::Vec3 J(const V3 &v) { return JPH::Vec3(v.x, v.y, v.z); }
V3 F(JPH::Vec3Arg v) { return V3{v.GetX(), v.GetY(), v.GetZ()}; }
JPH::Quat J(const Ragdoll::Quat &q) { return JPH::Quat(q.x, q.y, q.z, q.w).Normalized(); }
Ragdoll::Quat F(JPH::QuatArg q) { return Ragdoll::Quat{q.GetW(), q.GetX(), q.GetY(), q.GetZ()}; }

// ---- the body
enum B {
    PELVIS = Ragdoll::B_PELVIS, ABDOMEN = Ragdoll::B_ABDOMEN, CHESTLOW = Ragdoll::B_CHESTLOW, CHEST = Ragdoll::B_CHEST,
    HEAD = Ragdoll::B_HEAD, RUARM = Ragdoll::B_RUARM, RFARM = Ragdoll::B_RFARM, LUARM = Ragdoll::B_LUARM,
    LFARM = Ragdoll::B_LFARM, RTHIGH = Ragdoll::B_RTHIGH, RSHIN = Ragdoll::B_RSHIN, RFOOT = Ragdoll::B_RFOOT,
    LTHIGH = Ragdoll::B_LTHIGH, LSHIN = Ragdoll::B_LSHIN, LFOOT = Ragdoll::B_LFOOT, BODIES = Ragdoll::BCOUNT
};
const int kParent[BODIES] = {-1, PELVIS, ABDOMEN, CHESTLOW, CHEST, CHEST, RUARM, CHEST, LUARM, PELVIS, RTHIGH, RSHIN, PELVIS, LTHIGH, LSHIN};
const char *const kName[BODIES] = {"pelvis", "abdomen", "chestlow", "chest", "head", "r_upperarm", "r_forearm", "l_upperarm",
                                   "l_forearm", "r_thigh", "r_shin", "r_foot", "l_thigh", "l_shin", "l_foot"};
// Shared with the living contact proxies; forearms include the hands.
constexpr auto &kMass = Ragdoll::SegmentMass;

// Swing and twist limits in degrees. Hinges use flex and side as their bend limits.
struct Anatomy { float flex, side, twistLo, twistHi; Resistance passive; float muscle; };
const Anatomy kSpineLow{25, 15, -8, 8, {10, 6}, 180}, kSpineMid{20, 15, -15, 15, {10, 6}, 180},
    kSpineHigh{15, 12, -20, 20, {10, 6}, 180}, kNeck{50, 40, -70, 70, neckResistance, 40},
    kShoulder{75, 80, -70, 70, {.4f, 2}, 80}, kHip{70, 35, -40, 40, {8, 6}, 250},
    kAnkle{35, 20, -12, 12, {.3f, 1}, 120};
const Anatomy kElbow{0, 145, 0, 0, {.2f, 1}, 60}, kKnee{0, 150, 0, 0, {2, 2.5f}, 200};

struct Entry {
    unsigned serial = 0;
    JPH::Ref<JPH::RagdollSettings> settings;
    JPH::Ref<JPH::Ragdoll> ragdoll;
    bool hinge[BODIES] = {};
    V3 hingeAxisParent[BODIES] = {};      // hinge: its axis in the parent body's frame
    V3 hingeNormalParent[BODIES] = {}, hingeNormalChild[BODIES] = {};
    JPH::Quat target[BODIES];             // ball: child rotation relative to the parent, the muscles' target
    float targetAngle[BODIES] = {};       // hinge: the target angle
    const Anatomy *anatomy[BODIES] = {};
    unsigned animSeen = 0;
    bool motorsOn = false;
    float age = 0;
    const void *car = nullptr;              // seated in this car's cabin
    JPH::Ref<JPH::Constraint> seat;         // the pelvis held to the seat
    JPH::Ref<JPH::ConstraintSettings> seatAnchor; // local anchors retained across a carrier replacement
    const void *passCar = nullptr;          // born inside this car's hull (killed in its open door): passes through it
    float passClear = 0;                    // s since no part is inside that hull any more
    JPH::Ref<JPH::PointConstraint> grip;   // point on the driver's trunk reached by Tommy's hands
    JPH::BodyID handAnchor;
    bool reached = false;
    V3 lastHands{}, reachOffset{}, anchorTarget{}, outward{}, gripAxis{}, localGripAxis{};
    V3 localContact[2]{};
    int gripPart = -1;
    bool atContact = false;
    V3 pullVelocity{};
    float reachBlend = 0;
    float duck = 0;
    float reachStart = 0, deepestReach = 0, gripStart = 0, lastDepth = 0;
    float farthestPull = 0;

    // Held still until the static world around it is there (after a core reload, in a part of the city just
    // reached): its parts asleep, the velocities they had kept for when it goes.
    bool holding = false;
    float held = 0;
    JPH::Vec3 heldV[BODIES], heldW[BODIES];
};
std::unordered_map<const Ragdoll *, Entry> g_entries;
unsigned g_nextGroup = 1;

V3 Orth(const V3 &v, const V3 &axis) { return Norm(v - axis * Dot(v, axis), Cross(axis, std::fabs(axis.y) < 0.9f ? V3{0, 1, 0} : V3{1, 0, 0})); }

// The neutral direction `c` tilted by `deg` towards `towards`.
V3 Tilt(const V3 &c, const V3 &towards, float deg) {
    V3 t = Orth(towards, c);
    return Norm(c * std::cos(deg * kDeg) + t * std::sin(deg * kDeg));
}

// The swing-twist joint between `parent` and `child` at `anchor` (Jolt space). `c`: the neutral direction of the
// child's bone (world), `f`: the joint's forward axis (world, the direction the child's own z axis has in the
// neutral pose). Calibrate in the file's rest skeleton, then carry each joint frame with its own posed body.
// A death animation never redefines the anatomy or permanently widens its limits.
JPH::Ref<JPH::SwingTwistConstraintSettings> Ball(const Ragdoll *r, const Ragdoll *reference, int parent, int child, const V3 &anchor, V3 c, V3 f,
                                                 const Anatomy &a) {
    const auto &rp = JoltBridge::B(reference,parent);
    const auto &pb = JoltBridge::B(r,parent);
    const JoltBridge::Body &cb = JoltBridge::B(r, child);
    V3 bone = Norm(JoltBridge::Rot(cb.q, {1, 0, 0}));
    V3 z2 = JoltBridge::Rot(cb.q, {0, 0, 1});
    c = Norm(c);
    f = Orth(f, c);
    c = JoltBridge::Rot(pb.q,JoltBridge::InvRot(rp.q,c));
    f = JoltBridge::Rot(pb.q,JoltBridge::InvRot(rp.q,f));

    return articulation::Ball(anchor,{c,f},{bone,z2},a.flex,a.side,a.twistLo,a.twistHi,T(kMuscleHz));
}

// The hinge between `parent` and `child` at `anchor`: positive angles bend the child's bone about `axis` (world).
JPH::Ref<JPH::HingeConstraintSettings> Hinge(const Ragdoll *r, const Ragdoll *reference, int parent, int child, const V3 &anchor, const V3 &axis,
                                             const Anatomy &a, float *startAngle) {
    const JoltBridge::Body &pb = JoltBridge::B(r, parent), &cb = JoltBridge::B(r, child);
    const auto &rp = JoltBridge::B(reference,parent), &rc = JoltBridge::B(reference,child);
    V3 h = Norm(axis);
    V3 n = Orth(JoltBridge::Rot(rp.q, {1, 0, 0}), h);
    V3 h1=JoltBridge::Rot(pb.q,JoltBridge::InvRot(rp.q,h)), h2=JoltBridge::Rot(cb.q,JoltBridge::InvRot(rc.q,h));
    V3 n1=JoltBridge::Rot(pb.q,JoltBridge::InvRot(rp.q,n)), n2=JoltBridge::Rot(cb.q,JoltBridge::InvRot(rc.q,n));
    float phi = std::atan2(Dot(h1, Cross(n1, n2)), Dot(n1, n2));
    *startAngle = phi;
    // The normals are the two bones: Jolt's angle 0 is where they meet, so the angle is the bend from straight.
    return articulation::Hinge(anchor,{h1,n1},{h2,n2},a.flex,a.side,T(kMuscleHz));
}

// A part's collision shape in its body frame (x along the bone; for the pelvis, the trunk and the head z forward;
// for the feet z up from the sole).
JPH::RefConst<JPH::Shape> ShapeOf(const Ragdoll *r, int b, const V3 &up) {
    const JoltBridge::Body &bd = JoltBridge::B(r, b);
    float L = Len(bd.b - bd.a);
    V3 upLocal = JoltBridge::InvRot(bd.q, up);
    auto capsule = [&](float radius) -> JPH::RefConst<JPH::Shape> {
        float hh = std::max(0.01f, 0.5f * L - 0.5f * radius);
        return new JPH::RotatedTranslatedShape(JPH::Vec3::sZero(), JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), -0.5f * JPH::JPH_PI),
                                               new JPH::CapsuleShape(hh, radius));
    };
    auto box = [&](const V3 &half, const V3 &offset, float convex) -> JPH::RefConst<JPH::Shape> {
        return new JPH::RotatedTranslatedShape(J(offset), JPH::Quat::sIdentity(), new JPH::BoxShape(J(half), convex));
    };
    switch (b) {
    case PELVIS:   // hip to hip along x; the pelvis reaches higher above the hip joints than below them
        return box({0.5f * L + 0.07f, 0.10f, 0.11f}, upLocal * 0.03f, 0.04f);
    case ABDOMEN: return box({0.5f * L + 0.03f, 0.14f, 0.10f}, {0, 0, 0}, 0.04f);
    case CHESTLOW: return box({0.5f * L + 0.03f, 0.155f, 0.11f}, {0, 0, 0}, 0.04f);
    case CHEST: return box({0.5f * L + 0.04f, 0.175f, 0.12f}, {0, 0, 0}, 0.05f);
    case HEAD: return new JPH::RotatedTranslatedShape(JPH::Vec3(0.1f * L, 0, 0.02f), JPH::Quat::sIdentity(), new JPH::SphereShape(0.10f));
    case RUARM: case LUARM: return capsule(0.05f);
    case RFARM: case LFARM: {
        V3 elbow, wrist, tip;
        if (!r->HandShape(b, &elbow, &wrist, &tip)) return capsule(0.042f);
        JPH::StaticCompoundShapeSettings compound;
        auto addCapsule = [&](const V3 &a, const V3 &e, float radius) {
            JPH::Quat axis = JPH::Quat::sFromTo(JPH::Vec3::sAxisY(), J(Norm(e - a, {0, 1, 0})));
            compound.AddShape(J((a + e) * 0.5f), axis, new JPH::CapsuleShape(std::max(0.01f, Len(e - a) * 0.5f), radius));
        };
        addCapsule(elbow, wrist, bd.radius);
        addCapsule(wrist, tip, Ragdoll::PalmRadius);
        auto result = compound.Create();
        return result.IsValid() ? result.Get() : capsule(0.042f);
    }
    case RTHIGH: case LTHIGH: return capsule(0.075f);
    case RSHIN: case LSHIN: return capsule(0.055f);
    case RFOOT: case LFOOT: return box({0.5f * L + 0.02f, 0.045f, 0.035f}, {0, 0, -0.02f}, 0.02f);
    }
    return nullptr;
}

// The relative rotations of a pose (parent^-1 * child) and the hinge angles, from body frames set up on it.
void TargetsFrom(const Ragdoll &pose, Entry &e) {
    for (int b = 1; b < BODIES; b++) {
        const JoltBridge::Body &pb = JoltBridge::B(&pose, kParent[b]), &cb = JoltBridge::B(&pose, b);
        if (e.hinge[b]) {
            V3 h = Norm(JoltBridge::Rot(pb.q, e.hingeAxisParent[b]));
            V3 n1 = JoltBridge::Rot(pb.q,e.hingeNormalParent[b]), n2 = JoltBridge::Rot(cb.q,e.hingeNormalChild[b]);
            e.targetAngle[b] = std::atan2(Dot(h, Cross(n1, n2)), Dot(n1, n2));
        } else {
            e.target[b] = (J(pb.q).Conjugated() * J(cb.q)).Normalized();
        }
    }
}

// The muscles' strength now: from the moment of death (or of the impact) down to nothing.
float Strength(const Ragdoll *r, float age) {
    if (r->vehicleImpact) return 0;
    float s0 = r->knockdown ? T(kKnockStrength) : T(kDeathStrength), fade = r->knockdown ? T(kKnockFade) : T(kDeathFade);
    if (age >= fade || s0 <= 0) return 0;
    float u = 1.0f - age / fade;
    return s0 * u * u;
}

void Muscles(Ragdoll *r, Entry &e, float dt) {
    e.age += dt;
    float s = Strength(r, e.age);
    if (s <= 0) {
        if (e.motorsOn) {
            for (int b = 1; b < BODIES; b++) {
                JPH::TwoBodyConstraint *c = e.ragdoll->GetConstraint(e.settings->GetConstraintIndexForBodyIndex(b));
                if (e.hinge[b]) static_cast<JPH::HingeConstraint *>(c)->SetMotorState(JPH::EMotorState::Off);
                else {
                    auto *st = static_cast<JPH::SwingTwistConstraint *>(c);
                    st->SetSwingMotorState(JPH::EMotorState::Off);
                    st->SetTwistMotorState(JPH::EMotorState::Off);
                }
            }
            e.motorsOn = false;
        }
        if (e.ragdoll->IsActive(false)) {
            const float scale = T(kJointFriction);
            for (int b = 1; b < BODIES; ++b) {
                auto *joint = e.ragdoll->GetConstraint(e.settings->GetConstraintIndexForBodyIndex(b));
                SetPassiveResistance(*joint, e.anatomy[b]->passive, scale);
            }
        }
        return;
    }
    // The game's death animation, while the engine plays it: a new pose since the last one we took.
    if (!r->knockdown && T(kFollowAnim) > 0.5f && r->animSerial != e.animSeen) {
        e.animSeen = r->animSerial;
        Ragdoll pose;
        pose.Setup(r->anim.pts, r->anim.fwd, r->anim.rot);
        TargetsFrom(pose, e);
    }
    for (int b = 1; b < BODIES; b++) {
        JPH::TwoBodyConstraint *c = e.ragdoll->GetConstraint(e.settings->GetConstraintIndexForBodyIndex(b));
        float torque = s * e.anatomy[b]->muscle;
        if (e.hinge[b]) {
            auto *h = static_cast<JPH::HingeConstraint *>(c);
            h->GetMotorSettings().SetTorqueLimit(torque);
            h->SetMotorState(JPH::EMotorState::Position);
            h->SetTargetAngle(e.targetAngle[b]);
        } else {
            auto *st = static_cast<JPH::SwingTwistConstraint *>(c);
            st->GetSwingMotorSettings().SetTorqueLimit(torque);
            st->GetTwistMotorSettings().SetTorqueLimit(torque);
            st->SetSwingMotorState(JPH::EMotorState::Position);
            st->SetTwistMotorState(JPH::EMotorState::Position);
            st->SetTargetOrientationBS(e.target[b]);
        }
    }
    e.motorsOn = true;
}

void ReadBack(Ragdoll *r, Entry &e, float dt) {
    JPH::BodyInterface &bi = jolt_world::Bodies();
    V3 shift = jolt_world::Origin() - JoltBridge::Origin(r);   // Jolt space -> the ragdoll's local frame
    bool quiet = r->knockdown && !e.holding;
    for (int b = 0; b < BODIES; b++) {
        JPH::BodyID id = e.ragdoll->GetBodyID(b);
        JPH::RVec3 p;
        JPH::Quat q;
        bi.GetPositionAndRotation(id, p, q);
        JoltBridge::Body &bd = JoltBridge::B(r, b);
        bd.x = shift + F(p);
        bd.q = F(q);
        bd.v = F(bi.GetLinearVelocity(id));
        bd.w = F(bi.GetAngularVelocity(id));
        if (quiet) {
            // Include rotation at a point 20 cm from the centre so a spinning limb still delays recovery.
            constexpr float kMaxRecoverySpeed = 0.25f;
            quiet = Dot(bd.v, bd.v) + 0.04f * Dot(bd.w, bd.w) <= kMaxRecoverySpeed * kMaxRecoverySpeed;
        }
    }
    r->sleeping = !e.holding && !e.ragdoll->IsActive(false);
    r->sleepTimer = quiet ? r->sleepTimer + dt : 0;
}

bool g_inOrder;

void ReleaseGrip(Entry &e, const Ragdoll *r, bool endAction = true) {
    if (endAction) jolt_world::SetHolder(r, nullptr);

    if (e.grip && jolt_world::Running()) jolt_world::System()->RemoveConstraint(e.grip);
    e.grip = nullptr;
    if (!e.handAnchor.IsInvalid() && jolt_world::Running()) {
        auto &bi = jolt_world::Bodies();
        bi.RemoveBody(e.handAnchor);
        bi.DestroyBody(e.handAnchor);
    }
    e.handAnchor = JPH::BodyID();
    e.reached = false;
}

void Unseat(Entry &e) {
    if (e.seat) { jolt_world::System()->RemoveConstraint(e.seat); e.seat = nullptr; }
    e.seatAnchor = nullptr;
    if (e.car && e.ragdoll) {   // free now: an ordinary body again
        JPH::BodyInterface &bi = jolt_world::Bodies();
        for (int b = 0; b < BODIES; b++) bi.SetObjectLayer(e.ragdoll->GetBodyID(b), jolt_world::kDynamic);
    }
    e.car = nullptr;
}

// Only release a loose body once every part has cleared the visual model in cabin space.
bool OutsideCabin(const Entry &e) {
    const auto cabin = jolt_world::CabinBody(e.car);
    if (cabin.IsInvalid()) return false;
    const auto &bi = jolt_world::Bodies();
    const auto local = bi.GetCenterOfMassTransform(cabin).InversedRotationTranslation();
    const auto bounds = bi.GetShape(cabin)->GetLocalBounds();
    if (bounds.Contains(local * bi.GetCenterOfMassPosition(e.ragdoll->GetBodyID(PELVIS)))) return false;
    for (int b = 0; b < BODIES; ++b) {
        const auto id = e.ragdoll->GetBodyID(b);
        const auto pose = local * bi.GetCenterOfMassTransform(id);
        if (bounds.Overlaps(bi.GetShape(id)->GetWorldSpaceBounds(pose, JPH::Vec3::sOne()))) return false;
    }
    return true;
}

void OnCabinGone(const void *car) {
    for (auto &kv : g_entries) if (kv.second.car == car) Unseat(kv.second);
}

// The car jumped (a teleport): the bodies seated in it go with it, each part where it was in the car, at rest (left
// behind, the seat's spring dragged the body after its car through the street and the floor).
void OnCabinJump(const void *car, const JPH::Vec3 &fromPos, const JPH::Quat &fromRot, const JPH::Vec3 &toPos, const JPH::Quat &toRot) {
    JPH::BodyInterface &bi = jolt_world::Bodies();
    JPH::Quat turn = (toRot * fromRot.Conjugated()).Normalized();
    for (auto &kv : g_entries) {
        Entry &e = kv.second;
        if (e.car != car || !e.ragdoll) continue;
        for (int b = 0; b < BODIES; b++) {
            JPH::BodyID id = e.ragdoll->GetBodyID(b);
            JPH::RVec3 p;
            JPH::Quat q;
            bi.GetPositionAndRotation(id, p, q);
            bi.SetPositionAndRotation(id, toPos + turn * (p - fromPos), (turn * q).Normalized(), JPH::EActivation::Activate);
            bi.SetLinearAndAngularVelocity(id, JPH::Vec3::sZero(), JPH::Vec3::sZero());
        }
        RLOG("ragdoll %p carried with its car %p's jump", kv.first, car);
    }
}

// Refit the leg between the corrected hip and the original footwell position.
// Moving the whole seated skeleton forward can put the shin and foot on
// opposite sides of the dashboard before the first collision step.
static void FitSeatedLeg(const Ragdoll *r, Entry &e, const JPH::TransformedShape &cabin,
                         int thigh, int shin, int foot, JPH::Vec3Arg shift, JPH::Vec3Arg forward, JPH::Vec3Arg up) {
    auto &bi = jolt_world::Bodies();
    const JPH::BodyID ids[] = {e.ragdoll->GetBodyID(thigh), e.ragdoll->GetBodyID(shin), e.ragdoll->GetBodyID(foot)};
    const auto upper = bi.GetWorldTransform(ids[0]), lower = bi.GetWorldTransform(ids[1]), shoe = bi.GetWorldTransform(ids[2]);
    const auto &u = JoltBridge::B(r, thigh), &l = JoltBridge::B(r, shin);
    const auto hip = upper * J(u.a) + shift, oldKnee = upper * J(u.b), oldAnkle = lower * J(l.b);
    const float a = Len(u.b - u.a), b = Len(l.b - l.a);
    if (!(a > .001f && b > .001f)) {
        for (auto id : ids) bi.SetPosition(id, bi.GetPosition(id) + shift, JPH::EActivation::Activate);
        return;
    }
    const auto makeLink = [&](const JPH::RMat44 &oldPose, const JoltBridge::Body &segment,
                              JPH::Vec3Arg start, JPH::Vec3Arg end) {
        const auto q = oldPose.GetQuaternion();
        const auto direction = (q * J(segment.b - segment.a)).Normalized();
        const auto rotation = JPH::Quat::sFromTo(direction, (end - start).Normalized()) * q;
        return JPH::RMat44::sRotationTranslation(rotation, start - rotation * J(segment.a));
    };
    auto penetration = [&](const JPH::RMat44 *pose, float stopAt) {
        float depth = 0;
        for (int i = 0; i < 3 && depth < stopAt; ++i) {
            const auto shape = bi.GetShape(ids[i]);
            JPH::ClosestHitCollisionCollector<JPH::CollideShapeCollector> hit;
            cabin.CollideShape(shape, JPH::Vec3::sOne(), pose[i].PreTranslated(shape->GetCenterOfMass()),
                               {}, JPH::RVec3::sZero(), hit);
            if (hit.HadHit()) depth = std::max(depth, hit.mHit.mPenetrationDepth);
        }
        return depth;
    };
    JPH::RMat44 best[] = {upper, lower, shoe};
    for (auto &pose : best) pose.SetTranslation(pose.GetTranslation() + shift);
    const float initialDepth = penetration(best, FLT_MAX);
    float bestCost = initialDepth;
    const float slop = jolt_world::System()->GetPhysicsSettings().mPenetrationSlop;
    // A shoe can start inside the footwell's sloping front wall. Try the
    // nearest reachable placements towards the seat and above the floor;
    // test the full shapes of the thigh, shin and shoe at each placement.
    for (int step = 0; initialDepth > slop && step <= 12; ++step) {
        for (int lift = 0; lift <= step && lift <= 6; ++lift) {
            const float back = (step - lift) * .04f, height = lift * .04f;
            const auto toAnkle = JPH::Vec3(oldAnkle - hip) - forward * back + up * height;
            const auto axis = toAnkle.NormalizedOr(-up);
            const float reach = std::clamp(toAnkle.Length(), std::abs(a - b) + .0001f, a + b - .0001f);
            const float along = (a*a - b*b + reach*reach) / (2.f * reach);
            const auto bend = JPH::Vec3(oldKnee - hip);
            const auto pole = (bend - axis * bend.Dot(axis)).NormalizedOr(axis.GetNormalizedPerpendicular());
            const auto knee = hip + axis * along + pole * std::sqrt(std::max(0.f, a*a - along*along));
            const auto ankle = hip + axis * reach;
            const float displacementCost = .025f * (ankle - oldAnkle).Length();
            if (displacementCost >= bestCost) continue;
            JPH::RMat44 pose[] = {makeLink(upper, u, hip, knee), makeLink(lower, l, knee, ankle), shoe};
            pose[2].SetTranslation(shoe.GetTranslation() + (ankle - oldAnkle));
            const float cost = penetration(pose, bestCost - displacementCost) + displacementCost;
            if (cost < bestCost) {
                bestCost = cost;
                std::copy_n(pose, 3, best);
            }
        }
        if (bestCost <= .002f) break;
    }
    for (int i = 0; i < 3; ++i)
        bi.SetPositionAndRotation(ids[i], best[i].GetTranslation(), best[i].GetQuaternion(), JPH::EActivation::Activate);
    if (bestCost < initialDepth)
        RLOG("seated %s footwell: cabin penetration %.3f -> %.3f m", foot == LFOOT ? "left" : "right",
             initialDepth, penetration(best, FLT_MAX));
}

// The native seated pose can straddle the two faces of an upholstered backrest.
// Place the rigid trunk on the knee side before its pelvis is attached to the seat.
static void FitSeatBack(const Ragdoll *r, Entry &e, JPH::BodyID cabinID) {
    auto &bi = jolt_world::Bodies();
    const auto cabin = bi.GetTransformedShape(cabinID);
    const auto hip = bi.GetCenterOfMassPosition(e.ragdoll->GetBodyID(PELVIS));
    const auto feet = (bi.GetCenterOfMassPosition(e.ragdoll->GetBodyID(RFOOT)) +
                       bi.GetCenterOfMassPosition(e.ragdoll->GetBodyID(LFOOT))) * .5f;
    JPH::Vec3 forward = bi.GetRotation(cabinID) * JPH::Vec3::sAxisZ();
    if ((feet - hip).Dot(forward) < 0) forward = -forward;
    float clearance = 0;
    int crossed = 0;
    bool centreBehindFace = false;
    for (int part : {PELVIS, ABDOMEN, CHESTLOW}) {
        const auto id = e.ragdoll->GetBodyID(part);
        const auto centre = bi.GetCenterOfMassPosition(id);
        const auto toLocal = bi.GetRotation(id).Conjugated();
        const auto half = bi.GetShape(id)->GetLocalBounds().GetExtent();
        const float radius = (toLocal * forward).Abs().Dot(half);
        const float reach = radius + .15f; // allow for the thickness of the backrest
        JPH::RRayCast ray{centre + forward * reach, -forward * (reach + radius)};
        JPH::RayCastResult hit;
        if (!cabin.CastRay(ray, hit)) continue;
        const auto face = ray.GetPointOnRay(hit.mFraction);
        if (std::abs(cabin.GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, face).Dot(forward)) < .8f) continue;
        // From behind, the backrest must also reach the trunk. A surface wholly
        // in front of the body (e.g. the dashboard) is not a seating correction.
        JPH::RRayCast reverse{centre - forward * reach, forward * (reach + radius)};
        JPH::RayCastResult back;
        if (!cabin.CastRay(reverse, back)) continue;
        const auto rear = reverse.GetPointOnRay(back.mFraction);
        if (std::abs(cabin.GetWorldSpaceSurfaceNormal(back.mSubShapeID2, rear).Dot(forward)) < .8f ||
            (face - rear).Dot(forward) < -.001f) continue;
        const float distance = (face - centre).Dot(forward);
        const float overlap = distance + radius;
        if (overlap <= 0) continue;
        ++crossed;
        centreBehindFace = centreBehindFace || distance > 0;
        clearance = std::max(clearance, overlap);
    }
    if (crossed < 2 || !centreBehindFace) return;
    const auto shift = forward * clearance;
    const auto up = bi.GetRotation(cabinID) * JPH::Vec3::sAxisY();
    FitSeatedLeg(r, e, cabin, RTHIGH, RSHIN, RFOOT, shift, forward, up);
    FitSeatedLeg(r, e, cabin, LTHIGH, LSHIN, LFOOT, shift, forward, up);
    for (int part = 0; part < RTHIGH; ++part) {
        const auto id = e.ragdoll->GetBodyID(part);
        bi.SetPosition(id, bi.GetPosition(id) + shift, JPH::EActivation::Activate);
    }
    RLOG("seated trunk fitted %.3f m forward of its backrest (%d intersections)", clearance, crossed);
}

// The pelvis held where it sits: 5 cm of play each way and 15 degrees about every axis (a body on a seat with
// nothing to hold it but friction and the seat's shape; the cabin's triangles do the rest).
void Seat(const Ragdoll *r, Entry &e) {
    JPH::BodyID cabin = jolt_world::CabinBody(e.car);
    if (cabin.IsInvalid() || e.seat) return;
    JPH::BodyInterface &bi = jolt_world::Bodies();
    JPH::BodyID pelvis = e.ragdoll->GetBodyID(PELVIS);
    JPH::SixDOFConstraintSettings s;
    if (e.seatAnchor) {
        const auto &saved = *static_cast<const JPH::SixDOFConstraintSettings *>(e.seatAnchor.GetPtr());
        s.mSpace = saved.mSpace;
        s.mPosition1 = saved.mPosition1 - bi.GetShape(cabin)->GetCenterOfMass();
        s.mPosition2 = saved.mPosition2;
        s.mAxisX1 = saved.mAxisX1;
        s.mAxisY1 = saved.mAxisY1;
        s.mAxisX2 = saved.mAxisX2;
        s.mAxisY2 = saved.mAxisY2;
    } else {
        FitSeatBack(r, e, cabin);
        s.mSpace = JPH::EConstraintSpace::WorldSpace;
        s.mPosition1 = s.mPosition2 = bi.GetPosition(pelvis);
        s.mAxisX1 = s.mAxisX2 = JPH::Vec3::sAxisX();
        s.mAxisY1 = s.mAxisY2 = JPH::Vec3::sAxisY();
    }
    // Soft in translation: a spring past the play, so a body pressed between the seat and the dashboard (both of
    // them as heavy as the car) yields instead of its joints being torn apart.
    for (JPH::SixDOFConstraintSettings::EAxis a : {JPH::SixDOFConstraintSettings::EAxis::TranslationX, JPH::SixDOFConstraintSettings::EAxis::TranslationY,
                                                  JPH::SixDOFConstraintSettings::EAxis::TranslationZ}) {
        s.SetLimitedAxis(a, -0.05f, 0.05f);
        s.mLimitsSpringSettings[a] = JPH::SpringSettings(JPH::ESpringMode::FrequencyAndDamping, 3.0f, 1.0f);
    }
    for (JPH::SixDOFConstraintSettings::EAxis a : {JPH::SixDOFConstraintSettings::EAxis::RotationX, JPH::SixDOFConstraintSettings::EAxis::RotationY,
                                                  JPH::SixDOFConstraintSettings::EAxis::RotationZ})
        s.SetLimitedAxis(a, -0.26f, 0.26f);
    JPH::TwoBodyConstraint *c = bi.CreateConstraint(&s, cabin, pelvis);
    if (!c) return;
    jolt_world::System()->AddConstraint(c);
    e.seat = c;
    e.seatAnchor = nullptr;
}

} // namespace

void Init(LogFn log) {
    g_log = log;
    g_inOrder = tunables::InOrder(g_tune);
    if (!g_inOrder) RLOG("jolt ragdoll: the tunables table is out of order: Jolt ragdolls are off");
}

bool Enabled() { return g_inOrder; }

bool Has(const Ragdoll *r) {
    auto it = g_entries.find(r);
    return it != g_entries.end() && it->second.serial == r->serial;
}

bool WaitingForGround(const Ragdoll *r) {
    auto it=g_entries.find(r);
    return it!=g_entries.end() && it->second.serial==r->serial && it->second.holding;
}

uint32_t CollisionGroupOf(const Ragdoll *r) {
    auto it=g_entries.find(r);
    if (it==g_entries.end() || it->second.serial!=r->serial) return JPH::CollisionGroup::cInvalidGroup;
    return jolt_world::Bodies().GetCollisionGroup(it->second.ragdoll->GetBodyID(0)).GetGroupID();
}

bool OwnsBody(uint32_t body) {
    for (const auto &kv : g_entries) {
        const JPH::Ragdoll *rag = kv.second.ragdoll;
        if (!rag) continue;
        for (int b = 0; b < (int)rag->GetBodyCount(); b++)
            if (rag->GetBodyID(b).GetIndexAndSequenceNumber() == body) return true;
    }
    return false;
}

// The box whose static world a body needs before it moves: 3 m round its pelvis (lo, or hi).
V3 GroundBox(const Ragdoll *r, bool hi = false) {
    V3 c = JoltBridge::Origin(r) + JoltBridge::B(r, PELVIS).x;
    return hi ? c + V3{3, 3, 3} : c - V3{3, 3, 3};
}

// A body born inside a car's hull leaves the pass-through mode: it meets the car's hull again.
void EndPass(Entry &e) {
    if (!e.passCar) return;
    if (e.ragdoll && jolt_world::Running())
        for (int b = 0; b < BODIES; b++) jolt_world::SetPassThrough(e.ragdoll->GetBodyID(b), nullptr);
    e.passCar = nullptr;
    e.passClear = 0;
}

// Keep collisions with the cabin and the street while the body crosses the outer hull.
static void ReleaseSeat(Entry &e) {
    if (!e.car) return;
    e.passCar = e.car;
    for (int b = 0; b < BODIES; b++) jolt_world::SetPassThrough(e.ragdoll->GetBodyID(b), e.car);
    jolt_world::RequestCabin(e.car);
    Unseat(e);
}

void FinishPull(Ragdoll *r) {
    auto it = g_entries.find(r);
    if (it == g_entries.end()) return;
    ReleaseSeat(it->second); // The engine also vacates the seat if the player is interrupted.
    ReleaseGrip(it->second, r, false);
    jolt_world::ReleaseHolder(r);
}

void ForgetCarrier(const void *car) {
    for (auto &[body, entry] : g_entries) {
        if (entry.passCar == car) EndPass(entry);
        if (entry.car != car) continue;
        ReleaseGrip(entry, body);
        Unseat(entry);
    }
}

float PullContacts(const Ragdoll *r, V3 (&contacts)[2], const char **part) {
    const auto it = g_entries.find(r);
    if (it == g_entries.end() || it->second.reachBlend <= 0 || it->second.gripPart < 0) return 0;
    const Entry &e = it->second;
    if (part) *part = kName[e.gripPart];
    const auto transform = jolt_world::Bodies().GetCenterOfMassTransform(e.ragdoll->GetBodyID(e.gripPart));
    for (int i = 0; i < 2; ++i)
        contacts[i] = jolt_world::ToWorld(F(transform * J(e.localContact[i])));
    return e.reachBlend * e.reachBlend * (3.f - 2.f * e.reachBlend);
}

struct GraspSite {
    int part = -1;
    V3 local[2]{};
    float cost = 1e9f;
};

static GraspSite FindGrasp(const Entry &e, const V3 *pose, V3 hands) {
    auto &bi = jolt_world::Bodies();
    const V3 shoulders[] = {pose[Ragdoll::LARM], pose[Ragdoll::RARM]};
    const V3 across = Norm(shoulders[1] - shoulders[0]);
    const V3 pelvis = jolt_world::ToWorld(F(bi.GetCenterOfMassPosition(e.ragdoll->GetBodyID(PELVIS))));
    GraspSite best;
    // Load-bearing surfaces only: the ribcage, waist, hips and proximal thighs.
    // Head, neck, hands and other narrow distal joints are never candidates.
    for (int part : {CHEST, CHESTLOW, ABDOMEN, PELVIS, LTHIGH, RTHIGH}) {
        const auto body = e.ragdoll->GetBodyID(part);
        const auto shape = bi.GetTransformedShape(body);
        const auto transform = bi.GetCenterOfMassTransform(body);
        const V3 centre = jolt_world::ToWorld(F(transform.GetTranslation()));
        if (Len(hands - centre) >= .95f || Dot(hands - pose[Ragdoll::PELVIS], centre - pose[Ragdoll::PELVIS]) <= 0) continue;
        GraspSite site;
        V3 contact[2];
        const bool thigh = part == LTHIGH || part == RTHIGH;
        const V3 aimCentre = thigh ? centre + (pelvis - centre) * .45f : centre;
        for (int i = 0; i < 2; ++i) {
            const auto start = J(jolt_world::ToJolt(shoulders[i]));
            const auto end = J(jolt_world::ToJolt(aimCentre + across * (i ? .075f : -.075f)));
            JPH::RRayCast ray{start, JPH::Vec3(end - start)};
            JPH::RayCastResult hit;
            if (!shape.CastRay(ray, hit)) break;
            const auto point = ray.GetPointOnRay(hit.mFraction);
            contact[i] = jolt_world::ToWorld(F(point));
            site.local[i] = F(transform.InversedRotationTranslation() * point);
            if (i == 1) site.part = part;
        }
        if (site.part < 0) continue;
        if (Len(contact[1] - contact[0]) < .065f) continue; // room for both hands
        if (thigh && Dot((contact[0]+contact[1])*.5f - centre, pelvis-centre) < 0) continue;
        const auto mid = (contact[0] + contact[1]) * .5f;
        const PullReach reach(pose, contact, 1);
        // Prefer an exposed, elevated grasp. A trunk lying across the seat
        // presents its hips first; a seated upright driver presents the ribs.
        site.cost = 2 * reach.gap + .25f * Len(mid - hands) - Dot(mid - pelvis, e.outward)
                  - .60f * (mid.y - pelvis.y);
        if (thigh) site.cost += .10f; // prefer support nearer the centre of mass at equal reach
        if (part == e.gripPart) site.cost -= .07f; // keep a purchase while the differences are small
        if (site.cost < best.cost) best = site;
    }
    return best;
}

void PullFromSeat(Ragdoll *r, const void *holder, const V3 *pose, float dt) {
    auto it = g_entries.find(r);
    if (it == g_entries.end() || dt <= 0) return;
    Entry &e = it->second;
    if (!e.car && !e.grip) {
        e.reachBlend = 0; // the independent arm recovery no longer samples the released body
        return;
    }
    const V3 left = pose[Ragdoll::LHAND], right = pose[Ragdoll::RHAND], player = pose[Ragdoll::PELVIS];

    JPH::BodyInterface &bi = jolt_world::Bodies();

    const V3 hands = (left + right) * .5f;
    const V3 shoulderLine = pose[Ragdoll::RARM] - pose[Ragdoll::LARM];
    // Measure the actual reach and return across the door in the car's frame.
    // Vertical and fore-aft movement of the hands also occurs while reaching in.
    const void *car = e.car ? e.car : e.passCar;
    V3 origin{}, axes[3];
    float depth = e.lastDepth + Dot(hands - e.lastHands, e.outward);
    if (car && vehicles::Pose(const_cast<void *>(car), &origin, axes)) {
        V3 side = axes[0];
        if (Dot(side, e.reached ? e.outward : player - origin) < 0) side = side * -1.f;
        e.outward = side;
        depth = Dot(hands - origin, side);
    } else if (!e.reached) {
        return;
    }
    const float outwardSpeed = (depth - e.lastDepth) / dt;
    e.lastDepth = depth;
    if (!e.grip) {
        if (!e.reached || depth > e.reachStart) {
            e.reached = true;
            e.reachStart = e.deepestReach = depth;
        }
        e.deepestReach = std::min(e.deepestReach, depth);
        e.lastHands = hands;
        const GraspSite site = FindGrasp(e, pose, hands);
        if (site.part < 0) { e.atContact = false; return; }
        const auto grabbed = e.ragdoll->GetBodyID(site.part);
        const auto transform = bi.GetCenterOfMassTransform(grabbed);
        const bool switching = e.gripPart != site.part;
        V3 points[2]{};
        const bool previousTarget = PullContacts(r, points) > 0;
        bool onSurface = true;
        for (int i = 0; i < 2; ++i) {
            if (!previousTarget) e.localContact[i] = site.local[i];
            else if (switching) e.localContact[i] = F(transform.InversedRotationTranslation() * J(jolt_world::ToJolt(points[i])));
            const V3 delta = site.local[i] - e.localContact[i];
            const float step = dt * 1.4f;
            onSurface &= Len(delta) <= step;
            e.localContact[i] += delta * (step / std::max(step, Len(delta)));
            points[i] = jolt_world::ToWorld(F(transform * J(e.localContact[i])));
        }
        e.gripPart = site.part;
        const float progress = std::clamp((e.reachStart - e.deepestReach) / .12f, 0.f, 1.f);
        // The chest can first become reachable halfway into the native stroke.
        // Bring both arms off their animation continuously from that instant.
        e.reachBlend = std::min(progress, e.reachBlend + dt / .46f);
        const bool touched = onSurface && e.reachBlend >= 1 && PullReach(pose, points, 1).gap <= .012f;
        const bool established = touched && e.atContact && !switching;
        e.atContact = touched;
        if (!established || depth - e.deepestReach < .015f || outwardSpeed <= 0) return;
        const auto contact = J(jolt_world::ToJolt((points[0] + points[1]) * .5f));
        JPH::PointConstraintSettings settings;
        settings.mSpace = JPH::EConstraintSpace::WorldSpace;
        settings.mPoint1 = settings.mPoint2 = contact;
        // The hands keep their purchase when several limbs press against the sill.
        settings.mConstraintPriority = 1;
        settings.mNumPositionStepsOverride = 8;
        JPH::BodyCreationSettings anchor(new JPH::EmptyShape, settings.mPoint1, JPH::Quat::sIdentity(),
                                         JPH::EMotionType::Kinematic, jolt_world::kKinematic);
        e.handAnchor = bi.CreateAndAddBody(anchor, JPH::EActivation::Activate);
        if (e.handAnchor.IsInvalid()) return;
        auto *joint = bi.CreateConstraint(&settings, e.handAnchor, grabbed);
        if (!joint) { ReleaseGrip(e, r); return; }
        ReleaseSeat(e);
        e.grip = static_cast<JPH::PointConstraint *>(joint);
        jolt_world::System()->AddConstraint(e.grip);
        jolt_world::SetHolder(r, holder);
        e.reachOffset = jolt_world::ToWorld(F(contact)) - hands;
        e.anchorTarget = hands + e.reachOffset;
        e.pullVelocity = F(bi.GetLinearVelocity(grabbed));
        e.gripStart = e.farthestPull = depth;
        e.gripAxis = Norm(shoulderLine);
        e.localGripAxis = F(bi.GetRotation(grabbed).Conjugated() * J(e.gripAxis));
        return; // the first held tick keeps the anchor exactly on the original surface
    } else if (Len(hands - e.lastHands) > 1.5f) {
        ReleaseGrip(e, r); // the player was teleported across the scene
        return;
    }

    // The returning hands support the driver's weight while their outward
    // momentum dissipates. Release as they start separating from the driver.
    e.farthestPull = std::max(e.farthestPull, depth);
    const V3 hipVelocity = F(bi.GetLinearVelocity(e.ragdoll->GetBodyID(PELVIS)));
    const V3 handVelocity = (hands - e.lastHands) * (1.f / dt);
    const float hipSpeed = outwardSpeed + Dot(hipVelocity - handVelocity, e.outward);
    V3 contacts[2];
    PullContacts(r, contacts);
    if (e.farthestPull - e.gripStart > .45f &&
        ((outwardSpeed <= 0 && hipSpeed <= 0) || e.farthestPull - depth >= .025f ||
         PullReach(pose, contacts, 1).gap > .016f)) {
        ReleaseGrip(e, r, false); // ignore contact with Tommy until his native action actually finishes
        return;
    }
    e.lastHands = hands;
    // Once Tommy turns away and the hands swing past his shoulders, blend back
    // to the native release height. Preserve the lift while still in the cabin.
    const V3 shoulders = (pose[Ragdoll::LARM] + pose[Ragdoll::RARM]) * .5f;
    const float shoulderWidth = std::max(.1f, Len(shoulderLine));
    const float turn = std::clamp(.5f + Dot(hands - shoulders, e.outward) / (2 * shoulderWidth), 0.f, 1.f);
    const float lower = turn * turn * (3.f - 2.f * turn);
    V3 target = hands + e.reachOffset;
    target.y -= e.reachOffset.y * lower;
    const V3 midpoint = (contacts[0] + contacts[1]) * .5f;
    // Read the underside of the cabin ahead of the head. A high grip must
    // lower through a low header before the physical head hits its edge.
    const auto cabinId = jolt_world::CabinBody(car);
    float required = 0;
    if (!cabinId.IsInvalid()) {
        const auto cabin = bi.GetTransformedShape(cabinId);
        const auto head = bi.GetCenterOfMassPosition(e.ragdoll->GetBodyID(HEAD));
        JPH::RayCastSettings settings;
        settings.SetBackFaceMode(JPH::EBackFaceMode::CollideWithBackFaces);
        for (float lead : {.04f, .18f, .32f}) {
            const auto from = head + J(e.outward * lead) - JPH::Vec3(0, .08f, 0);
            JPH::RRayCast ray{from, JPH::Vec3(0, .38f, 0)};
            JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> hit;
            cabin.CastRay(ray, settings, hit);
            if (!hit.HadHit()) continue;
            const auto point = ray.GetPointOnRay(hit.mHit.mFraction);
            const float slope = std::abs(cabin.GetWorldSpaceSurfaceNormal(hit.mHit.mSubShapeID2, point).GetY());
            if (slope < .6f) continue;
            const float headroom = point.GetY() - head.GetY();
            required = std::max(required, target.y - midpoint.y + .14f / slope - headroom);
        }
    }
    e.duck += std::clamp(required - e.duck, -dt * .45f, dt * 1.2f);
    target.y -= e.duck;
    const int arms[] = {Ragdoll::LARM, Ragdoll::RARM};
    const int elbows[] = {Ragdoll::LELB, Ragdoll::RELB};
    const int wrists[] = {Ragdoll::LHAND, Ragdoll::RHAND};
    // Each material contact has its own reachable sphere. Limit the physics
    // goal too, otherwise the chest can leave one drawn hand at full extension.
    V3 centre[2];
    float radius[2];
    for (int i = 0; i < 2; ++i) {
        centre[i] = pose[arms[i]] - (contacts[i] - midpoint);
        radius[i] = .94f * (Len(pose[elbows[i]] - pose[arms[i]]) + Len(pose[wrists[i]] - pose[elbows[i]]));
    }
    auto withinReach = [&](V3 point) {
        for (int pass = 0; pass < 3; ++pass) {
            for (int i = 0; i < 2; ++i) {
                const V3 delta = point - centre[i];
                if (Len(delta) > radius[i]) point = centre[i] + Norm(delta) * radius[i];
            }
        }
        return point;
    };
    target = withinReach(target);
    // The material grip stays rigid. Give the pulling arms finite acceleration
    // when the stock motion or the required cabin clearance changes abruptly.
    // As Tommy turns out of the doorway, follow his braking motion closely
    // again so that the torso does not continue accelerating at release.
    const float response = 24.f + 60.f * lower;
    const V3 error = e.anchorTarget - target;
    const V3 c = e.pullVelocity + error * response;
    const float decay = std::exp(-response * dt);
    target = target + (error + c * dt) * decay;
    // Limit the actual anchor after reach projection. Projecting it again
    // after filtering used to drag a distant torso 42 cm in a single frame.
    V3 desiredVelocity = (target - e.anchorTarget) * (1.f / dt);
    desiredVelocity = desiredVelocity * (2.7f / std::max(2.7f, Len(desiredVelocity)));
    const V3 change = desiredVelocity - e.pullVelocity;
    const float step = 14.f * dt;
    const V3 velocity = e.pullVelocity + change * (step / std::max(step, Len(change)));
    target = e.anchorTarget + velocity * dt;
    // Slide the grip along the seat back or door header when either the ribs
    // or the head is obstructed. The solver can still rotate the body around
    // that contact; the anchor no longer drives straight through the panel.
    if (!cabinId.IsInvalid()) {
        const auto cabin = bi.GetTransformedShape(cabinId);
        JPH::ShapeCastSettings settings;
        settings.SetBackFaceMode(JPH::EBackFaceMode::CollideWithBackFaces);
        for (int part : {e.gripPart, static_cast<int>(HEAD)}) {
            const auto id = e.ragdoll->GetBodyID(part);
            const V3 motion = target - midpoint;
            if (Len(motion) < .001f) break;
            JPH::RShapeCast cast(bi.GetShape(id), JPH::Vec3::sOne(), bi.GetCenterOfMassTransform(id), J(motion));
            JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> hit;
            cabin.CastShape(cast, settings, JPH::RVec3::sZero(), hit);
            if (hit.HadHit()) {
                const V3 into = Norm(F(hit.mHit.mPenetrationAxis));
                const float blocked = std::max(0.f, Dot(motion, into)) * (1.f - hit.mHit.mFraction);
                target = target - into * blocked;
            }
        }
    }
    e.pullVelocity = (target - e.anchorTarget) * (1.f / dt);
    e.anchorTarget = target;
    bi.MoveKinematic(e.handAnchor, J(jolt_world::ToJolt(target)), JPH::Quat::sIdentity(), dt);
    const auto held = e.grip->GetBody2()->GetID();
    // Two hands can turn the trunk as a pair. Resist angular slip perpendicular
    // to the line between them, leaving rotation about that line free.
    const V3 axis = Norm(shoulderLine, e.gripAxis);
    const float span = Len(contacts[1] - contacts[0]);
    if (span > .02f) {
        const V3 handSpin = Cross(e.gripAxis, axis) * (1.f / dt);
        const V3 slip = handSpin - F(bi.GetAngularVelocity(held));
        const V3 transverse = slip - axis * Dot(slip, axis);
        // Two compliant hand contacts. The material line between the grasp
        // points rotates with the trunk. Its displacement supplies restoring
        // torque even after the hands stop turning. Rotation around it is free.
        const V3 materialAxis = F(bi.GetRotation(held) * J(e.localGripAxis));
        const V3 torque = (Cross(materialAxis, axis) * 350.f + transverse * 50.f) * (span * span);
        const float limit = 80.f * span;
        const float scale = limit / std::max(limit, Len(torque));
        bi.AddAngularImpulse(held, J(torque * (dt * scale)));
    }
    e.gripAxis = axis;
    bi.ActivateBody(held);
}

const void *SeatedCar(const Ragdoll *r) {
    auto it = g_entries.find(r);
    return it == g_entries.end() ? nullptr : it->second.car;
}

bool TransferSeat(Ragdoll *r, void *car) {
    auto it = g_entries.find(r);
    if (it == g_entries.end() || !it->second.car) return false;
    Entry &e = it->second;
    if (e.car == car) return true;
    auto nativePose = [](void *vehicle, JPH::RMat44 &out) {
        V3 p, axes[3];
        if (!vehicles::Pose(vehicle, &p, axes)) return false;
        const JPH::Mat44 turn(JPH::Vec4(J(axes[0]), 0), JPH::Vec4(J(axes[1]), 0),
                              JPH::Vec4(J(axes[2]), 0), JPH::Vec4(0, 0, 0, 1));
        out = JPH::RMat44::sRotationTranslation(turn.GetQuaternion().Normalized(), J(jolt_world::ToJolt(p)));
        return true;
    };
    auto &bi = jolt_world::Bodies();
    JPH::RMat44 from, to;
    const auto source = jolt_world::CabinBody(e.car);
    if (source.IsInvalid()) {
        if (!nativePose(const_cast<void *>(e.car), from)) return false;
    } else {
        from = bi.GetWorldTransform(source);
    }
    if (!nativePose(car, to)) return false;
    if (e.seat) {
        e.seatAnchor = e.seat->GetConstraintSettings();
        // Store the vehicle-local anchor. Different cabin shapes can have different centres of mass.
        auto &saved = *static_cast<JPH::SixDOFConstraintSettings *>(e.seatAnchor.GetPtr());
        saved.mPosition1 += bi.GetShape(source)->GetCenterOfMass();
        jolt_world::System()->RemoveConstraint(e.seat);
        e.seat = nullptr;
    }
    ReleaseGrip(e, r);
    const auto map = to * from.InversedRotationTranslation();
    const auto turn = map.GetQuaternion().Normalized();
    for (int part = 0; part < BODIES; ++part) {
        const auto id = e.ragdoll->GetBodyID(part);
        const auto &b = JoltBridge::B(r, part);
        const auto v = r->velocityEdited ? J(b.v) : e.holding ? e.heldV[part] : bi.GetLinearVelocity(id);
        const auto w = r->velocityEdited ? J(b.w) : e.holding ? e.heldW[part] : bi.GetAngularVelocity(id);
        const auto pose = map * bi.GetWorldTransform(id);
        bi.SetPositionAndRotation(id, pose.GetTranslation(), pose.GetQuaternion(), JPH::EActivation::Activate);
        bi.SetLinearAndAngularVelocity(id, turn * v, turn * w);
        if (e.holding) { e.heldV[part] = turn * v; e.heldW[part] = turn * w; }
    }
    e.car = car;
    r->velocityEdited = false;
    jolt_world::RequestCabin(car, r);
    ReadBack(r, e, 0.f);
    return true;
}

float GripDistance(const Ragdoll *r, V3 *contact) {
    auto it = g_entries.find(r);
    if (it == g_entries.end() || !it->second.grip) return -1.f;
    const Entry &e = it->second;
    const auto transform = jolt_world::Bodies().GetCenterOfMassTransform(e.grip->GetBody2()->GetID());
    const V3 point = jolt_world::ToWorld(F(transform * e.grip->GetLocalSpacePoint2()));
    if (contact) *contact = point;
    return Len(point - e.anchorTarget);
}

int ShotPart(const Ragdoll *r, const V3 &from, const V3 &delta, V3 *point) {
    auto it = g_entries.find(r);
    return it == g_entries.end() ? -1 : articulation::ShotPart(*it->second.ragdoll,from,delta,point);
}

bool CabinBlocksShot(const void *key, const V3 &from, const V3 &point, bool requireCabin) {
    auto id = jolt_world::CabinBody(key);
    if (id.IsInvalid()) return requireCabin;
    JPH::RRayCast ray{J(jolt_world::ToJolt(from)), J(point - from)};
    JPH::RayCastSettings settings;
    settings.SetBackFaceMode(JPH::EBackFaceMode::CollideWithBackFaces);
    JPH::AnyHitCollisionCollector<JPH::CastRayCollector> hits;
    jolt_world::Bodies().GetTransformedShape(id).CastRay(ray, settings, hits);
    return hits.HadHit();
}

void Remove(Ragdoll *r) {
    auto it = g_entries.find(r);
    if (it == g_entries.end()) return;
    ReleaseGrip(it->second, r);
    EndPass(it->second);
    if (it->second.seat) { jolt_world::System()->RemoveConstraint(it->second.seat); it->second.seat = nullptr; }
    if (it->second.ragdoll) it->second.ragdoll->RemoveFromPhysicsSystem(false);
    g_entries.erase(it);
}

void Clear() {
    for (auto &kv : g_entries) {
        ReleaseGrip(kv.second, kv.first);
        EndPass(kv.second);
        if (kv.second.seat && jolt_world::Running()) jolt_world::System()->RemoveConstraint(kv.second.seat);
        kv.second.seat = nullptr;
        if (kv.second.ragdoll) kv.second.ragdoll->RemoveFromPhysicsSystem(false);
    }
    g_entries.clear();
}

bool Add(Ragdoll *r, std::string *err, const void *seatedIn, const void *struckBy) {
    if (Has(r)) return true;
    Remove(r);
    V3 rorigin = JoltBridge::Origin(r);
    if (!jolt_world::Running() && !jolt_world::Start(g_log, rorigin, err)) return false;
    if (struckBy) {
        smoke::Mover vehicle;
        if (!vehicles::MoverOf(const_cast<void *>(struckBy), &vehicle) || !jolt_world::EnsureVehicle(vehicle))
            RLOG("ragdoll %p: striking vehicle %p has no native collision shape yet; no artificial impulse", r->actor, struckBy);
    }
    jolt_world::SetCabinGoneListener(OnCabinGone);
    jolt_world::SetCabinJumpListener(OnCabinJump);
    V3 shift = rorigin - jolt_world::Origin();   // the ragdoll's local frame -> Jolt space
    const V3 *P = JoltBridge::Points(r);         // the start pose (world)
    V3 fwd = JoltBridge::Facing(r);
    auto toJ = [&](const V3 &world) { return (world - rorigin) + shift; };

    // Anatomical frames, as Ragdoll::Setup makes them (x right, y up, z forward).
    using RP = Ragdoll;
    Ragdoll neutral;
    neutral.Setup(r->reference.pts,r->reference.fwd,r->reference.rot);
    const V3 *N = r->reference.pts;
    TrunkAxes axes = TrunkAxes::From(N, r->reference.fwd);
    V3 pr = axes.pelvisRight, pf = axes.pelvisFront;
    V3 pu = Cross(pf, pr);
    V3 cr = axes.chestRight, cf = axes.chestFront;
    V3 cu = Cross(cf, cr);
    auto axisOf = [&](int b, const V3 &local) { return Norm(JoltBridge::Rot(JoltBridge::B(&neutral, b).q, local)); };
    auto dir = [&](int a, int b) { return Norm(N[b] - N[a]); };

    Entry e;
    e.serial = r->serial;
    JPH::Ref<JPH::Skeleton> sk = new JPH::Skeleton;
    for (int b = 0; b < BODIES; b++) sk->AddJoint(kName[b], kParent[b]);
    JPH::Ref<JPH::RagdollSettings> rs = new JPH::RagdollSettings;
    rs->mSkeleton = sk;
    rs->mParts.resize(BODIES);
    JPH::Mat44 joints[BODIES];
    for (int b = 0; b < BODIES; b++) {
        const JoltBridge::Body &bd = JoltBridge::B(r, b);
        JPH::RagdollSettings::Part &p = rs->mParts[b];
        p.SetShape(ShapeOf(r, b, JoltBridge::Rotations(r)[18].r[1]));
        p.mPosition = J(shift + bd.x);
        p.mRotation = J(bd.q);
        p.mMotionType = JPH::EMotionType::Dynamic;
        p.mObjectLayer = seatedIn ? jolt_world::kOccupant : jolt_world::kDynamic;
        p.mFriction = T(kFriction);
        p.mRestitution = 0.0f;
        p.mLinearDamping = T(kLinDamping);
        p.mAngularDamping = T(kAngDamping);
        p.mMotionQuality = T(kCcd) > 0.5f ? JPH::EMotionQuality::LinearCast : JPH::EMotionQuality::Discrete;
        p.mAllowSleeping = true;
        p.mNumVelocityStepsOverride = (unsigned)T(kVelocitySteps);
        p.mNumPositionStepsOverride = (unsigned)T(kPositionSteps);
        p.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        p.mMassPropertiesOverride.mMass = kMass[b];
        p.mLinearVelocity = J(bd.v);
        p.mAngularVelocity = J(bd.w);
        joints[b] = JPH::Mat44::sRotationTranslation(p.mRotation, p.mPosition);
    }
    // The joints, child by child.
    auto ball = [&](int child, int point, const V3 &c, const V3 &f, const Anatomy &a) {
        rs->mParts[child].mToParent = Ball(r, &neutral, kParent[child], child, toJ(P[point]), c, f, a);
        e.anatomy[child] = &a;
    };
    auto hinge = [&](int child, int point, V3 axis, const V3 &fallback, const Anatomy &a) {
        if (Len(axis) < 0.3f) axis = fallback;
        float start;
        rs->mParts[child].mToParent = Hinge(r, &neutral, kParent[child], child, toJ(P[point]), axis, a, &start);
        e.anatomy[child] = &a;
        e.hinge[child] = true;
        const auto &rp=JoltBridge::B(&neutral,kParent[child]), &rc=JoltBridge::B(&neutral,child);
        V3 h=Norm(axis), n=Orth(JoltBridge::Rot(rp.q,{1,0,0}),h);
        e.hingeAxisParent[child] = JoltBridge::InvRot(rp.q,h);
        e.hingeNormalParent[child] = JoltBridge::InvRot(rp.q,n);
        e.hingeNormalChild[child] = JoltBridge::InvRot(rc.q,n);
    };
    // Spine and neck: straight on from the segment below, leaning a little forward.
    ball(ABDOMEN, RP::BACK1, Tilt(pu, pf, 5), pf, kSpineLow);
    ball(CHESTLOW, RP::BACK2, Tilt(axisOf(ABDOMEN, {1, 0, 0}), axisOf(ABDOMEN, {0, 0, 1}), 5), axisOf(ABDOMEN, {0, 0, 1}), kSpineMid);
    ball(CHEST, RP::BACK3, Tilt(axisOf(CHESTLOW, {1, 0, 0}), axisOf(CHESTLOW, {0, 0, 1}), 5), axisOf(CHESTLOW, {0, 0, 1}), kSpineHigh);
    ball(HEAD, RP::NECK, Tilt(axisOf(CHEST, {1, 0, 0}), cf, 10), cf, kNeck);
    // Shoulders: the middle of the arm's reach, out, down and a little forward of the chest.
    for (int s = 0; s < 2; s++) {
        float side = s ? -1.0f : 1.0f;
        int arm = s ? LUARM : RUARM, fore = s ? LFARM : RFARM;
        int ptArm = s ? RP::LARM : RP::RARM, ptElb = s ? RP::LELB : RP::RELB;
        V3 c = Norm(cr * (0.55f * side) - cu * 0.6f + cf * 0.35f);
        ball(arm, ptArm, c, cf, kShoulder);
        // Elbows bend the forearm forward (an arm hanging down), whatever the start pose.
        hinge(fore, ptElb, Cross(dir(ptArm, ptElb), cf), -cr, kElbow);
    }
    // Hips: the thigh's reach is mostly forward (flexion 115, extension 25) and a little out.
    for (int s = 0; s < 2; s++) {
        float side = s ? -1.0f : 1.0f;
        int thigh = s ? LTHIGH : RTHIGH, shin = s ? LSHIN : RSHIN, foot = s ? LFOOT : RFOOT;
        int ptHip = s ? RP::LHIP : RP::RHIP, ptKnee = s ? RP::LKNEE : RP::RKNEE, ptAnkle = s ? RP::LANKLE : RP::RANKLE;
        V3 down = -pu;
        V3 c = Tilt(Tilt(down, pf, 45), pr * side, 8);
        ball(thigh, ptHip, c, pf, kHip);
        // Knees bend the shin backwards.
        hinge(shin, ptKnee, Cross(dir(ptHip, ptKnee), -pf), pr, kKnee);
        // Ankles: the foot forward of the shin, tilted a little down (plantarflexion 50, dorsiflexion 20).
        V3 shinDir = dir(ptKnee, ptAnkle);
        V3 footFwd = Orth(pf, shinDir);
        ball(foot, ptAnkle, Tilt(footFwd, shinDir, 15), -shinDir, kAnkle);
    }
    articulation::Prepare(*rs,joints,.02f);
    JPH::Ragdoll *rag = rs->CreateRagdoll(g_nextGroup++, (JPH::uint64)(uintptr_t)r, jolt_world::System());
    if (!rag) { *err = "Jolt could not make the ragdoll (body or constraint limit)"; return false; }
    rag->AddToPhysicsSystem(JPH::EActivation::Activate, false);
    for (int b = 0; b < BODIES; b++)
        jolt_world::SetFloat(rag->GetBodyID(b), kFloat[b] * T(kBuoyancy), T(kWaterDrag), T(kWaterDrag) * 0.12f);
    e.settings = rs;
    e.ragdoll = rag;
    // The muscles' first target: the pose of the moment (the death, the impact).
    Ragdoll startPose;
    startPose.Setup(P, fwd, JoltBridge::Rotations(r));
    TargetsFrom(startPose, e);
    e.animSeen = r->animSerial;
    e.car = seatedIn;
    if (seatedIn) jolt_world::RequestCabin(seatedIn);
    // Born inside a car's hull (killed in its open door, half in the cabin): the hull would squeeze the body against
    // the street for good. It passes through that car's hull and meets the car's own cabin instead, until no part
    // is inside the hull any more.
    if (!seatedIn) {
        const void *hull = nullptr;
        for (int b = 0; b < BODIES && !hull; b++) hull = jolt_world::HullAround(rag->GetBodyID(b), 0.01f);
        // A car impact necessarily starts at (and may slightly penetrate) the hull. That is not a doorway
        // occupant: disabling the striking hull lets the whole car pass through its victim.
        if (hull && hull != struckBy) {
            e.passCar = hull;
            for (int b = 0; b < BODIES; b++) jolt_world::SetPassThrough(rag->GetBodyID(b), hull);
            jolt_world::RequestCabin(hull);
            RLOG("ragdoll %p starts inside car %p's hull (its open door): it meets the car's cabin, not the hull, until it is out",
                 r->actor, hull);
        }
        if (hull && hull == struckBy) RLOG("ragdoll %p: striking car %p stays solid at initial contact", r->actor, hull);
    }
    // Rail riders have a supplied carriage mesh in the first step. Waiting
    // on street scenery would pin them to world space while the carriage leaves.
    if (!r->railOccupant && !jolt_world::StaticReady(GroundBox(r), GroundBox(r, true))) {
        JPH::BodyInterface &bi = jolt_world::Bodies();
        for (int b = 0; b < BODIES; b++) {
            JPH::BodyID id = rag->GetBodyID(b);
            e.heldV[b] = bi.GetLinearVelocity(id);
            e.heldW[b] = bi.GetAngularVelocity(id);
            bi.DeactivateBody(id);
        }
        e.holding = true;
        RLOG("ragdoll %p waits for the ground: the static world around it is not all there yet (%s)", r->actor,
             jolt_world::ReadyReport(GroundBox(r), GroundBox(r, true)).c_str());
    }
    g_entries[r] = std::move(e);
    return true;
}

void Step(const std::vector<Ragdoll *> &all, float dt, const std::vector<smoke::Mover> &movers, const V3 *player) {
    // Entries of ragdolls that are gone (or whose address a new ragdoll took) leave the world first.
    for (auto it = g_entries.begin(); it != g_entries.end();) {
        const Ragdoll *r = it->first;
        bool live = std::find(all.begin(), all.end(), r) != all.end() && r->serial == it->second.serial;
        if (live) { ++it; continue; }
        ReleaseGrip(it->second, r);
        EndPass(it->second);
        if (it->second.seat) { jolt_world::System()->RemoveConstraint(it->second.seat); it->second.seat = nullptr; }
        if (it->second.ragdoll) it->second.ragdoll->RemoveFromPhysicsSystem(false);
        it = g_entries.erase(it);
    }
    if (!jolt_world::Running()) return;
    jolt_world::SetPersonPush(T(kPushForce), T(kPushMass));

    // The seated: their cars' cabins kept, their seats made once the cabin is there.
    for (auto &[body, entry] : g_entries) {
        if (!entry.car) continue;
        if (!entry.holding && !entry.seat && OutsideCabin(entry)) {
            RLOG("ragdoll %p cleared cabin %p and entered the world", body->actor, entry.car);
            Unseat(entry);
            continue;
        }
        jolt_world::RequestCabin(entry.car, body, body->railOccupant);
    }
    // Bodies passing through a car's hull: its cabin kept while any part is inside the hull; out for half a second
    // (or the car gone from the world): an ordinary body again.
    for (auto &kv : g_entries) {
        Entry &e = kv.second;
        if (!e.passCar) continue;
        bool inside = false;
        if (jolt_world::HasKinematic(e.passCar))
            for (int b = 0; b < BODIES && !inside; b++) inside = jolt_world::HullAround(e.ragdoll->GetBodyID(b), 0.0f, e.passCar) != nullptr;
        e.passClear = inside ? 0.0f : e.passClear + dt;
        if (inside || e.passClear < 0.5f) { jolt_world::RequestCabin(e.passCar); continue; }
        RLOG("ragdoll %p is out of car %p's hull: an ordinary body again", kv.first->actor, e.passCar);
        EndPass(e);
    }
    JPH::BodyInterface &bi = jolt_world::Bodies();
    for (Ragdoll *r : all) {
        auto it = g_entries.find(r);
        if (it == g_entries.end()) continue;
        Entry &e = it->second;
        r->age += dt;
        if (e.holding) {
            e.held += dt;
            bool ready = jolt_world::StaticReady(GroundBox(r), GroundBox(r, true));
            if (r->velocityEdited) {   // a bullet while it waits: kept for when it goes
                for (int b = 0; b < BODIES; b++) {
                    const JoltBridge::Body &bd = JoltBridge::B(r, b);
                    e.heldV[b] = J(bd.v);
                    e.heldW[b] = J(bd.w);
                }
                r->velocityEdited = false;
            }
            if (ready || e.held > 5.0f) {
                for (int b = 0; b < BODIES; b++) bi.SetLinearAndAngularVelocity(e.ragdoll->GetBodyID(b), e.heldV[b], e.heldW[b]);
                e.ragdoll->Activate(false);
                e.holding = false;
                RLOG("ragdoll %p goes after %.2f s: %s%s", r->actor, e.held, ready ? "the ground is there" : "no ground came in 5 s: ",
                     ready ? "" : jolt_world::ReadyReport(GroundBox(r), GroundBox(r, true)).c_str());
            } else {
                for (int b = 0; b < BODIES; b++) bi.DeactivateBody(e.ragdoll->GetBodyID(b));
            }
        }
        if (r->velocityEdited) {   // a bullet, a command: the parts' new velocities go in
            for (int b = 0; b < BODIES; b++) {
                const JoltBridge::Body &bd = JoltBridge::B(r, b);
                bi.SetLinearAndAngularVelocity(e.ragdoll->GetBodyID(b), J(bd.v), J(bd.w));
            }
            e.ragdoll->Activate(false);
            r->velocityEdited = false;
        }
        Muscles(r, e, dt);
        // The static world must reach wherever the body goes.
        V3 c = JoltBridge::Origin(r) + JoltBridge::B(r, PELVIS).x;
        jolt_world::AddFocus(c - V3{12, 8, 12}, c + V3{12, 8, 12});
    }
    jolt_world::Step(dt, movers, player, [] {
        for (auto &[body, entry] : g_entries)
            if (entry.car && !body->railOccupant) Seat(body, entry);
    });
    for (Ragdoll *r : all) {
        auto it = g_entries.find(r);
        if (it != g_entries.end()) ReadBack(r, it->second, dt);
    }
}

void Tunes(std::vector<tunables::Info> &out) { tunables::List(g_tune, out); }

bool SetTune(const char *name, float value, std::string *out) {
    const Tunable *t = tunables::Set(g_tune, name, value);
    if (out) *out = t ? Fmt("jolt: %s = %g (%s)", t->name, value, t->what) : Fmt("jolt: no setting '%s' (`jolt set` lists them)", name);
    return t != nullptr;
}

std::string Describe() {
    unsigned active = 0;
    for (auto &kv : g_entries) active += kv.second.ragdoll && kv.second.ragdoll->IsActive(false);
    return Fmt("jolt ragdolls: %s, %u (%u moving)", Enabled() ? "on" : "off (the settings table is out of order)",
               (unsigned)g_entries.size(), active);   // (the world's own: jolt_world::Describe, beside it in `jolt`)
}

// `jolt body <actor>`: one ragdoll's parts as Jolt has them: speed, spin, and what each part overlaps now (the
// deepest overlap and what it belongs to: a car, the static world, another body). A body that twitches while it
// lies is held inside something.
std::string BodyReport(const std::string &args) {
    unsigned long key = 0;
    sscanf(args.c_str(), "%*s %lx", &key);
    const Entry *e = nullptr;
    const Ragdoll *rd = nullptr;
    for (auto &kv : g_entries)
        if (reinterpret_cast<uintptr_t>(kv.first->actor) == key) { rd = kv.first; e = &kv.second; }
    if (!e || !e->ragdoll) return Fmt("jolt body: no Jolt ragdoll on %08lX", key);
    JPH::PhysicsSystem *ps = jolt_world::System();
    JPH::BodyInterface &bi = jolt_world::Bodies();
    JPH::IgnoreMultipleBodiesFilter own;
    for (int b = 0; b < BODIES; b++) own.IgnoreBody(e->ragdoll->GetBodyID(b));
    std::string s = Fmt("jolt body %p: age %.1f s, %s%s%s", rd->actor, e->age, e->ragdoll->IsActive(false) ? "moving" : "asleep",
                        e->car ? Fmt(", seated in car %p", e->car).c_str() : "",
                        e->passCar ? Fmt(", passing through car %p's hull", e->passCar).c_str() : "");
    for (int b = 0; b < BODIES; b++) {
        JPH::BodyID id = e->ragdoll->GetBodyID(b);
        V3 v = F(bi.GetLinearVelocity(id)), wv = F(bi.GetAngularVelocity(id));
        // the part's shape where it is, against everything but its own body
        JPH::RefConst<JPH::Shape> shape = bi.GetShape(id);
        JPH::RMat44 at = bi.GetCenterOfMassTransform(id);
        JPH::CollideShapeSettings cs;
        cs.mMaxSeparationDistance = 0.02f;   // touching counts (a resting contact has next to no depth)
        JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> hits;
        ps->GetNarrowPhaseQueryNoLock().CollideShape(shape, JPH::Vec3::sReplicate(1.0f), at, cs, JPH::RVec3::sZero(), hits,
                                                     {}, {}, own);
        float deepest = -1;
        std::string what = "-";
        for (const JPH::CollideShapeResult &h : hits.mHits) {
            if (h.mPenetrationDepth <= deepest) continue;
            deepest = h.mPenetrationDepth;
            JPH::ObjectLayer l = bi.GetObjectLayer(h.mBodyID2);
            if (const void *car = jolt_world::KinematicOwner(h.mBodyID2)) {
                // which part of the car, where (world height) and which way it pushes the body out
                JPH::Vec3 n = -h.mPenetrationAxis.NormalizedOr(JPH::Vec3::sAxisY());
                what = Fmt("car %p part %d at height %.2f, normal %.2f %.2f %.2f", car, jolt_world::CarPartOf(h.mBodyID2, h.mSubShapeID2.GetValue()),
                           (float)h.mContactPointOn2.GetY() + jolt_world::Origin().y, n.GetX(), n.GetY(), n.GetZ());
            }
            else if (l == jolt_world::kStatic) what = "static world";
            else if (l == jolt_world::kKinematic) what = "a living person";
            else if (l == jolt_world::kCabin) what = "a car's cabin";
            else what = "another body";
        }
        s += Fmt("\n  %-10s v %5.2f m/s  w %5.1f rad/s  overlaps %u, deepest %.3f m: %s", kName[b], Len(v), Len(wv),
                 (unsigned)hits.mHits.size(), deepest, what.c_str());
    }
    // Every joint against its limits (degrees): swing about the constraint's y (plane half cone) and z (normal half
    // cone), twist about x; a hinge's angle. OVER: outside the limit (a joint turned past it and held there).
    s += "\n  joints:";
    const float d = 57.29578f;
    for (int i = 0; i < (int)e->ragdoll->GetConstraintCount(); i++) {
        const JPH::TwoBodyConstraint *c = e->ragdoll->GetConstraint(i);
        int part = -1;
        for (int b = 0; b < BODIES; b++) if (c->GetBody2()->GetID() == e->ragdoll->GetBodyID(b)) part = b;
        const char *name = part >= 0 ? kName[part] : "?";
        if (c->GetSubType() == JPH::EConstraintSubType::SwingTwist) {
            auto *st = static_cast<const JPH::SwingTwistConstraint *>(c);
            JPH::Quat q = st->GetRotationInConstraintSpace();
            if (q.GetW() < 0) q = -q;
            JPH::Quat swing, twist;
            q.GetSwingTwist(swing, twist);
            float tw = 2.0f * std::atan2(twist.GetX(), twist.GetW()) * d;
            float sy = 2.0f * std::atan2(swing.GetY(), swing.GetW()) * d, sz = 2.0f * std::atan2(swing.GetZ(), swing.GetW()) * d;
            float py = st->GetPlaneHalfConeAngle() * d, pz = st->GetNormalHalfConeAngle() * d;
            float t0 = st->GetTwistMinAngle() * d, t1 = st->GetTwistMaxAngle() * d;
            bool over = std::fabs(sy) > py + 3 || std::fabs(sz) > pz + 3 || tw < t0 - 3 || tw > t1 + 3;
            s += Fmt("\n  %-10s swing y %6.1f (+-%.0f) z %6.1f (+-%.0f) twist %6.1f (%.0f..%.0f)%s", name, sy, py, sz, pz, tw, t0,
                     t1, over ? "  OVER" : "");
        } else if (c->GetSubType() == JPH::EConstraintSubType::Hinge) {
            auto *h = static_cast<const JPH::HingeConstraint *>(c);
            float a = h->GetCurrentAngle() * d, lo = h->GetLimitsMin() * d, hi = h->GetLimitsMax() * d;
            s += Fmt("\n  %-10s hinge %6.1f (%.0f..%.0f)%s", name, a, lo, hi, a < lo - 3 || a > hi + 3 ? "  OVER" : "");
        }
    }
    return s;
}

bool Exec(const std::string &cmd, const std::string &args, std::string *out) {
    if (cmd != "jolt") return false;
    char what[32] = "";
    sscanf(args.c_str(), "%31s", what);
    std::string w = what;
    if (w.empty()) { *out = Describe(); return true; }
    if (w == "set") {
        char name[32] = ""; float value = 0;
        if (sscanf(args.c_str(), "%*s %31s %f", name, &value) == 2) { SetTune(name, value, out); return true; }
        *out = "jolt settings:";
        for (const Tunable &t : g_tune) *out += Fmt("\n  %s = %g: %s", t.name, t.value, t.what);
        return true;
    }
    if (w == "body") { *out = BodyReport(args); return true; }
    if (w == "car") {
        unsigned long key = 0;
        sscanf(args.c_str(), "%*s %lx", &key);
        *out = jolt_world::DescribeCar(reinterpret_cast<const void *>(key));
        return true;
    }
    *out = "usage: jolt | jolt set [name value] | jolt body <actor hex>";
    return true;
}

}
