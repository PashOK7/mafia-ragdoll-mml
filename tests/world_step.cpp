// Production scenery and Jolt step, with two instances of one mesh supplied by the SDK.
#include "mod.h"
#include "scenery.h"
#include "headwear.h"
#include "jolt_world.h"
#include "joint_resistance.h"
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mod {
const MmlHost *H;
MmlMod *M;
void Log(const char *, ...) {}
}

// The standalone world fixture has no character models or headwear.
namespace headwear {
bool OwnsBody(uint32_t) { return false; }
bool OwnsVisual(void *) { return false; }
bool ReleasingFrom(uint32_t,uint32_t) { return false; }
void BodyContact(uint32_t) {}
void FlightContact(uint32_t,const JPH::BodyID &,unsigned,V3,V3,float) {}
}

// A supported torso with a freely hanging 6 kg head. Give the head one
// transverse impulse. Measure its angle from vertical with sleeping disabled.
static bool SwingTest(bool hinge, float scale, float dt) {
    using namespace JPH;
    const RVec3 pivot(8, 12, 0);
    BodyCreationSettings settings(new SphereShape(.10f), pivot + Vec3(0, -.10f, 0),
                                  Quat::sIdentity(), EMotionType::Dynamic, jolt_world::kDynamic);
    settings.mAllowSleeping = false;
    settings.mAngularDamping = .5f;
    settings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
    settings.mMassPropertiesOverride.mMass = 6.f;
    auto &bodies = jolt_world::Bodies();
    auto *head = bodies.CreateBody(settings);
    bodies.AddBody(head->GetID(), EActivation::Activate);
    HingeConstraintSettings elbow;
    elbow.mPoint1 = elbow.mPoint2 = pivot;
    elbow.mHingeAxis1 = elbow.mHingeAxis2 = Vec3::sAxisX();
    elbow.mNormalAxis1 = elbow.mNormalAxis2 = Vec3::sAxisY();
    SwingTwistConstraintSettings neck;
    neck.mPosition1 = neck.mPosition2 = pivot;
    neck.mTwistAxis1 = neck.mTwistAxis2 = Vec3::sAxisY();
    neck.mPlaneAxis1 = neck.mPlaneAxis2 = Vec3::sAxisX();
    neck.mNormalHalfConeAngle = DegreesToRadians(50.f);
    neck.mPlaneHalfConeAngle = DegreesToRadians(40.f);
    Ref<TwoBodyConstraint> joint = hinge ? elbow.Create(Body::sFixedToWorld, *head)
                                         : neck.Create(Body::sFixedToWorld, *head);
    jolt_world::System()->AddConstraint(joint);
    bodies.AddImpulse(head->GetID(), Vec3(0, 0, 2.f));
    float first = 0, late = 0;
    for (int i = 0; i < std::lround(4.f / dt); ++i) {
        jolt_ragdoll::SetPassiveResistance(*joint, jolt_ragdoll::neckResistance, scale);
        jolt_world::Step(dt, {}, nullptr);
        const auto offset = head->GetCenterOfMassPosition() - pivot;
        const float angle = std::abs(RadiansToDegrees(std::atan2(offset.GetZ(), -offset.GetY())));
        if (i * dt < .8f) first = std::max(first, angle);
        if (i * dt >= 3.f) late = std::max(late, angle);
    }
    jolt_world::System()->RemoveConstraint(joint);
    bodies.RemoveBody(head->GetID());
    bodies.DestroyBody(head->GetID());
    std::printf("%s %.0f Hz, scale %.0f: peak %.3f deg, after 3 s %.3f deg\n",
                hinge ? "hinge" : "neck", 1.f / dt, scale, first, late);
    return first > 2.f && (scale == 0 ? late > 3.f : late < .5f);
}

static bool CabinPanelTest(bool solid, bool wreck = false) {
    using namespace JPH;
    std::vector<V3> surfaces{{-1,0,-6},{1,0,0},{1,0,-6}};
    std::vector<V3> wall{
        {-1,0,-5},{.25f,2,-5},{.25f,0,-5},
        {-1,0,-5},{-1,2,-5},{.25f,2,-5},
    };
    smoke::Mover cabin;
    cabin.kind = smoke::Mover::HULL;
    cabin.key = &cabin;
    cabin.c = {0,10,0};
    cabin.ax[0] = {1,0,0};
    cabin.ax[1] = {0,1,0};
    cabin.ax[2] = {0,0,1};
    cabin.hull = &surfaces;
    cabin.hullIsTriangles = cabin.cabinOnly = true;
    auto turn = [wreck](V3 v) { return wreck ? V3{-v.z, v.y, v.x} : v; };
    for (V3 &v : surfaces) v = turn(v);
    for (V3 &v : wall) v = turn(v);
    const smoke::CabinInterior interior{turn({0, 1, -4.4f}), turn({0, .8f, -4.4f}),
                                        turn({0, .6f, -4.4f}), turn({0, .4f, -4.4f})};
    if (solid && wreck) cabin.wreckInterior = &interior;
    if (solid && !wreck) cabin.cabinPanels = &wall;
    else surfaces.insert(surfaces.end(),wall.begin(),wall.end());
    auto &bodies = jolt_world::Bodies();
    auto point = [&](V3 v) { v = turn(v); return RVec3(v.x, v.y + 10, v.z); };
    BodyCreationSettings limb(new SphereShape(.12f), point({-.55f,1,-5.035f}),
        Quat::sIdentity(), EMotionType::Dynamic, jolt_world::kOccupant);
    limb.mGravityFactor = 0;
    limb.mMotionQuality = EMotionQuality::LinearCast;
    limb.mAllowSleeping = false;
    limb.mUserData = reinterpret_cast<uintptr_t>(&limb);
    const auto caught = bodies.CreateAndAddBody(limb, EActivation::Activate);
    limb.mPosition = point({.65f,1,-4.75f});
    limb.mLinearVelocity = wreck ? Vec3(4,0,0) : Vec3(0,0,-4);
    const auto free = bodies.CreateAndAddBody(limb, EActivation::Activate);
    for (int i=0;i<40;++i) {
        jolt_world::RequestCabin(&cabin,&limb,!wreck);
        jolt_world::Step(1.f/120.f,{cabin},nullptr);
    }
    auto depth = [&](BodyID id) {
        const auto p = bodies.GetPosition(id);
        return wreck ? -p.GetX() : p.GetZ();
    };
    const float recovered = depth(caught), exited = depth(free);
    for (auto id : {caught,free}) {
        bodies.RemoveBody(id);
        bodies.DestroyBody(id);
    }
    jolt_world::Step(1.f/120.f,{},nullptr);
    std::printf("%s %s settled at %.3f m; opening crossed to %.3f m\n",
                wreck ? "Wreck" : "Rail", solid ? "solid panel" : "two-sided mesh", recovered, exited);
    return (solid ? recovered > -4.93f : recovered < -5.f) && exited < -5.5f;
}

static bool SharedBarrierTest() {
    using namespace JPH;
    const RefConst<Shape> shapes[] = {
        new BoxShape(Vec3(.16f,.035f,.17f),.015f),
        new CapsuleShape(.12f,.085f),
        new CapsuleShape(.10f,.11f),
    };
    const ObjectLayer layers[]{jolt_world::kDynamic,jolt_world::kDynamic,jolt_world::kOccupant};
    BodyID bodiesUnderTest[3];
    auto &bodies = jolt_world::Bodies();
    for (int i=0;i<3;++i) {
        BodyCreationSettings s(shapes[i], RVec3(2,10,.8f+1.2f*i), Quat::sIdentity(),
                               EMotionType::Dynamic,layers[i]);
        s.mGravityFactor = 0;
        s.mLinearVelocity = Vec3(5,0,0);
        s.mMotionQuality = EMotionQuality::LinearCast;
        bodiesUnderTest[i] = bodies.CreateAndAddBody(s,EActivation::Activate);
    }
    for (int i=0;i<120;++i) jolt_world::Step(1.f/120.f,{},nullptr);
    bool valid = true;
    for (auto id : bodiesUnderTest) {
        const float x = bodies.GetPosition(id).GetX();
        std::printf("Shared barrier test: body stopped at x=%.3f m\n",x);
        valid &= x>4.6f && x<5.f;
        bodies.RemoveBody(id);
        bodies.DestroyBody(id);
    }
    return valid;
}

// An ordinary off-centre bumper contact against a body resting on the ground.
static bool GroundedBodyCarTest() {
    using namespace JPH;
    auto &bodies = jolt_world::Bodies();
    BodyCreationSettings floor(new BoxShape(Vec3(2.f,.1f,3.f)), RVec3(12,9.13f,1.7f),
        Quat::sIdentity(), EMotionType::Static, jolt_world::kStatic);
    floor.mFriction = .9f;
    const auto ground = bodies.CreateAndAddBody(floor, EActivation::DontActivate);
    BodyCreationSettings segment(new SphereShape(.24f), RVec3(12.65f,9.47f,1.78f),
        Quat::sIdentity(), EMotionType::Dynamic, jolt_world::kDynamic);
    segment.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
    segment.mMassPropertiesOverride.mMass = 28.f;
    segment.mFriction = .9f;
    const auto body = bodies.CreateAndAddBody(segment, EActivation::Activate);
    smoke::Mover car;
    car.kind = smoke::Mover::BOX; car.key = &car; car.mass = 950;
    car.h = {.8f,.48f,1.35f}; car.c = {12,10,0};
    car.ax[0] = {1,0,0}; car.ax[1] = {0,1,0}; car.ax[2] = {0,0,1};
    constexpr float dt = 1.f / 120;
    jolt_world::Step(dt, {car}, nullptr);
    float lift = 0, turn = 0;
    for (int i = 0; i < 12; ++i) {
        car.c.z += 8.f * dt;
        jolt_world::Step(dt, {car}, nullptr);
        for (const auto &kick : jolt_world::CarKicks()) {
            if (kick.car != &car) continue;
            lift = std::max(lift, kick.dv.y);
            turn = std::max(turn, Len(kick.dw));
        }
    }
    const float travel = bodies.GetPosition(body).GetZ() - 1.78f;
    std::printf("Grounded body contact: car lift %.3f m/s, turn %.3f rad/s, body travel %.3f m\n", lift, turn, travel);
    bodies.RemoveBody(body);
    bodies.DestroyBody(body);
    bodies.RemoveBody(ground);
    bodies.DestroyBody(ground);
    return lift > 0 && lift < .18f && turn > 0 && turn < .32f && travel > .4f;
}

int main() {
    static MmlMeshPiece input[2]{};
    static MmlShape slabs[3]{};
    const float pos[] = {0,0,0, 0,0,1, 1,0,0};
    const uint32_t index[] = {0,1,2};
    for (auto &p : input) {
        p.visual = &p;
        p.pos = pos; p.index = index; p.vertices = 3; p.triangles = 1;
        p.world[0] = p.world[5] = p.world[10] = p.world[15] = 1;
    }
    MmlHost host{};
    host.scene_focus = [](MmlMod *, const float *, const float *) { return 1; };
    host.scene_ready = host.scene_focus;
    host.world_shapes = [](MmlMod *, const float *, const float *, uint32_t, MmlShape *out, int cap) {
        if (!slabs[0].kind) return 0;
        if (out) std::copy_n(slabs,std::min(cap,3),out);
        return 3;
    };
    host.scene_pieces = [](MmlMod *, MmlMeshPiece *out, int cap) {
        if (out) std::copy_n(input, std::min(cap, 2), out);
        return 2;
    };
    host.world_water = [](MmlMod *, const float *, MmlWater *w) { w->height = 0; return 1; };
    mod::H = &host;
    scenery::SetFocus(scenery::kRagdoll, {-5,-5,-5}, {5,5,5});
    for (int dry : {0,1,0}) {
        for (auto &p : input) { p.world[13] = &p == &input[dry] ? 2.f : 0.f; ++p.version; }
        scenery::Tick();
        const auto &pieces = scenery::Pieces();
        if (pieces.size() != 1 || pieces.front().key != &input[dry]) {
            std::puts("FAIL: waterline instances aliased");
            return 1;
        }
    }
    std::string error;
    if (!jolt_world::Start(nullptr, {}, &error)) { std::puts(error.c_str()); return 1; }
    if (!CabinPanelTest(false) || !CabinPanelTest(true) ||
        !CabinPanelTest(false, true) || !CabinPanelTest(true, true)) {
        std::puts("FAIL: cabin panel must return an overlapping limb inward and keep the gap open");
        return 1;
    }
    for (int i=0;i<3;++i) {
        auto &s = slabs[i];
        s.prim = &s;
        s.kind = MML_SHAPE_BOX;
        s.flags = i==1 ? 0x0D000083 : 0x21000083;
        s.center[0] = i==0 ? 3.f : i==1 ? 5.f : 4.f;
        s.center[1] = i==2 ? 8.f : 10.f;
        s.center[2] = 2.f;
        s.half[0] = 2.f;
        s.half[1] = .1f;
        s.half[2] = i==2 ? 2.f : 1.f;
        if (i==2) s.axes[0] = s.axes[4] = s.axes[8] = 1.f;
        else s.axes[2] = s.axes[3] = s.axes[7] = 1.f;
    }
    scenery::SetFocus(scenery::kRagdoll,{-5,-5,-5},{8,11,5});
    scenery::Tick();
    const auto &pieces = scenery::Pieces();
    const auto contains = [&](int i) {
        return std::any_of(pieces.begin(),pieces.end(),[i](const auto &piece) { return piece.key==&slabs[i]; });
    };
    if (contains(0) || !contains(1) || !contains(2) || !SharedBarrierTest()) {
        std::puts("FAIL: shared physics must pass pedestrian barriers and keep walls and floors");
        return 1;
    }
    if (!GroundedBodyCarTest()) return 1;
    smoke::Mover car;
    car.kind = smoke::Mover::BOX; car.key = &car; car.mass = 1200;
    car.h = {1,.5f,2}; car.c = {0,10,0};
    car.ax[0] = {1,0,0}; car.ax[1] = {0,1,0}; car.ax[2] = {0,0,1};
    jolt_world::Step(.016f, {car}, nullptr);
    // A limb can reach the street before the rest of the passenger clears the door.
    // Keep it on the occupant layer throughout the fall onto the static scene triangle.
    JPH::BodyCreationSettings limb(new JPH::SphereShape(.1f), JPH::RVec3(.22f, 3.f, .22f),
        JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, jolt_world::kOccupant);
    const auto limbID = jolt_world::Bodies().CreateAndAddBody(limb, JPH::EActivation::Activate);
    for (int i = 0; i < 120; ++i) jolt_world::Step(1.f / 120.f, {car}, nullptr);
    const float restingHeight = jolt_world::Bodies().GetPosition(limbID).GetY();
    jolt_world::Bodies().RemoveBody(limbID);
    jolt_world::Bodies().DestroyBody(limbID);
    if (std::abs(restingHeight - 2.1f) > .025f) {
        std::printf("FAIL: occupant missed the static floor (centre %.3f)\n", restingHeight);
        return 1;
    }
    JPH::BodyIDVector ids;
    jolt_world::System()->GetBodies(ids);
    auto id = std::find_if(ids.begin(), ids.end(), [&](const auto &v) { return jolt_world::KinematicOwner(v) == &car; });
    if (id == ids.end()) { std::puts("FAIL: missing driven car"); return 1; }
    for (int step = 1; step <= 6; ++step) {
        car.c.x = static_cast<float>(step);
        jolt_world::Step(.100f, {car}, nullptr);
        const float actual = jolt_world::Bodies().GetPosition(*id).GetX();
        if (std::abs(actual - car.c.x) > .001f) {
            std::printf("FAIL: 100 ms step reached %.4f, expected %.4f\n", actual, car.c.x);
            return 1;
        }
    }
    for (float dt : {1.f / 30, 1.f / 120})
        for (bool hinge : {false, true})
            for (float scale : {0.f, 1.f})
                if (!SwingTest(hinge, scale, dt)) return 1;
    jolt_world::Stop();
    std::puts("PASS: waterline is per placement and six 100 ms steps reach the native car pose");
    return 0;
}
