// Production scenery and Jolt step, with two instances of one mesh supplied by the SDK.
#include "mod.h"
#include "scenery.h"
#include "headwear.h"
#include "jolt_world.h"
#include "joint_resistance.h"
#include <Jolt/Physics/Body/BodyCreationSettings.h>
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

static bool RailPanelTest(bool solid) {
    using namespace JPH;
    std::vector<V3> surfaces{{-1,0,-6},{1,0,0},{1,0,-6}};
    const std::vector<V3> wall{
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
    if (solid) cabin.cabinPanels = &wall;
    else surfaces.insert(surfaces.end(),wall.begin(),wall.end());
    auto &bodies = jolt_world::Bodies();
    BodyCreationSettings limb(new SphereShape(.12f), RVec3(-.55f,11,-5.035f),
        Quat::sIdentity(), EMotionType::Dynamic, jolt_world::kOccupant);
    limb.mGravityFactor = 0;
    limb.mMotionQuality = EMotionQuality::LinearCast;
    limb.mAllowSleeping = false;
    limb.mUserData = reinterpret_cast<uintptr_t>(&limb);
    const auto caught = bodies.CreateAndAddBody(limb, EActivation::Activate);
    limb.mPosition = RVec3(.65f,11,-4.75f);
    limb.mLinearVelocity = Vec3(0,0,-4);
    const auto free = bodies.CreateAndAddBody(limb, EActivation::Activate);
    for (int i=0;i<40;++i) {
        jolt_world::RequestCabin(&cabin,&limb,true);
        jolt_world::Step(1.f/120.f,{cabin},nullptr);
    }
    const float recovered = bodies.GetPosition(caught).GetZ();
    const float exited = bodies.GetPosition(free).GetZ();
    for (auto id : {caught,free}) {
        bodies.RemoveBody(id);
        bodies.DestroyBody(id);
    }
    jolt_world::Step(1.f/120.f,{},nullptr);
    std::printf("Rail %s settled at %.3f m; opening crossed to %.3f m\n",
                solid ? "solid panel" : "two-sided mesh",recovered,exited);
    return (solid ? recovered > -4.93f : recovered < -5.f) && exited < -5.5f;
}

int main() {
    static MmlMeshPiece input[2]{};
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
    host.world_shapes = [](MmlMod *, const float *, const float *, uint32_t, MmlShape *, int) { return 0; };
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
    if (!RailPanelTest(false) || !RailPanelTest(true)) {
        std::puts("FAIL: rail panel must return an overlapping limb inward and keep the gap open");
        return 1;
    }
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
