#pragma once
#include <vector>
#include <string>
#include "mathx.h"
#include "collision.h"
#include "hat_fit.h"

struct MmlCapsule;

// A ragdoll's binding to the game: 15 body parts made from the human's skeleton (their masses, inertia and
// capsules), each driving one joint frame of the Mafia human skeleton. The parts move in Jolt (jolt_ragdoll: the
// joints, the muscles, the contacts); here their state (pose, velocities), the blows that change it, and the bones
// written from it. (The first solver, substepped XPBD, lived here until 2026-10-11.)
class Ragdoll {
    friend struct JoltBridge;   // src/jolt_ragdoll.cpp: the Jolt bodies of the 15 parts
    friend int main(int, char **);
public:
    // Skeleton points captured from the animated pose (joint origins plus helper tips).
    enum P {
        PELVIS, BACK1, BACK2, BACK3, NECK, HEAD, NOSE,
        RCLAV, RARM, RELB, RHAND, RTIP, RTHUMB,
        LCLAV, LARM, LELB, LHAND, LTIP, LTHUMB,
        RHIP, RKNEE, RANKLE, RTOE,
        LHIP, LKNEE, LANKLE, LTOE,
        COUNT
    };
    enum B {
        B_PELVIS, B_ABDOMEN, B_CHESTLOW, B_CHEST, B_HEAD,
        B_RUARM, B_RFARM, B_LUARM, B_LFARM,
        B_RTHIGH, B_RSHIN, B_RFOOT, B_LTHIGH, B_LSHIN, B_LFOOT,
        BCOUNT
    };
    // The same adult segment masses for dynamic bodies and living animated contact proxies (78 kg total).
    inline static constexpr float SegmentMass[BCOUNT] = {
        11.5f, 7.5f, 8.0f, 11.0f, 6.0f, 2.2f, 2.2f, 2.2f, 2.2f, 8.0f, 3.5f, 1.1f, 8.0f, 3.5f, 1.1f};

    inline static unsigned serials = 0;
    unsigned serial = ++serials;   // unique per ragdoll (a new one may take a dropped one's address)
    void *actor = nullptr;     // identity: a C_human, or a traffic pedestrian element (not an actor)
    void *model = nullptr;     // the model frame whose skeleton is driven
    bool headwearChecked = false;
    HatFit hatFit;
    bool pedestrian = false;   // `actor` is a pedestrian manager element
    bool carExtraction = false; // follow the puller's track, which outlasts the victim's track
    uint32_t keyVtable = 0;    // the actor's vtable when the ragdoll started (detects a reused address)
    bool puppet = false;       // started by command on a living human (no death)
    bool knockdown = false;    // a living human knocked over by a car: handed back to the game once it lies still
    bool railOccupant = false; // limbs and pelvis rest freely on the carriage geometry
    bool vehicleImpact = false; // car impacts use collision forces, with no animation-driven muscle target
    void *impactVehicle = nullptr; // keep the striking vehicle discoverable until it leaves the body's area
    bool sleeping = false;     // at rest (Jolt's sleep)
    int restWrites = 0;        // the pose written at rest so far (core's AtRest: after two, the bones are left be)
    bool shapeAtRest = false;  // its capsules given to the loader's bullets while asleep (kept there: not again)
    float sleepTimer = 0;      // time below the get-up motion threshold, including while the solver is awake
    float age = 0;

    // prevPts/prevDt: an earlier pose of the same human (world points) and the time since, so the ragdoll
    // starts with the motion the animation had (standing still, running, falling...).
    // A consistent picture of a pose: skeleton points plus the world rotation and origin of the 20 frames.
    struct PoseSnap { V3 pts[COUNT]; V3 fwd; M3 rot[20]; V3 org[20]; void *frames[20]; };
    static bool Snapshot(void *model, const V3 *dir, PoseSnap *out, std::string *err);
    // model: the human model frame; dir: the facing (a C_human's +0x30), or null to take the model's z axis.
    // start: the pose to build the ragdoll from (null = the pose now); prevPts/prevDt: an earlier pose for motion.
    bool Init(void *key, void *model, const V3 *dir, std::string *err, const V3 *prevPts = nullptr, float prevDt = 0,
              const PoseSnap *start = nullptr);
    // Closest point of the body to a line (a bullet's path), world.
    V3 ClosestToLine(const V3 &a, const V3 &b) const;
    // Skeleton points and helper tips follow the complete bone frames; fwd remains an informational heading.
    static void PointsFromPose(const M3 *rot20, const V3 *org20, V3 *pts);
    PoseSnap reference{};   // the model file's rest skeleton, independent of the animation at death
    // The pose the model's frames hold now (the 20 frames found at Init), facing the way the physical pelvis does.
    bool SnapshotNow(PoseSnap *out) const;
    bool BindClonedModel(void *clone); // same skeleton duplicated by the game's traffic carjack
    // The pose the bodies hold now, as frames 0..18 of `frames` (PoseSnap's order; 18 the pelvis): each one's world
    // rotation (rows) and origin, from the bodies themselves (what Apply writes, without the game's matrices).
    void PhysicalPose(M3 *rot19, V3 *org19) const;
    // Puts a pose on a skeleton (`frames20`: the frames Capture finds): the world rotations of frames 0..18 and the
    // pelvis frame's world origin; the root (19) stays where the game has it. `model` is re-skinned (as in Apply).
    static void WritePose(void *model, void *const *frames20, const M3 *rot19, const V3 &pelvis);
    // A frame's world rotation (rows) and origin, set through its parent.
    static void PlaceFrame(void *frame, const M3 &rot, const V3 &pos);
    void *const *Frames() const { return frames; }   // the skeleton's 20 frames (PoseSnap's order)
    void *Model() const { return model; }
    const V3 *InitialPoints() const { return initPos; }
    // The pose the game's animation gave the model after our last write (render hook), for the muscles: `anim`
    // holds it, `animSerial` counts the poses taken.
    PoseSnap anim{};
    unsigned animSerial = 0;
    // Set when bullets or blasts edit the parts' velocities: the physics takes them over.
    bool velocityEdited = false;
    void SetMotionFrom(const V3 *prevWorld, float dt, const V3 *rootVelocity = nullptr);
    void Setup(const V3 *pts, const V3 &fwd, const M3 *rot20 = nullptr);   // parts and full skeletal orientation
    // A blast at `centre`: each part thrown away from it (and up by `lift` of its speed), `speed` m/s at the centre
    // falling off to 0 at `radius` as (1 - d/radius)^1.5; the nearer side more, so the body turns. False: out of reach.
    bool Blast(const V3 &centre, float radius, float speed, float lift);
    // A blow of momentum J (N s, world) at `at`. `whole` of it moves the whole body (each part by the same speed);
    // the rest goes to the parts within `radius` of `at` (by mass and closeness; the nearest part when none is),
    // with the spin its offset gives. The blows of one tick add up, and all of them together speed a part up by at
    // most `maxPart` m/s and spin it by at most 2 * maxPart rad/s: the rest of its share goes to the whole body, so
    // the body takes all of J. (Nine pellets on one forearm, each held alone, once spun it past its joint's limits
    // in one step: the joint was caught on the wrong side and stayed twisted.)
    // When the bullet ray already selected a part, its local share goes to that part alone.
    float Kick(const V3 &J, const V3 &at, float whole, float radius, float maxPart, int part = -1);
    static constexpr int BulletShapeCount() { return BCOUNT + 2; }
    static int BulletBody(int shape);
    void BulletCapsules(MmlCapsule *caps) const;
    // Elbow, wrist and palm tip in the forearm's rigid-body space, shared by bullet and ground contact.
    bool HandShape(int forearm, V3 *elbow, V3 *wrist, V3 *tip) const;
    static constexpr float PalmRadius = 0.045f;
    void Apply();              // write bone transforms into the frames
    std::string Describe() const;
    std::string Diagnose() const;
    std::string BoneCheck() const;  // frame pose in game vs. physics, per bone
    // Largest change (degrees) the game made to our bones since the last Apply, and which bone.
    float Overwritten(const char **bone) const;
    // Whether any frame it writes (its bones, its start pose's) is in `frames` (frames LS3D freed).
    template <class Set> bool UsesAny(const Set &frames) const {
        for (const BoneLink &b : bones) if (frames.count(b.frame)) return true;
        for (void *f : this->frames) if (f && frames.count(f)) return true;
        return false;
    }
    V3 Center() const;         // chest position, for impulses
    // Physics pelvis vs. where the game will draw it (pelvis frame world matrix), and the model root.
    void Probe(V3 *physPelvis, V3 *drawnPelvis, V3 *root) const;
    V3 BodyPos(int b) const { return origin + body[b].x; }   // world centre of a body part
    V3 BodyVel(int b) const { return body[b].v; }            // its velocity (m/s) as the solver last left it
    V3 PointVelocity(int b, V3 point) const { return body[b].v + Cross(body[b].w, point - BodyPos(b)); }
    V3 HeadLocalPoint(V3 world) const { return InvRotate(body[B_HEAD].q,world-BodyPos(B_HEAD)); }
    V3 HeadWorldPoint(V3 local) const { return BodyPos(B_HEAD)+Rotate(body[B_HEAD].q,local); }
    // Collision capsule of a body part in world space (segment ends and radius), for other simulations.
    static constexpr int BodyCount() { return BCOUNT; }
    void BodyCapsule(int b, V3 *a, V3 *e, float *r) const {
        const Body &bd = body[b];
        *a = origin + bd.x + Rotate(bd.q, bd.a);
        *e = origin + bd.x + Rotate(bd.q, bd.b);
        *r = bd.radius;
    }
    V3 rootPos{};              // where Apply last put the model root (world)

    struct Quat { float w = 1, x = 0, y = 0, z = 0; };
    Quat NeckRotation() const {
        const Quat &chest = body[B_CHEST].q;
        return Mul({chest.w, -chest.x, -chest.y, -chest.z}, body[B_HEAD].q);
    }

private:
    struct Body {
        V3 x{}, v{}, w{};          // center of mass, linear and angular velocity (world)
        Quat q, qPrev;             // orientation (local -> world)
        V3 xPrev{};
        float invMass = 0;
        V3 invInertia{};           // diagonal, body frame
        V3 a{}, b{};               // capsule segment ends in body frame
        float radius = 0;
    };

    Body body[BCOUNT];
    V3 kickV[BCOUNT] = {}, kickW[BCOUNT] = {};   // what this tick's blows gave each part locally (Kick)
    float kickAge = -1;                          // the age those belong to

    // Bone mapping (engine row-matrix convention): bone world = offset * body world rows.
    struct BoneLink { void *frame; int bodyIdx; M3 offset; V3 originLocal; const char *name; float written[4] = {1, 0, 0, 0}; bool wrote = false; };
    std::vector<BoneLink> bones;
    void *frames[20] = {};           // the frames of the start pose (PoseSnap::frames)
    void *rootFrame = nullptr;
    void *pelvisFrame = nullptr;
    V3 pelvisOffset{};   // pelvis frame origin in the pelvis body frame

    V3 origin{0, 0, 0};   // world position of the local simulation frame
    V3 initPos[COUNT];
    V3 initFwd{0, 0, 1};
    M3 initRot[20];

    static V3 Rotate(const Quat &q, const V3 &v);
    static V3 InvRotate(const Quat &q, const V3 &v);
    static Quat FromAxes(const V3 &c0, const V3 &c1, const V3 &c2);   // columns = local axes in world
    static Quat Mul(const Quat &a, const Quat &b);
    static void Normalize(Quat &q);
    static void AddRotation(Quat &q, const V3 &w);   // q += 0.5 * [w,0] * q, normalized
    V3 InvInertiaWorld(const Body &bd, const V3 &v) const;

    void MakeBody(int id, const V3 &a, const V3 &b, const V3 &side, float radius, float mass);
    void SetCapsule(int id, const V3 &a, const V3 &b, float radius);   // collision shape independent of the bone axis
    V3 AxisPoint(int b, float t) const;                       // world point on body b's axis, t in [0,1]
};
