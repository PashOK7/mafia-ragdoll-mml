#include <windows.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "dogs.h"
#include "fmt.h"
#include "game.h"
#include "jolt_world.h"
#include "mod.h"
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Skeleton/Skeleton.h>

namespace dogs {
namespace {
constexpr int kDog = 0x15, kHealth = 0xd0, kState = 0xb4, kDead = 0x0f;
// The stock pes skin has one physical trunk root and four independent leg chains.
enum Bone { Pelvis, Back, Chest, Neck, Throat, Head, Jaw,
            FrontLeftShoulder, FrontLeftElbow, FrontLeftWrist, FrontLeftToe,
            FrontRightShoulder, FrontRightElbow, FrontRightWrist, FrontRightToe,
            HindLeftHip, HindLeftKnee, HindLeftHock, HindLeftToe,
            HindRightHip, HindRightKnee, HindRightHock, HindRightToe,
            Tail, TailMid, TailTip, BoneCount };
struct BoneDef { const char *name; int parent; };
constexpr BoneDef bones[] = {
    {"a", -1}, {"b", Pelvis}, {"c", Back}, {"d", Chest}, {"e", Neck}, {"f", Throat}, {"jaw", Head},
    {"g", Chest}, {"h", FrontLeftShoulder}, {"i", FrontLeftElbow}, {"j", FrontLeftWrist},
    {"k", Chest}, {"l", FrontRightShoulder}, {"m", FrontRightElbow}, {"n", FrontRightWrist},
    {"o", Pelvis}, {"p", HindLeftHip}, {"q", HindLeftKnee}, {"r", HindLeftHock},
    {"s", Pelvis}, {"t", HindRightHip}, {"u", HindRightKnee}, {"v", HindRightHock},
    {"w", Pelvis}, {"x", Tail}, {"y", TailMid}
};
struct PartDef {
    const char *name;
    Bone frame, from, to;
    int parent;
    float radius, mass, flex, side, twist;
};
constexpr PartDef parts[] = {
    {"pelvis", Pelvis, Pelvis, Tail, -1, .115f, 6.5f, 0, 0, 0},
    {"chest", Chest, Back, Neck, 0, .128f, 8.f, 24, 18, 15},
    {"neck", Neck, Neck, Head, 1, .078f, 1.5f, 40, 34, 25},
    {"head", Head, Head, Jaw, 2, .072f, 2.f, 32, 28, 22},
    {"front-left-shoulder", FrontLeftShoulder, FrontLeftShoulder, FrontLeftElbow, 1, .052f, 1.3f, 65, 30, 25},
    {"front-left-foreleg", FrontLeftElbow, FrontLeftElbow, FrontLeftWrist, 4, .033f, .7f, 70, 12, 10},
    {"front-left-paw", FrontLeftWrist, FrontLeftWrist, FrontLeftToe, 5, .028f, .3f, 48, 18, 16},
    {"front-right-shoulder", FrontRightShoulder, FrontRightShoulder, FrontRightElbow, 1, .052f, 1.3f, 65, 30, 25},
    {"front-right-foreleg", FrontRightElbow, FrontRightElbow, FrontRightWrist, 7, .033f, .7f, 70, 12, 10},
    {"front-right-paw", FrontRightWrist, FrontRightWrist, FrontRightToe, 8, .028f, .3f, 48, 18, 16},
    {"hind-left-thigh", HindLeftHip, HindLeftHip, HindLeftKnee, 0, .064f, 1.5f, 65, 32, 23},
    {"hind-left-shin", HindLeftKnee, HindLeftKnee, HindLeftHock, 10, .042f, .8f, 75, 13, 10},
    {"hind-left-paw", HindLeftHock, HindLeftHock, HindLeftToe, 11, .029f, .35f, 50, 17, 16},
    {"hind-right-thigh", HindRightHip, HindRightHip, HindRightKnee, 0, .064f, 1.5f, 65, 32, 23},
    {"hind-right-shin", HindRightKnee, HindRightKnee, HindRightHock, 13, .042f, .8f, 75, 13, 10},
    {"hind-right-paw", HindRightHock, HindRightHock, HindRightToe, 14, .029f, .35f, 50, 17, 16},
    {"tail", Tail, Tail, TailTip, 0, .022f, .24f, 42, 35, 25},
};
constexpr int PartCount = std::size(parts);
constexpr float kRadians = 0.01745329252f;

struct Local { V3 pos; Q rot; V3 scale; };
struct Transform { V3 pos; M3 basis; };
struct Pose { std::array<Local, BoneCount> local; std::array<Transform, BoneCount> world; };
struct Link { V3 offset; M3 rotation; };
struct Shot { V3 pos{}, dir{}; float damage=0, remaining=0; };
struct Dog {
    void *model = nullptr;
    std::array<void *, BoneCount> frames{};
    Pose pose{};
    std::array<Local, BoneCount> beforeRender{};
    std::array<Link, PartCount> links{};
    JPH::Ref<JPH::RagdollSettings> settings;
    JPH::Ref<JPH::Ragdoll> body;
    V3 velocity{};
    Shot shot{};
    bool captured=false, renderPose=false;
    unsigned seen=0;
    float age=0;
};
std::unordered_map<void *, Dog> tracked;
unsigned tick;
JPH::CollisionGroup::GroupID nextGroup = 0x40000000u;

JPH::Vec3 Vec(V3 v) { return {v.x,v.y,v.z}; }
V3 Vec(JPH::Vec3Arg v) { return {v.GetX(),v.GetY(),v.GetZ()}; }
V3 Position(const float *m) { return {m[12],m[13],m[14]}; }
JPH::Quat Rotation(const M3 &m) {
    const auto q=QuatOf(m);
    return {-q.x,-q.y,-q.z,q.w};
}
M3 Rotation(JPH::QuatArg q) {
    return {{Vec(q*JPH::Vec3::sAxisX()),Vec(q*JPH::Vec3::sAxisY()),Vec(q*JPH::Vec3::sAxisZ())}};
}
bool Dead(void *actor) {
    return game::at<int>(actor,kState)==kDead && game::at<float>(actor,kHealth)<=0.f;
}
bool Bind(Dog &d) {
    for (int i=0;i<BoneCount;++i) {
        const int parent=bones[i].parent;
        void *root=parent<0 ? d.model : d.frames[parent];
        void *frame=game::FindChild(root,bones[i].name);
        if (!frame || game::at<void *>(frame,game::kFrameParent)!=root ||
            game::at<int>(frame,game::kFrameType)!=(parent<0 ? 1 : 10)) return false;
        d.frames[i]=frame;
    }
    return true;
}
Local LocalPose(void *frame) {
    game::WorldMatrix(frame);
    const float *m=&game::at<float>(frame,game::kFrameLocal);
    return {Position(m),QuatOf(M3::FromWorld(m)),game::at<V3>(frame,game::kFrameScale)};
}
Transform WorldPose(void *frame) {
    const float *m=game::WorldMatrix(frame);
    return {Position(m),M3::Raw(m)};
}
Pose Capture(const Dog &dog) {
    Pose out;
    for (int i=0;i<BoneCount;++i) {
        out.world[i]=WorldPose(dog.frames[i]);
        out.local[i]=LocalPose(dog.frames[i]);
    }
    return out;
}
void Write(void *frame,const Local &local) {
    game::SetLocalRotation(frame,local.rot);
    game::SetLocalPosition(frame,local.pos);
}
void Update(void *model) {
    using UpdateFn=void (__stdcall *)(void *);
    reinterpret_cast<UpdateFn>((*reinterpret_cast<void ***>(model))[0x18/4])(model);
}
void Restore(Dog &d) {
    if (!d.renderPose) return;
    for (int i=0;i<BoneCount;++i) Write(d.frames[i],d.beforeRender[i]);
    Update(d.model);
    d.renderPose=false;
}
void Remove(Dog &d) {
    Restore(d);
    if (d.body && jolt_world::Running()) d.body->RemoveFromPhysicsSystem(false);
    d.body=nullptr;
    d.settings=nullptr;
}
M3 PartOrientation(V3 from,V3 to,V3 lateral) {
    const V3 y=Norm(to-from);
    V3 x=lateral-y*Dot(lateral,y);
    if (Len(x)<.02f) x=Cross(y,std::fabs(y.x)<.8f ? V3{1,0,0} : V3{0,0,1});
    x=Norm(x);
    return {{x,y,Cross(x,y)}};
}
bool Add(Dog &d) {
    JPH::Ref<JPH::Skeleton> skeleton=new JPH::Skeleton;
    JPH::Ref<JPH::RagdollSettings> settings=new JPH::RagdollSettings;
    settings->mSkeleton=skeleton;
    settings->mParts.resize(PartCount);
    JPH::Mat44 poses[PartCount];
    const auto &points=d.pose.world;
    const V3 lateral=Norm(points[FrontRightShoulder].pos-points[FrontLeftShoulder].pos,{1,0,0});
    for (int i=0;i<PartCount;++i) {
        const PartDef &spec=parts[i];
        skeleton->AddJoint(spec.name,spec.parent);
        const V3 from=points[spec.from].pos, to=points[spec.to].pos;
        const float length=Len(to-from);
        if (!(length>.015f && length<.85f)) return false;
        const M3 basis=PartOrientation(from,to,lateral);
        const V3 centre=(from+to)*.5f;
        const auto &bone=points[spec.frame];
        M3 boneRotation=bone.basis;
        boneRotation.Orthonormalize();
        d.links[i]={basis.T().Apply(bone.pos-centre),boneRotation*basis.T()};
        auto &part=settings->mParts[i];
        part.SetShape(new JPH::CapsuleShape(std::max(.01f,length*.5f-spec.radius*.45f),spec.radius));
        part.mPosition=Vec(jolt_world::ToJolt(centre));
        part.mRotation=Rotation(basis);
        part.mObjectLayer=jolt_world::kDynamic;
        part.mMotionType=JPH::EMotionType::Dynamic;
        part.mFriction=.72f;
        part.mLinearDamping=.25f;
        part.mAngularDamping=.8f;
        part.mMotionQuality=JPH::EMotionQuality::LinearCast;
        part.mOverrideMassProperties=JPH::EOverrideMassProperties::CalculateInertia;
        part.mMassPropertiesOverride.mMass=spec.mass;
        part.mLinearVelocity=Vec(d.velocity);
        poses[i]=JPH::Mat44::sRotationTranslation(part.mRotation,part.mPosition);
        if (spec.parent<0) continue;
        auto *joint=new JPH::SwingTwistConstraintSettings;
        joint->mPosition1=joint->mPosition2=Vec(jolt_world::ToJolt(bone.pos));
        joint->mTwistAxis1=joint->mTwistAxis2=Vec(basis.r[1]);
        joint->mPlaneAxis1=joint->mPlaneAxis2=Vec(basis.r[0]);
        joint->mNormalHalfConeAngle=spec.flex*kRadians;
        joint->mPlaneHalfConeAngle=spec.side*kRadians;
        joint->mTwistMinAngle=-spec.twist*kRadians;
        joint->mTwistMaxAngle=spec.twist*kRadians;
        joint->mMaxFrictionTorque=.12f*spec.mass;
        part.mToParent=joint;
    }
    settings->Stabilize();
    settings->DisableParentChildCollisions(poses,.025f);
    settings->CalculateConstraintPriorities();
    settings->CalculateBodyIndexToConstraintIndex();
    d.body=settings->CreateRagdoll(nextGroup++,0,jolt_world::System());
    if (!d.body) return false;
    d.settings=settings;
    d.age=0;
    d.body->AddToPhysicsSystem(JPH::EActivation::Activate,false);
    if (d.shot.remaining>0) {
        int closest=0;
        for (int i=1;i<PartCount;++i)
            if (Len(points[parts[i].frame].pos-d.shot.pos)<Len(points[parts[closest].frame].pos-d.shot.pos)) closest=i;
        const float strength=std::clamp(.055f*d.shot.damage,.8f,4.f);
        const auto id=d.body->GetBodyID(closest);
        auto &physics=jolt_world::Bodies();
        const V3 centre=Vec(physics.GetCenterOfMassPosition(id));
        const V3 offset=jolt_world::ToJolt(d.shot.pos)-centre;
        // Native hit volumes can extend well beyond the animated mesh.
        const V3 point=centre+Norm(offset)*std::min(Len(offset),.18f);
        physics.AddImpulse(id,Vec(d.shot.dir*strength),Vec(point));
    }
    return true;
}
Transform Relative(const Transform &parent,const Local &local) {
    float m[16];
    QuatMatrix(local.rot,m);
    M3 b=M3::Raw(m);
    b.r[0]=b.r[0]*local.scale.x;
    b.r[1]=b.r[1]*local.scale.y;
    b.r[2]=b.r[2]*local.scale.z;
    return {parent.pos+parent.basis.Apply(local.pos),b*parent.basis};
}
void Apply(Dog &d) {
    const Transform root=WorldPose(d.model);
    std::array<Transform,BoneCount> world;
    auto &physics=jolt_world::Bodies();
    for (int i=0;i<BoneCount;++i) d.beforeRender[i]=LocalPose(d.frames[i]);
    d.renderPose=true;
    for (int i=0;i<BoneCount;++i) {
        const int parent=bones[i].parent;
        const auto &above=parent<0 ? root : world[parent];
        Local local=d.pose.local[i];
        const auto part=std::find_if(std::begin(parts),std::end(parts),[i](const PartDef &p){return p.frame==i;});
        if (part!=std::end(parts)) {
            const auto index=part-std::begin(parts);
            JPH::RVec3 p;
            JPH::Quat q;
            physics.GetPositionAndRotation(d.body->GetBodyID((int)index),p,q);
            const M3 rotated=Rotation(q);
            const V3 point=jolt_world::ToWorld(Vec(p))+rotated.Apply(d.links[index].offset);
            M3 aboveRotation=above.basis;
            aboveRotation.Orthonormalize();
            local.pos=above.basis.Inverse().Apply(point-above.pos);
            local.rot=QuatOf(d.links[index].rotation*rotated*aboveRotation.T());
        }
        Write(d.frames[i],local);
        world[i]=Relative(above,local);
    }
    Update(d.model);
}
V3 Centre(const Dog &d) {
    return d.body ? jolt_world::ToWorld(Vec(jolt_world::Bodies().GetPosition(d.body->GetBodyID(0))))
                  : d.pose.world[Pelvis].pos;
}
}

void PrepareStep(float dt,const V3 &player) {
    ++tick;
    std::vector<void *> actors;
    game::ListActors(&actors);
    for (void *actor:actors) {
        if (!actor || game::at<int>(actor,game::kActorType)!=kDog) continue;
        void *model=game::at<void *>(actor,game::kActorFrame);
        if (!model || !(game::at<uint32_t>(model,game::kFrameFlags)&1)) continue;
        auto found=tracked.find(actor);
        if (found!=tracked.end() && found->second.model!=model) {
            Remove(found->second);
            tracked.erase(found);
        }
        if (Len(game::ActorWorldPos(actor)-player)>65.f && !tracked.contains(actor)) continue;
        auto &dog=tracked[actor];
        if (dog.seen==tick) continue;
        dog.seen=tick;
        if (!dog.model) {
            dog.model=model;
            if (!Bind(dog)) dog.frames[0]=nullptr;
        }
        if (!dog.frames[0]) continue;
        if (!Dead(actor)) {
            dog.shot.remaining-=dt;
            if (dog.body) Remove(dog);
            Pose current=Capture(dog);
            const V3 delta=current.world[Pelvis].pos-dog.pose.world[Pelvis].pos;
            if (dog.captured && dt>.001f && dt<.15f) {
                const V3 velocity=delta*(1.f/dt);
                dog.velocity=Len(velocity)<12.f ? dog.velocity*.6f+velocity*.4f : V3{};
            }
            dog.pose=std::move(current);
            dog.captured=true;
            continue;
        }
        if (!dog.captured) {
            dog.pose=Capture(dog);
            dog.captured=true;
        }
        const V3 centre=Centre(dog);
        jolt_world::AddFocus(centre-V3{8,4,8},centre+V3{8,4,8});
        if (dog.body) dog.age+=dt;
        else if (jolt_world::StaticReady(centre-V3{1,2,1},centre+V3{1,1,1}) && Add(dog))
            mod::Log("quadruped ragdoll %p: %d parts",actor,PartCount);
    }
    for (auto it=tracked.begin();it!=tracked.end();) {
        if (it->second.seen==tick) { ++it; continue; }
        Remove(it->second);
        it=tracked.erase(it);
    }
}

bool Near(const V3 &point) {
    return std::any_of(tracked.begin(),tracked.end(),[&](const auto &entry) {
        return entry.second.body && Len(Centre(entry.second)-point)<6.f;
    });
}
void Bullet(const MmlBulletHit &hit) {
    const auto it=tracked.find(hit.actor);
    if (it==tracked.end() || it->second.body) return;
    it->second.shot={{hit.pos.x,hit.pos.y,hit.pos.z},{hit.dir.x,hit.dir.y,hit.dir.z},hit.damage,.25f};
}
void DrawPose() {
    for (auto &[actor,dog]:tracked)
        if (dog.body && Dead(actor) && game::at<void *>(actor,game::kActorFrame)==dog.model) Apply(dog);
}
void RestorePose() {
    for (auto &[_,dog]:tracked) Restore(dog);
}
void Forget(void *actor) {
    const auto it=tracked.find(actor);
    if (it==tracked.end()) return;
    Remove(it->second);
    tracked.erase(it);
}
void Clear() {
    for (auto &[_,dog]:tracked) Remove(dog);
    tracked.clear();
}
std::string State() {
    std::string result="[";
    for (const auto &[actor,dog]:tracked) {
        if (result.size()>1) result+=",";
        const V3 p=Centre(dog);
        result+=Fmt("{\"actor\":\"0x%08X\",\"bound\":%s,\"active\":%s,\"parts\":%d,\"age\":%.3f,\"centre\":[%.4f,%.4f,%.4f]}",
                    (unsigned)(uintptr_t)actor,dog.frames[0]?"true":"false",dog.body?"true":"false",
                    dog.body?PartCount:0,dog.age,p.x,p.y,p.z);
    }
    return result+"]";
}
}
