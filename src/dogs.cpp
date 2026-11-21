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
#include "articulation.h"
#include "joint_resistance.h"
#include "model_skeleton.h"
#include "mod.h"
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
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
    bool hinge=false;
};
constexpr PartDef parts[] = {
    {"pelvis", Pelvis, Pelvis, Tail, -1, .115f, 6.5f, 0, 0, 0},
    {"chest", Chest, Back, Neck, 0, .145f, 8.f, 24, 18, 15},
    {"neck", Neck, Neck, Head, 1, .081f, 1.5f, 43, 34, 25},
    {"head", Head, Head, Jaw, 2, .079f, 2.f, 36, 28, 22},
    {"front-left-shoulder", FrontLeftShoulder, FrontLeftShoulder, FrontLeftElbow, 1, .052f, 1.3f, 65, 30, 25},
    {"front-left-foreleg", FrontLeftElbow, FrontLeftElbow, FrontLeftWrist, 4, .038f, .7f, 112, 19, 0, true},
    {"front-left-paw", FrontLeftWrist, FrontLeftWrist, FrontLeftToe, 5, .032f, .3f, 48, 18, 16},
    {"front-right-shoulder", FrontRightShoulder, FrontRightShoulder, FrontRightElbow, 1, .052f, 1.3f, 65, 30, 25},
    {"front-right-foreleg", FrontRightElbow, FrontRightElbow, FrontRightWrist, 7, .038f, .7f, 112, 19, 0, true},
    {"front-right-paw", FrontRightWrist, FrontRightWrist, FrontRightToe, 8, .032f, .3f, 48, 18, 16},
    {"hind-left-thigh", HindLeftHip, HindLeftHip, HindLeftKnee, 0, .064f, 1.5f, 65, 32, 23},
    {"hind-left-shin", HindLeftKnee, HindLeftKnee, HindLeftHock, 10, .047f, .8f, 91, 34, 0, true},
    {"hind-left-paw", HindLeftHock, HindLeftHock, HindLeftToe, 11, .033f, .35f, 60, 16, 14},
    {"hind-right-thigh", HindRightHip, HindRightHip, HindRightKnee, 0, .064f, 1.5f, 65, 32, 23},
    {"hind-right-shin", HindRightKnee, HindRightKnee, HindRightHock, 13, .047f, .8f, 91, 34, 0, true},
    {"hind-right-paw", HindRightHock, HindRightHock, HindRightToe, 14, .033f, .35f, 60, 16, 14},
    {"tail", Tail, Tail, TailMid, 0, .026f, .18f, 40, 42, 25},
    {"tail-tip", TailMid, TailMid, TailTip, 16, .018f, .09f, 32, 36, 22},
};
constexpr int PartCount = std::size(parts);

struct Local { V3 pos; Q rot; V3 scale; };
using Transform=SkeletonTransform;
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
    std::array<Transform,BoneCount> rest{};
    std::array<V3,PartCount> velocity{}, spin{};
    V3 rootVelocity{};
    Shot shot{};
    bool captured=false, renderPose=false, releaseMotors=false;
    unsigned seen=0;
    float age=0, poseError=0;
};
std::unordered_map<void *, Dog> tracked;
std::unordered_map<std::string,std::array<Transform,BoneCount>> restByModel;
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
    char file[512]{};
    const auto *host=mod::H;
    if (!host->model_file(mod::M,d.model,file,sizeof file)) return false;
    const auto cached=restByModel.find(file);
    if (cached!=restByModel.end()) {
        d.rest=cached->second;
        return true;
    }
    std::vector<MmlModelNode> nodes;
    std::vector<SkeletonTransform> world;
    std::string error;
    if (!ReadSkeleton(file,nodes,world,&error)) {
        mod::Warn("dog rig: %s",error.c_str());
        return false;
    }
    std::array<int,BoneCount> indices{};
    for (int i=0;i<BoneCount;++i) {
        const int parent=bones[i].parent;
        const int parentIndex=parent<0 ? -1 : indices[parent];
        auto node=std::find_if(nodes.begin(),nodes.end(),[&](const MmlModelNode &candidate) {
            return candidate.parent==parentIndex && std::strcmp(candidate.name,bones[i].name)==0;
        });
        if (node==nodes.end()) return false;
        indices[i]=int(node-nodes.begin());
        d.rest[i]=world[indices[i]];
    }
    restByModel.emplace(file,d.rest);
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
    if (d.body && jolt_world::Running()) {
        for (int i=0;i<PartCount;++i) jolt_world::ClearFloat(d.body->GetBodyID(i));
        d.body->RemoveFromPhysicsSystem(false);
    }
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
using Skeleton=std::array<Transform,BoneCount>;
std::array<Transform,PartCount> PartPose(const Skeleton &pose) {
    const V3 lateral=Norm(pose[FrontRightShoulder].pos-pose[FrontLeftShoulder].pos,{1,0,0});
    std::array<Transform,PartCount> result;
    for (int i=0;i<PartCount;++i) {
        const auto &part=parts[i];
        const V3 from=pose[part.from].pos, to=pose[part.to].pos;
        result[i]={(from+to)*.5f,PartOrientation(from,to,lateral)};
    }
    return result;
}
V3 ClampLength(V3 v,float max) {
    const float n=Len(v);
    return n>max ? v*(max/n) : v;
}
void TrackMotion(Dog &dog,const Pose &current,float dt) {
    if (!dog.captured || !(dt>.001f && dt<.12f)) return;
    const V3 delta=current.world[Pelvis].pos-dog.pose.world[Pelvis].pos;
    if (Len(delta)>dt*14.f+.08f) {
        dog.velocity={};
        dog.spin={};
        dog.rootVelocity={};
        return;
    }
    const float blend=1.f-std::exp(-dt/.052f);
    const V3 root=ClampLength(delta*(1.f/dt),10.f);
    dog.rootVelocity=dog.rootVelocity*(1.f-blend)+root*blend;
    const auto before=PartPose(dog.pose.world), now=PartPose(current.world);
    for (int i=0;i<PartCount;++i) {
        const V3 part=(now[i].pos-before[i].pos)*(1.f/dt);
        const V3 velocity=ClampLength(part-root,i<4 ? 2.f : 4.f);
        const auto rotation=(Rotation(now[i].basis)*Rotation(before[i].basis).Conjugated()).Normalized();
        const V3 spin=ClampLength(Vec(rotation.GetAngularVelocity(dt)),i<4 ? 5.f : 12.f);
        dog.velocity[i]=dog.velocity[i]*(1.f-blend)+velocity*blend;
        dog.spin[i]=dog.spin[i]*(1.f-blend)+spin*blend;
    }
}
JPH::RefConst<JPH::Shape> Shape(int index,float length,float scale,float side) {
    const float radius=parts[index].radius*scale;
    if (index<2)
        return new JPH::BoxShape(Vec(V3{radius+side,length*.5f+radius*.34f,radius*1.12f}),radius*.24f);
    return new JPH::CapsuleShape(std::max(.009f*scale,length*.5f-radius*.55f),radius);
}
void Joint(JPH::RagdollSettings::Part &part,int index,V3 anchor,
           const std::array<Transform,PartCount> &now,const std::array<Transform,PartCount> &rest,
           V3 forward) {
    const PartDef &spec=parts[index];
    const M3 &child=now[index].basis, &neutralChild=rest[index].basis;
    const M3 carryParent=rest[spec.parent].basis.T()*now[spec.parent].basis;
    if (spec.hinge) {
        V3 axis=neutralChild.r[0];
        if (Dot(Cross(axis,neutralChild.r[1]),forward)>0) axis=-axis;
        const V3 normal=neutralChild.r[1];
        const M3 carryChild=neutralChild.T()*child;
        part.mToParent=articulation::Hinge(anchor,{carryParent.Apply(axis),carryParent.Apply(normal)},
            {carryChild.Apply(axis),carryChild.Apply(normal)},-spec.side,spec.flex,4.f);
    } else {
        part.mToParent=articulation::Ball(anchor,
            {carryParent.Apply(neutralChild.r[1]),carryParent.Apply(neutralChild.r[0])},
            {child.r[1],child.r[0]},spec.flex,spec.side,-spec.twist,spec.twist,4.f);
    }
}
bool Add(Dog &d) {
    JPH::Ref<JPH::Skeleton> skeleton=new JPH::Skeleton;
    JPH::Ref<JPH::RagdollSettings> settings=new JPH::RagdollSettings;
    settings->mSkeleton=skeleton;
    settings->mParts.resize(PartCount);
    JPH::Mat44 poses[PartCount];
    const auto &points=d.pose.world;
    const auto now=PartPose(points), rest=PartPose(d.rest);
    const V3 forward=Norm(d.rest[Neck].pos-d.rest[Tail].pos,{0,0,1});
    const float scale=std::clamp(Len(points[Pelvis].basis.r[0]),.6f,1.55f);
    const float shoulderWidth=Len(points[FrontRightShoulder].pos-points[FrontLeftShoulder].pos);
    for (int i=0;i<PartCount;++i) {
        const PartDef &spec=parts[i];
        skeleton->AddJoint(spec.name,spec.parent);
        const V3 from=points[spec.from].pos, to=points[spec.to].pos;
        const float length=Len(to-from);
        if (!(length>.015f && length<.85f)) return false;
        const auto &[centre,basis]=now[i];
        const auto &bone=points[spec.frame];
        M3 boneRotation=bone.basis;
        boneRotation.Orthonormalize();
        d.links[i]={basis.T().Apply(bone.pos-centre),boneRotation*basis.T()};
        auto &part=settings->mParts[i];
        const float width=std::clamp(shoulderWidth*.5f-spec.radius*scale,0.f,.035f*scale);
        part.SetShape(Shape(i,length,scale,width));
        part.mPosition=Vec(jolt_world::ToJolt(centre));
        part.mRotation=Rotation(basis);
        part.mObjectLayer=jolt_world::kDynamic;
        part.mMotionType=JPH::EMotionType::Dynamic;
        part.mFriction=.72f;
        part.mLinearDamping=.25f;
        part.mAngularDamping=.45f;
        part.mNumVelocityStepsOverride=12;
        part.mNumPositionStepsOverride=5;
        part.mMotionQuality=JPH::EMotionQuality::LinearCast;
        part.mOverrideMassProperties=JPH::EOverrideMassProperties::CalculateInertia;
        part.mMassPropertiesOverride.mMass=spec.mass;
        part.mLinearVelocity=Vec(d.rootVelocity+d.velocity[i]);
        part.mAngularVelocity=Vec(d.spin[i]);
        poses[i]=JPH::Mat44::sRotationTranslation(part.mRotation,part.mPosition);
        if (spec.parent>=0) Joint(part,i,jolt_world::ToJolt(bone.pos),now,rest,forward);
    }
    articulation::Prepare(*settings,poses,.025f);
    d.body=settings->CreateRagdoll(nextGroup++,0,jolt_world::System());
    if (!d.body) return false;
    d.settings=settings;
    d.age=0;
    d.body->AddToPhysicsSystem(JPH::EActivation::Activate,false);
    for (int i=0;i<PartCount;++i)
        jolt_world::SetFloat(d.body->GetBodyID(i),i<4?1.07f:.88f,.6f,.072f);
    if (d.shot.remaining>0) {
        int closest=0;
        for (int i=1;i<PartCount;++i)
            if (Len(points[parts[i].frame].pos-d.shot.pos)<Len(points[parts[closest].frame].pos-d.shot.pos)) closest=i;
        const float strength=std::clamp(.053f*d.shot.damage,.5f,5.8f);
        const auto id=d.body->GetBodyID(closest);
        auto &physics=jolt_world::Bodies();
        const V3 centre=Vec(physics.GetCenterOfMassPosition(id));
        const V3 offset=jolt_world::ToJolt(d.shot.pos)-centre;
        // Native hit volumes can extend well beyond the animated mesh.
        const V3 point=centre+Norm(offset)*std::min(Len(offset),.18f);
        physics.AddImpulse(id,Vec(d.shot.dir*(strength*.64f)),Vec(point));
        physics.AddImpulse(d.body->GetBodyID(0),Vec(d.shot.dir*(strength*.16f)));
        physics.AddImpulse(d.body->GetBodyID(1),Vec(d.shot.dir*(strength*.20f)));
    }
    return true;
}
void Resist(Dog &d) {
    if (!d.body->IsActive(false)) return;
    const float release=std::max(0.f,1.f-d.age/.64f);
    for (int i=1;i<PartCount;++i) {
        const auto &part=parts[i];
        auto *joint=d.body->GetConstraint(d.settings->GetConstraintIndexForBodyIndex(i));
        jolt_ragdoll::SetPassiveResistance(*joint,{.06f*part.mass,.32f*part.mass},1.f);
        if (part.hinge && (release>0.f || d.releaseMotors)) {
            auto &hinge=static_cast<JPH::HingeConstraint &>(*joint);
            // Lose the extended stance under load. Only a short internal
            // flexion torque is applied, with no force on the whole body.
            const float bend=i<10 ? 66.f : 48.f;
            hinge.SetTargetAngle(bend*articulation::radians);
            hinge.GetMotorSettings().SetTorqueLimit(12.f*part.mass*release*release);
            hinge.SetMotorState(release>0.f ? JPH::EMotorState::Position : JPH::EMotorState::Off);
        }
    }
    d.releaseMotors=release>0.f;
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
    d.poseError=0.f;
    for (const auto &part:parts)
        d.poseError=std::max(d.poseError,Len(WorldPose(d.frames[part.frame]).pos-world[part.frame].pos));
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
            TrackMotion(dog,current,dt);
            dog.pose=std::move(current);
            dog.captured=true;
            const V3 centre=Centre(dog);
            if (Len(centre-player)<24.f)
                jolt_world::AddFocus(centre-V3{4,3,4},centre+V3{4,3,4});
            continue;
        }
        if (!dog.captured) {
            dog.pose=Capture(dog);
            dog.captured=true;
        }
        const V3 centre=Centre(dog);
        jolt_world::AddFocus(centre-V3{8,4,8},centre+V3{8,4,8});
        if (dog.body) {
            dog.age+=dt;
            Resist(dog);
        }
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
        const bool sleeping=dog.body && !dog.body->IsActive(false);
        float maxSpeed=0;
        if (dog.body) {
            auto &physics=jolt_world::Bodies();
            for (int i=0;i<PartCount;++i)
                maxSpeed=std::max(maxSpeed,physics.GetLinearVelocity(dog.body->GetBodyID(i)).Length());
        }
        result+=Fmt("{\"actor\":\"0x%08X\",\"bound\":%s,\"active\":%s,\"sleeping\":%s,\"parts\":%d,\"age\":%.3f,\"centre\":[%.4f,%.4f,%.4f],\"pose_error\":%.5f,\"max_speed\":%.4f}",
                    (unsigned)(uintptr_t)actor,dog.frames[0]?"true":"false",dog.body?"true":"false",sleeping?"true":"false",
                    dog.body?PartCount:0,dog.age,p.x,p.y,p.z,dog.poseError,maxSpeed);
    }
    return result+"]";
}
}
