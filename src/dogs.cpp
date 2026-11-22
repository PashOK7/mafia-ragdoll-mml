#include <windows.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "dogs.h"
#include "fmt.h"
#include "mml/game/effects.h"
#include "game.h"
#include "impact.h"
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
constexpr float kRecoveryTime=.58f;
constexpr float kDogShotGain=6.f, kDogImpulsePerDamage=.053f;
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
struct SavedPart { V3 position; Q rotation; V3 velocity,angularVelocity; };
struct SavedDog {
    uint32_t version=2,model=0,name=0,sleeping=0,bloodEffects=0,living=0;
    V3 nativePosition{};
    float age=0.f;
    Pose birth{};
    std::array<SavedPart,PartCount> parts{};
};
constexpr char kSaveKey[]="quadruped_pose";
constexpr std::size_t kMaxSavedDogs=512;
struct Dog {
    void *model = nullptr;
    std::array<void *, BoneCount> frames{};
    Pose pose{};
    std::array<Local, BoneCount> beforeRender{};
    std::array<Local, BoneCount> recoverFrom{};
    std::array<Link, PartCount> links{};
    JPH::Ref<JPH::RagdollSettings> settings;
    JPH::Ref<JPH::Ragdoll> body;
    std::array<Transform,BoneCount> rest{};
    std::array<V3,PartCount> velocity{}, spin{};
    std::array<V3,PartCount> pendingBlast{};
    V3 rootVelocity{};
    Shot shot{};
    bool captured=false, renderPose=false, releaseMotors=false;
    unsigned seen=0;
    float age=0, poseError=0;
    float knockTime=0.f, recovery=0.f, quiet=0.f;
    float shotBudgetUsed=0.f,lastShotImpulse=0.f;
    std::array<float,PartCount> localShotUsed{};
    uint32_t modelTag=0;
    unsigned corpseHits=0,bloodEffects=0,blastHits=0,wheelContacts=0;
    int lastHitPart=-1;
    bool restored=false;
};
std::unordered_map<void *, Dog> tracked;
std::unordered_map<std::string,std::array<Transform,BoneCount>> restByModel;
std::vector<SavedDog> pendingRestore;
unsigned tick;
JPH::CollisionGroup::GroupID nextGroup = 0x40000000u;
using DogAIFn=void (__fastcall *)(void *,void *,unsigned);
using DogHitFn=char (__fastcall *)(void *,void *,int,V3 *,V3 *,V3 *,float,void *,int,void *);
DogAIFn nativeAI;
DogHitFn nativeHit;
bool knockdownAvailable;

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
    return game::at<float>(actor,kHealth)<=0.f;
}

void __fastcall DogAI(void *actor,void *edx,unsigned elapsed) {
    const auto it=tracked.find(actor);
    // A live body holds native locomotion. A dead one lets the real death
    // clip complete and unregister its volumes, while stale path states stay held.
    if (it==tracked.end() || !it->second.body || game::at<int>(actor,kState)==kDead ||
        game::at<void *>(actor,game::kActorFrame)!=it->second.model)
        nativeAI(actor,edx,elapsed);
}

char __fastcall DogHit(void *actor,void *edx,int type,V3 *direction,V3 *at,V3 *hit,float damage,
                       void *shooter,int flags,void *frame) {
    const float before=game::at<float>(actor,kHealth);
    const char result=nativeHit(actor,edx,type,direction,at,hit,damage,shooter,flags,frame);
    const auto it=tracked.find(actor);
    if (it==tracked.end() || !it->second.captured || it->second.body || !knockdownAvailable) return result;
    auto &dog=it->second;
    const float remaining=game::at<float>(actor,kHealth);
    if ((type==0 || type==2) && remaining>0.f && before-remaining>=15.f) dog.knockTime=.35f;
    if (type==0 && direction && at && before>remaining && dog.shot.remaining<=0.f)
        dog.shot={*at,Norm(*direction),before-remaining,.25f};
    return result;
}
uint32_t Identity(const char *name) {
    if (!name || !*name) return 0;
    uint32_t hash=2166136261u;
    for (std::size_t i=0;i<512 && name[i];++i) {
        const unsigned char c=name[i];
        hash=(hash^(c>='A' && c<='Z' ? c+('a'-'A') : c))*16777619u;
    }
    return hash;
}
uint32_t NameOf(const Dog &dog) { return Identity(game::at<const char *>(dog.model,game::kFrameName)); }

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
    d.modelTag=Identity(file);
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
void Push(Dog &d,int part,V3 point,V3 impulse) {
    const auto id=d.body->GetBodyID(part);
    auto &physics=jolt_world::Bodies();
    const V3 centre=Vec(physics.GetCenterOfMassPosition(id));
    const V3 offset=jolt_world::ToJolt(point)-centre;
    // Native hit volumes can extend beyond the animated mesh.
    const V3 contact=centre+Norm(offset)*std::min(Len(offset),.18f);
    physics.AddImpulse(id,Vec(impulse*.64f),Vec(contact));
    physics.AddImpulse(d.body->GetBodyID(0),Vec(impulse*.16f));
    physics.AddImpulse(d.body->GetBodyID(1),Vec(impulse*.20f));
}

void PushSimulatedDog(Dog &dog,int part,V3 point,V3 direction,float baseImpulse) {
    // Reserve most of the kick for translation and bound a whole pellet
    // volley and each joint separately.
    constexpr float volleyLimit=36.f, localSpeedLimit=4.f;
    constexpr float totalMass=[] {
        float mass=0.f;
        for (const auto &segment:parts) mass+=segment.mass;
        return mass;
    }();
    const float impulse=std::min(baseImpulse*kDogShotGain,volleyLimit-dog.shotBudgetUsed);
    if (!(impulse>0.f)) return;
    dog.shotBudgetUsed+=impulse;
    dog.lastShotImpulse=dog.shotBudgetUsed;
    const float local=std::min(impulse*.30f,parts[part].mass*localSpeedLimit-dog.localShotUsed[part]);
    dog.localShotUsed[part]+=local;

    auto &physics=jolt_world::Bodies();
    const auto id=dog.body->GetBodyID(part);
    const auto centre=physics.GetCenterOfMassPosition(id);
    const V3 offset=jolt_world::ToJolt(point)-Vec(centre);
    const V3 contact=Vec(centre)+Norm(offset)*std::min(Len(offset),.07f);
    if (local>0.f) physics.AddImpulse(id,Vec(direction*local),Vec(contact));
    const auto whole=Vec(direction*((impulse-local)/totalMass));
    dog.body->AddLinearVelocity(whole,false);
}

bool Add(Dog &d,bool dead) {
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
        part.mLinearVelocity=Vec(d.rootVelocity+d.velocity[i]+d.pendingBlast[i]);
        part.mAngularVelocity=Vec(d.spin[i]);
        poses[i]=JPH::Mat44::sRotationTranslation(part.mRotation,part.mPosition);
        if (spec.parent>=0) Joint(part,i,jolt_world::ToJolt(bone.pos),now,rest,forward);
    }
    articulation::Prepare(*settings,poses,.025f);
    d.body=settings->CreateRagdoll(nextGroup++,0,jolt_world::System());
    if (!d.body) return false;
    d.settings=settings;
    d.age=0;
    d.quiet=0;
    d.body->AddToPhysicsSystem(JPH::EActivation::Activate,false);
    d.pendingBlast={};
    for (int i=0;i<PartCount;++i) {
        // The torso supplies enough displacement to support the limbs.
        // Near-neutral average buoyancy let a fall carry the whole dog under.
        const float buoyancy=i<2 ? 1.45f : i<4 ? 1.06f : .91f;
        jolt_world::SetFloat(d.body->GetBodyID(i),buoyancy,.85f,.072f);
    }
    if (d.shot.remaining>0) {
        int closest=0;
        for (int i=1;i<PartCount;++i)
            if (Len(points[parts[i].frame].pos-d.shot.pos)<Len(points[parts[closest].frame].pos-d.shot.pos)) closest=i;
        const float strength=std::clamp(kDogImpulsePerDamage*d.shot.damage,.5f,5.8f);
        if (dead && Dot(d.shot.dir,d.shot.dir)>.01f)
            PushSimulatedDog(d,closest,d.shot.pos,d.shot.dir,strength);
        else
            Push(d,closest,d.shot.pos,d.shot.dir*strength);
        d.shot={};
    }
    return true;
}

bool Finite(V3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
bool ValidRotation(Q q) {
    const float length=q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z;
    return length>.98f && length<1.02f;
}
bool Valid(const SavedDog &saved) {
    if (saved.version!=2 || !saved.model || saved.sleeping>1 || saved.living>1 || saved.bloodEffects>9 ||
        !Finite(saved.nativePosition) || !std::isfinite(saved.age) || saved.age<0.f) return false;
    for (int i=0;i<BoneCount;++i) {
        const auto &local=saved.birth.local[i];
        const auto &world=saved.birth.world[i];
        if (!Finite(world.pos) || Len(world.pos-saved.birth.world[Pelvis].pos)>4.f ||
            !Finite(local.pos) || !Finite(local.scale) || !ValidRotation(local.rot)) return false;
        for (V3 axis:world.basis.r) if (!(Len(axis)>.05f && Len(axis)<4.f)) return false;
    }
    for (const auto &part:saved.parts) {
        if (!Finite(part.position) || !ValidRotation(part.rotation) ||
            !(Len(part.velocity)<150.f && Len(part.angularVelocity)<150.f)) return false;
    }
    return true;
}
auto SavedFor(void *actor,const Dog &dog) {
    const V3 position=game::at<V3>(actor,game::kActorPos);
    const uint32_t name=NameOf(dog);
    const bool living=!Dead(actor);
    auto found=pendingRestore.end();
    for (auto it=pendingRestore.begin();it!=pendingRestore.end();++it) {
        if (it->model!=dog.modelTag || it->name!=name || (it->living!=0)!=living ||
            Len(position-it->nativePosition)>.12f) continue;
        // An ambiguous anonymous clone should keep its native load pose.
        if (found!=pendingRestore.end()) return pendingRestore.end();
        found=it;
    }
    return found;
}
void RestoreSaved(Dog &dog,const SavedDog &saved) {
    auto &physics=jolt_world::Bodies();
    if (saved.sleeping) physics.DeactivateBodies(dog.body->GetBodyIDs().data(),PartCount);
    for (int i=0;i<PartCount;++i) {
        const auto &part=saved.parts[i];
        const auto id=dog.body->GetBodyID(i);
        const auto &q=part.rotation;
        const JPH::Quat rotation(q.x,q.y,q.z,q.w);
        physics.SetPositionAndRotation(id,Vec(jolt_world::ToJolt(part.position)),rotation,JPH::EActivation::DontActivate);
        if (!saved.sleeping) physics.SetLinearAndAngularVelocity(id,Vec(part.velocity),Vec(part.angularVelocity));
    }
    dog.age=saved.age;
    dog.bloodEffects=saved.bloodEffects;
    dog.restored=true;
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
float Fastest(const Dog &d) {
    float speed=0.f;
    auto &physics=jolt_world::Bodies();
    for (int i=0;i<PartCount;++i)
        speed=std::max(speed,physics.GetLinearVelocity(d.body->GetBodyID(i)).Length());
    return speed;
}

void BeginRecovery(void *actor,Dog &dog) {
    const V3 centre=Centre(dog);
    const float from[]={centre.x,centre.y+.12f,centre.z},to[]={centre.x,centre.y-.85f,centre.z};
    MmlHit ground{};
    if (!mod::H->world_ray(mod::M,from,to,MML_WORLD_STATIC,nullptr,&ground) || ground.normal[1]<.64f) return;
    const V3 world{ground.pos[0],ground.pos[1],ground.pos[2]};
    float waterHeight=0.f;
    if (jolt_world::WaterAt(centre.x,centre.y,centre.z,&waterHeight,nullptr,nullptr) &&
        waterHeight>world.y+.10f) return;

    const V3 position=game::ToParentPoint(dog.model,world);
    const V3 chest=jolt_world::ToWorld(Vec(jolt_world::Bodies().GetPosition(dog.body->GetBodyID(1))));
    const V3 towardsChest=game::ToParentPoint(dog.model,chest)-game::ToParentPoint(dog.model,centre);
    const V3 forward=Norm({towardsChest.x,0,towardsChest.z},game::at<V3>(actor,game::kActorFacing));
    const V3 up{0,1,0};
    game::at<V3>(actor,game::kActorPos)=position;
    game::at<V3>(actor,game::kActorFacing)=forward;
    game::at<V3>(actor,0xb8)=forward; // C_dog::Update's requested direction.
    game::SetLocalPosition(dog.model,position);
    game::SetLocalRotation(dog.model,QuatOf(M3{{Cross(up,forward),up,forward}}));
    Update(dog.model);
    Apply(dog);
    for (int i=0;i<BoneCount;++i) dog.recoverFrom[i]=LocalPose(dog.frames[i]);
    Remove(dog);
    dog.recovery=kRecoveryTime;
    dog.captured=false;
    dog.shot={};
    dog.knockTime=0.f;

    using Choose=char (__thiscall *)(void *,int,int);
    using SetAnim=void (__thiscall *)(void *,int);
    reinterpret_cast<Choose>(game::Exe().dogChooseAction)(actor,0xd,0);
    reinterpret_cast<SetAnim>(game::Exe().dogSetAnim)(actor,0);
}

void ApplyRecovery(Dog &dog) {
    const float t=std::clamp(1.f-dog.recovery/kRecoveryTime,0.f,1.f);
    const float weight=t*t*(3.f-2.f*t);
    for (int i=0;i<BoneCount;++i) {
        Local target=LocalPose(dog.frames[i]);
        dog.beforeRender[i]=target;
        const Local &from=dog.recoverFrom[i];
        const Q a=from.rot,b=target.rot;
        const auto q=JPH::Quat(a.x,a.y,a.z,a.w).SLERP(JPH::Quat(b.x,b.y,b.z,b.w),weight);
        target.pos=from.pos+(target.pos-from.pos)*weight;
        target.rot={q.GetW(),q.GetX(),q.GetY(),q.GetZ()};
        Write(dog.frames[i],target);
    }
    dog.renderPose=true;
    Update(dog.model);
}
}

void Start() {
    const auto ai=game::Exe().dogAI, hit=game::Exe().dogHit;
    const auto *host=mod::H;
    const bool haveAI=host->hook(mod::M,ai,reinterpret_cast<void *>(&DogAI),reinterpret_cast<void **>(&nativeAI));
    knockdownAvailable=haveAI &&
        host->hook(mod::M,hit,reinterpret_cast<void *>(&DogHit),reinterpret_cast<void **>(&nativeHit));
    if (!knockdownAvailable) {
        if (haveAI) host->unhook(mod::M,ai);
        mod::Warn("dog knockdown handover unavailable");
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
        dog.shotBudgetUsed=0.f;
        dog.localShotUsed={};
        const bool dead=Dead(actor);
        if (dog.recovery>0.f) {
            if (!dead) {
                dog.recovery=std::max(0.f,dog.recovery-dt);
                dog.knockTime=0.f;
                if (dog.recovery>0.f) continue;
            } else {
                ApplyRecovery(dog);
                dog.pose=Capture(dog);
                Restore(dog);
                dog.captured=true;
                dog.recovery=0.f;
            }
        }
        const auto saved=dog.body ? pendingRestore.end() : SavedFor(actor,dog);
        const bool restoring=saved!=pendingRestore.end() && (dead || knockdownAvailable);
        if (!dead && !dog.body && !restoring && dog.knockTime<=0.f) {
            dog.shot.remaining-=dt;
            dog.corpseHits=0;
            dog.lastHitPart=-1;
            dog.lastShotImpulse=0.f;
            dog.bloodEffects=dog.blastHits=0;
            dog.restored=false;
            dog.pendingBlast={};
            Pose current=Capture(dog);
            TrackMotion(dog,current,dt);
            dog.pose=std::move(current);
            dog.captured=true;
            const V3 centre=Centre(dog);
            if (Len(centre-player)<24.f)
                jolt_world::AddFocus(centre-V3{4,3,4},centre+V3{4,3,4});
            continue;
        }
        if (!dog.captured || (!dog.body && !restoring &&
            Len(game::ActorWorldPos(actor)-dog.pose.world[Pelvis].pos)>2.f)) {
            // A teleported actor can be hit before another living pose was sampled.
            // Use the model at contact and discard velocity from the old location.
            dog.pose=Capture(dog);
            dog.rootVelocity={};
            dog.velocity={};
            dog.spin={};
            dog.captured=true;
        }
        const V3 centre=restoring ? saved->parts[0].position : Centre(dog);
        jolt_world::AddFocus(centre-V3{8,4,8},centre+V3{8,4,8});
        if (dog.body) {
            dog.age+=dt;
            Resist(dog);
            if (!dead && dog.age>.65f) {
                dog.quiet=Fastest(dog)<.38f ? dog.quiet+dt : 0.f;
                if (dog.quiet>.28f) BeginRecovery(actor,dog);
            }
        }
        else {
            float waterHeight=0.f;
            const bool openWater=jolt_world::WaterAt(centre.x,centre.y,centre.z,&waterHeight,nullptr,nullptr) &&
                centre.y<waterHeight+2.f;
            if (!openWater && !jolt_world::StaticReady(centre-V3{1,2,1},centre+V3{1,1,1})) {
                dog.knockTime=std::max(0.f,dog.knockTime-dt);
                continue;
            }
            if (restoring) dog.pose=saved->birth;
            if (Add(dog,dead)) {
                dog.knockTime=0.f;
                if (restoring) {
                    RestoreSaved(dog,*saved);
                    pendingRestore.erase(saved);
                }
                mod::Log("quadruped ragdoll %p: %d parts%s",actor,PartCount,restoring?" restored":"");
            }
        }
        dog.knockTime=std::max(0.f,dog.knockTime-dt);
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
void FinishStep() {
    for (const auto &contact:jolt_world::CarContacts()) {
        if (contact.sensor || !contact.wheel) continue;
        for (auto &[_,dog]:tracked) {
            if (!dog.body) continue;
            const auto &ids=dog.body->GetBodyIDs();
            if (std::find(ids.begin(),ids.end(),JPH::BodyID(contact.body))==ids.end()) continue;
            ++dog.wheelContacts;
            break;
        }
    }
}
void Bullet(const MmlBulletHit &hit) {
    const auto it=tracked.find(hit.actor);
    if (it==tracked.end() || it->second.body) return;
    it->second.shot={{hit.pos.x,hit.pos.y,hit.pos.z},{hit.dir.x,hit.dir.y,hit.dir.z},hit.damage,.25f};
}
unsigned Blast(V3 centre,float radius,float speed,float lift) {
    unsigned thrown=0;
    for (auto &[actor,dog]:tracked) {
        if (!dog.captured || (!dog.body && !Dead(actor) && dog.knockTime<=0.f) ||
            Len(Centre(dog)-centre)>radius+1.5f) continue;
        auto &physics=jolt_world::Bodies();
        const auto posed=dog.body ? std::array<Transform,PartCount>{} : PartPose(dog.pose.world);
        bool affected=false;
        for (int i=0;i<PartCount;++i) {
            const auto id=dog.body ? dog.body->GetBodyID(i) : JPH::BodyID();
            const V3 at=dog.body ? jolt_world::ToWorld(Vec(physics.GetPosition(id))) : posed[i].pos;
            const V3 delta=BlastVelocity(at,centre,radius,speed,lift);
            if (!(Dot(delta,delta)>0.f)) continue;
            if (dog.body) physics.AddLinearVelocity(id,Vec(delta));
            else dog.pendingBlast[i]+=delta;
            affected=true;
        }
        if (affected) {
            ++dog.blastHits;
            ++thrown;
        }
    }
    return thrown;
}
std::size_t AppendShotExclusions(void **objects,std::size_t count,std::size_t capacity) {
    // C_dog::Death removes these two volumes at the end of the native death clip.
    constexpr int nativeVolumes[]={0x234,0x238};
    for (const auto &[actor,dog]:tracked) {
        if (!dog.body) continue;
        for (int offset:nativeVolumes) {
            if (count==capacity) return count;
            void *volume=game::at<void *>(actor,offset);
            if (volume && std::find(objects,objects+count,volume)==objects+count)
                objects[count++]=volume;
        }
    }
    return count;
}
bool TraceShot(V3 from,V3 axis,float limit,ShotHit &hit) {
    for (const auto &[actor,dog]:tracked) {
        if (!dog.body) continue;
        const V3 to=Centre(dog)-from;
        if (Len(to-axis*std::clamp(Dot(to,axis),0.f,limit))>2.f) continue;
        V3 point{};
        const int part=articulation::ShotPart(*dog.body,from,axis*limit,&point);
        if (part<0) continue;
        limit=Len(point-from);
        hit={actor,part,point};
        if (!(limit>0.f)) break;
    }
    return hit.actor!=nullptr;
}
void HitBody(const ShotHit &hit,V3 direction,float strength,float damage,void *shooter) {
    auto &dog=tracked.at(hit.actor);
    PushSimulatedDog(dog,hit.part,hit.point,direction,strength*(kDogImpulsePerDamage*.5f));
    if (!Dead(hit.actor) && nativeHit) {
        V3 point=hit.point;
        dog.quiet=0.f;
        nativeHit(hit.actor,nullptr,0,&direction,&point,nullptr,damage,shooter,0,nullptr);
        return;
    }
    ++dog.corpseHits;
    dog.lastHitPart=hit.part;
    if (dog.bloodEffects<9 && game::EmitBloodImpact(hit.point,direction*3.f,.7f))
        ++dog.bloodEffects;
}
void DrawPose() {
    for (auto &[actor,dog]:tracked) {
        if (game::at<void *>(actor,game::kActorFrame)!=dog.model) continue;
        if (dog.body) Apply(dog);
        else if (dog.recovery>0.f) ApplyRecovery(dog);
    }
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
    pendingRestore.clear();
}
void Save() {
    std::vector<SavedDog> saved=pendingRestore;
    for (const auto &[actor,dog]:tracked) {
        if (saved.size()>=kMaxSavedDogs) break;
        if (!dog.body) continue;
        SavedDog entry{};
        entry.model=dog.modelTag;
        entry.name=NameOf(dog);
        entry.nativePosition=game::at<V3>(actor,game::kActorPos);
        entry.sleeping=!dog.body->IsActive(false);
        entry.living=!Dead(actor);
        entry.bloodEffects=dog.bloodEffects;
        entry.age=dog.age;
        entry.birth=dog.pose;
        auto &physics=jolt_world::Bodies();
        for (int i=0;i<PartCount;++i) {
            const auto id=dog.body->GetBodyID(i);
            JPH::RVec3 position;
            JPH::Quat rotation;
            physics.GetPositionAndRotation(id,position,rotation);
            entry.parts[i]={jolt_world::ToWorld(Vec(position)),
                {rotation.GetW(),rotation.GetX(),rotation.GetY(),rotation.GetZ()},
                Vec(physics.GetLinearVelocity(id)),Vec(physics.GetAngularVelocity(id))};
        }
        if (Valid(entry)) saved.push_back(entry);
    }
    if (!mod::H->save_put(mod::M,MML_SAVE_SLOT,kSaveKey,saved.empty()?nullptr:saved.data(),saved.size()*sizeof(SavedDog)))
        mod::Warn("could not save quadruped poses");
}
void Load() {
    const auto size=mod::H->save_get(mod::M,MML_SAVE_SLOT,kSaveKey,nullptr,0);
    if (!size || size%sizeof(SavedDog) || size/sizeof(SavedDog)>kMaxSavedDogs) return;
    pendingRestore.resize(size/sizeof(SavedDog));
    if (mod::H->save_get(mod::M,MML_SAVE_SLOT,kSaveKey,pendingRestore.data(),size)!=size) pendingRestore.clear();
    std::erase_if(pendingRestore,[](const SavedDog &saved){return !Valid(saved);});
}
std::string State() {
    std::string result="[";
    for (const auto &[actor,dog]:tracked) {
        if (result.size()>1) result+=",";
        const V3 p=Centre(dog);
        const bool sleeping=dog.body && !dog.body->IsActive(false);
        const float maxSpeed=dog.body ? Fastest(dog) : 0.f;
        result+=Fmt("{\"actor\":\"0x%08X\",\"bound\":%s,\"active\":%s,\"sleeping\":%s,\"parts\":%d,\"age\":%.3f,\"centre\":[%.4f,%.4f,%.4f],\"pose_error\":%.5f,\"max_speed\":%.4f,\"corpse_hits\":%u,\"last_hit_part\":\"%s\",\"last_shot_impulse\":%.3f,\"blood_effects\":%u,\"blast_hits\":%u,\"wheel_contacts\":%u,\"restored\":%s,\"recovering\":%s,\"dead\":%s}",
                    (unsigned)(uintptr_t)actor,dog.frames[0]?"true":"false",dog.body?"true":"false",sleeping?"true":"false",
                    dog.body?PartCount:0,dog.age,p.x,p.y,p.z,dog.poseError,maxSpeed,
                    dog.corpseHits,dog.lastHitPart<0?"":parts[dog.lastHitPart].name,dog.lastShotImpulse,
                    dog.bloodEffects,dog.blastHits,dog.wheelContacts,dog.restored?"true":"false",
                    dog.recovery>0.f?"true":"false",Dead(actor)?"true":"false");
    }
    return result+"]";
}
}
