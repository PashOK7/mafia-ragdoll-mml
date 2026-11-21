#include <windows.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include "headwear.h"
#include "hat_model.h"
#include "game.h"
#include "mod.h"
#include "ragdoll.h"
#include "jolt_world.h"
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace headwear {
namespace {
using namespace mod;
namespace fs = std::filesystem;
// Archive character names with a separate hat or cap material. Each installed
// model must also pass the face and skin checks before its virtual override exists.
const char *candidates =
    "alice1;asispz1;bguard03;bguard03m;biff;bigdig;bobaut01;bookmaker01;bookmaker02;china1;"
    "civil02;civil03;civil04;civil05;civil06;civil11;civil11m;civil12;civil13;civil14;civil15;civil16;civil16m;"
    "civil17;civil18;civil19;civil19m;civil31;civil35;civil36;civil36m;civil37;civil38;civil39;civil40;civil41;"
    "civil43;civil44;civil51;civil51m;civil52;civil53;civil54;civil54m;civil60;civil61;civil62;civil63;civil71;"
    "civil72;czena08;delnik01;delnik02;delnik03;detektiv01;detektiv02;detektiv03;enemy01+;enemy02+;enemy03+;"
    "enemy04k;enemy06+;enemy07+;enemy08+;enemy08k;enemy09+;enemy09;enemy09k;enemy10+;enemy10;enemy10k;enemy11k;"
    "enemy12k;enemy13c;enemy91;enemy92;fmvenemy11k;freegang01;freegang02;friend1;gangster02;gangster04;gangster05;"
    "homeless01;hoolig04;hoolig06;kasar;lucas;march1;milenka1;morellohigh;morellolow;muz zam;organizator01;"
    "pauliecoathat;pauliecthigh;portguard01;portguard02;prokur;pumpar01;radni01;ridicnakladaku;salman01k;"
    "salman02k;salman03k;salman05k;samcoathat;synrad1;synrad1blood;synrad1dead;tommycoat;tommycoathat;"
    "tommydelnik;tommydelnikhigh;tommyhat;tommyhict bezt;tommyhighcoathat;tommyhighhat;tommytaxidriver;"
    "tommytaxidriverhigh;tony;vrabec;vratny1;vypravci2;willg2;willman01;willman02;zenarad01";
struct HatModel {
    V3 dimensions{};
    hats::PropMesh mesh;
    JPH::RefConst<JPH::Shape> hull;
};
std::unordered_map<std::string,HatModel> ready;
bool prepared = false;
struct FallenHat {
    void *prop, *pedestrian, *model;
    bool wasDead;
    JPH::BodyID body;
    V3 previous, drawn;
    const hats::PropMesh *mesh;
    float scale;
    bool touchedRagdoll=false;
    bool reportedGround=false;
    bool sleeping=false;
};
std::unordered_map<void *, FallenHat> fallen;
std::vector<FallenHat> retired;
constexpr float hatMass=.22f;
JPH::Vec3 J(V3 v) { return {v.x,v.y,v.z}; }
V3 F(JPH::RVec3Arg v) { return {float(v.GetX()),float(v.GetY()),float(v.GetZ())}; }

float Support(const FallenHat &hat,JPH::QuatArg rotation,V3 normal) {
    const V3 local=F(rotation.Conjugated()*J(normal));
    float reach=0.f;
    for (const auto &vertex:hat.mesh->vertices) reach=std::max(reach,-Dot(local,vertex));
    return reach*hat.scale;
}

bool StaticFloor(V3 from,V3 to,MmlHit &hit) {
    return H->world_ray(M,&from.x,&to.x,MML_WORLD_STATIC,nullptr,&hit) &&
           hit.normal[1]>.5f && (hit.flags>>24)!=31 && (hit.flags>>24)!=44;
}

float DrawClearance(const FallenHat &hat,V3 pos,JPH::QuatArg q) {
    const float low=pos.y-Support(hat,q,{0,1,0});
    MmlHit ground{};
    if (!StaticFloor(pos+V3{0,.025f,0},{pos.x,low-.045f,pos.z},ground)) return 0;
    constexpr float gap=.004f;
    const V3 normal{ground.normal[0],ground.normal[1],ground.normal[2]};
    const V3 surface{ground.pos[0],ground.pos[1],ground.pos[2]};
    float lift=(Support(hat,q,normal)+gap-Dot(pos-surface,normal))/normal.y;

    // At a slope change the central ray's plane can miss a low edge.
    // Probe the lowest rendered point in each quadrant of the footprint.
    std::array<V3,4> lowest{};
    std::array<bool,4> found{};
    for (const auto &v:hat.mesh->vertices) {
        const V3 offset=F(q*J(v))*hat.scale;
        const unsigned quadrant=(offset.x>=0 ? 1u : 0u)+(offset.z>=0 ? 2u : 0u);
        if (!found[quadrant] || offset.y<lowest[quadrant].y) {
            found[quadrant]=true;
            lowest[quadrant]=offset;
        }
    }
    for (size_t i=0;i<lowest.size();++i) {
        if (!found[i]) continue;
        const V3 point=pos+lowest[i];
        if (StaticFloor(point+V3{0,.042f,0},point-V3{0,.012f,0},ground))
            lift=std::max(lift,ground.pos[1]+gap/ground.normal[1]-point.y);
    }
    return std::clamp(lift,0.f,.05f);
}

bool ClearRelease(const JPH::Shape *shape,JPH::QuatArg rotation,float depth,V3 &position) {
    const V3 initial=position;
    const auto &query=jolt_world::System()->GetNarrowPhaseQueryNoLock();
    JPH::CollideShapeSettings settings;
    for (int i=0;i<12;++i) {
        const auto transform=JPH::RMat44::sRotationTranslation(rotation,J(jolt_world::ToJolt(position)))
                                 .PreTranslated(shape->GetCenterOfMass());
        JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> hits;
        query.CollideShape(shape,JPH::Vec3::sOne(),transform,settings,JPH::RVec3::sZero(),hits);
        const JPH::CollideShapeResult *deepest=nullptr;
        float penetration=.002f;
        for (const auto &hit:hits.mHits) {
            if (hit.mPenetrationDepth>penetration) {
                deepest=&hit;
                penetration=hit.mPenetrationDepth;
            }
        }
        if (!deepest) return true;
        position-=F(deepest->mPenetrationAxis.NormalizedOr(JPH::Vec3::sAxisY()))*(penetration+.002f);
        if (Len(position-initial)>depth) return false;
    }
    return false;
}

void Remove(const FallenHat &hat) {
    if (jolt_world::Running()) {
        auto &bi=jolt_world::Bodies();
        bi.RemoveBody(hat.body);
        bi.DestroyBody(hat.body);
    }
    H->prop_remove(M,hat.prop);
}

fs::path utf8path(const std::string &s) {
    return fs::path(std::u8string(s.begin(),s.end()));
}
bool otherProvider(const std::string &name, const std::string &ours) {
    char path[2048]{};
    return H->file_source(name.c_str(),path,sizeof path) && utf8path(path)!=utf8path(ours);
}
bool save(const std::string &name, const hats::Bytes &bytes) {
    const fs::path path = utf8path(name);
    std::ifstream old(path, std::ios::binary | std::ios::ate);
    if (old && old.tellg() == std::streamoff(bytes.size())) {
        hats::Bytes previous(bytes.size());
        old.seekg(0);
        if (old.read(reinterpret_cast<char *>(previous.data()), bytes.size()) && previous == bytes) return true;
    }
    old.close();
    fs::path tmp = path;
    tmp += L".tmp";
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
    out.close();
    if (out && MoveFileExW(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING)) return true;
    DeleteFileW(tmp.c_str());
    return false;
}
void show(void *frame, bool on) {
    using SetOn = void (__stdcall *)(void *, bool);
    reinterpret_cast<SetOn>((*reinterpret_cast<void ***>(frame))[0x24/4])(frame,on);
}
void gone(void *frame, void *) {
    auto it = fallen.find(frame);
    if (it == fallen.end()) return;
    retired.push_back(it->second);
    fallen.erase(it);
}
}

void Prepare() {
    if (prepared || H->file_read(M,"models\\civil15.4ds",nullptr,0,0)<=0) return;
    prepared = true;
    const char *base = H->mod_data_dir(M);
    if (!base) return;
    const std::string dir = std::string(base)+"headwear\\";
    std::error_code error;
    fs::create_directories(utf8path(dir),error);
    if (error) { Warn("cannot create headwear cache"); return; }
    const std::string lining=dir+"rd_lining.bmp";
    if (otherProvider("maps\\rd_lining.bmp",lining) || !save(lining,hats::liningBitmap()) ||
        !H->file_add(M,"maps\\rd_lining.bmp",lining.c_str())) {
        Warn("cannot register hat lining texture");
        return;
    }
    std::istringstream names(candidates);
    std::string name;
    while (std::getline(names,name,';')) {
        const auto gameName = "models\\"+name+".4ds";
        const std::string personPath = dir+name+".4ds";
        if (otherProvider(gameName,personPath)) continue;
        auto size = H->file_read(M,gameName.c_str(),nullptr,0,MML_FILE_GAME);
        if (size <= 0 || size > 16*1024*1024) continue;
        hats::Bytes source(static_cast<size_t>(size)), person, prop;
        if (H->file_read(M,gameName.c_str(),source.data(),source.size(),MML_FILE_GAME)!=size) continue;
        std::string marker = "rdhat_"+name;
        HatModel model;
        if (!hats::split(source,marker+"_scalp",person,prop,model.dimensions,model.mesh)) continue;
        const std::string propPath = dir+marker+"_lined.4ds";
        const std::string propName = "models\\"+marker+"_lined.4ds";
        if (otherProvider(propName,propPath) || !save(propPath,prop) || !save(personPath,person)) continue;
        if (!H->file_add(M,propName.c_str(),propPath.c_str())) continue;
        if (!H->file_add(M,gameName.c_str(),personPath.c_str())) {
            H->file_add(M,propName.c_str(),nullptr);
            continue;
        }
        ready.emplace(marker,std::move(model));
    }
    Log("detachable headwear prepared for %u installed character models",unsigned(ready.size()));
}

void Tick(std::span<const MmlPedestrian> pedestrians) {
    if (!prepared) Prepare();
    for (auto it=fallen.begin();it!=fallen.end();) {
        auto &[frame,hat]=*it;
        if (!hat.pedestrian) { ++it; continue; }
        auto p=std::find_if(pedestrians.begin(),pedestrians.end(),[&](const auto &person){
            return person.element==hat.pedestrian && person.model==hat.model && person.spawned;
        });
        if (p!=pedestrians.end() && !(hat.wasDead && p->alive)) {
            hat.wasDead=!p->alive;
            ++it;
            continue;
        }
        // The traffic manager can return a still-live frame to the model pool.
        // Restore only the visual we hid before another pedestrian inherits it.
        H->frame_watch(M,frame,nullptr,nullptr);
        show(frame,true);
        retired.push_back(hat);
        it=fallen.erase(it);
    }
    for (const auto &hat:retired) Remove(hat);
    retired.clear();
}

void Sync() {
    if (fallen.empty() || !jolt_world::Running()) return;
    auto &bi=jolt_world::Bodies();
    for (auto &[_,hat]:fallen) {
        const bool asleep=!bi.IsActive(hat.body);
        if (asleep && hat.sleeping) continue;
        hat.sleeping=asleep;
        JPH::RVec3 p;
        JPH::Quat q;
        bi.GetPositionAndRotation(hat.body,p,q);
        V3 pos=jolt_world::ToWorld(F(p));
        if (pos.y<hat.previous.y) {
            // The nearby visual-mesh cache does not contain every native floor.
            // Check the path before the render transform is committed.
            const float reach=Support(hat,q,{0,1,0});
            const V3 from=hat.previous+V3{0,.025f,0};
            const V3 to=pos-V3{0,reach,0};
            MmlHit ground{};
            if (StaticFloor(from,to,ground)) {
                V3 n{ground.normal[0],ground.normal[1],ground.normal[2]};
                const float support=Support(hat,q,n);
                const V3 surface{ground.pos[0],ground.pos[1],ground.pos[2]};
                const float penetration=support-Dot(pos-surface,n);
                if (penetration>.008f) {
                    pos+=n*penetration;
                    auto velocity=bi.GetLinearVelocity(hat.body);
                    const auto normal=J(n);
                    velocity-=normal*std::min(0.f,velocity.Dot(normal));
                    bi.SetPosition(hat.body,J(jolt_world::ToJolt(pos)),JPH::EActivation::DontActivate);
                    bi.SetLinearVelocity(hat.body,velocity);
                    if (!hat.reportedGround) {
                        hat.reportedGround=true;
                        Log("headwear native ground support at %.3f %.3f %.3f, correction %.3f m",
                            surface.x,surface.y,surface.z,penetration);
                    }
                }
            }
        }
        hat.previous=pos;
        // Leave the sleeping body and its contact manifold in Jolt. Move
        // only the rendered surface clear of the asphalt's depth plane.
        hat.drawn=pos+V3{0,DrawClearance(hat,pos,q),0};
        const Q rotation{q.GetW(),-q.GetX(),-q.GetY(),-q.GetZ()};
        H->prop_place(M,hat.prop,&hat.drawn.x,&rotation.w,0);
    }
}

bool Near(V3 point) {
    if (fallen.empty() || !jolt_world::Running()) return false;
    auto &bi=jolt_world::Bodies();
    for (const auto &[_,hat]:fallen) {
        const V3 delta=jolt_world::ToWorld(F(bi.GetPosition(hat.body)))-point;
        if (Dot(delta,delta)<9.f) return true;
    }
    return false;
}

bool OwnsBody(uint32_t body) {
    return std::any_of(fallen.begin(),fallen.end(),[body](const auto &entry){
        return entry.second.body.GetIndexAndSequenceNumber()==body;
    });
}

void ApplyAirDrag(float dt) {
    if (fallen.empty() || !jolt_world::Running() || !(dt>0)) return;
    auto &bi=jolt_world::Bodies();
    for (const auto &[_,hat]:fallen) {
        if (!bi.IsActive(hat.body)) continue;
        const auto velocity=bi.GetLinearVelocity(hat.body);
        const V3 p=jolt_world::ToWorld(F(bi.GetPosition(hat.body)));
        jolt_world::AddFocus(p-V3{3,3,3},p+V3{3,2,3});
        // Integrate quadratic drag analytically so a fast hat sheds speed
        // rapidly while a foot can still slide it along the ground.
        constexpr float dragPerMetre=.32f;
        bi.SetLinearVelocity(hat.body,velocity/(1.f+dragPerMetre*velocity.Length()*dt));
    }
}

bool Shot(V3 from,V3 axis,float range,V3 *point,uint32_t *body) {
    if (fallen.empty() || !jolt_world::Running()) return false;
    auto &bi=jolt_world::Bodies();
    bool found=false;
    for (const auto &[_,hat]:fallen) {
        JPH::RVec3 p;
        JPH::Quat q;
        bi.GetPositionAndRotation(hat.body,p,q);
        V3 delta=hat.drawn-from;
        const V3 nearest=delta-axis*std::clamp(Dot(delta,axis),0.f,range);
        const float radius=hat.scale*hat.mesh->radius;
        if (Dot(nearest,nearest)>radius*radius) continue;
        const V3 origin=F(q.Conjugated()*J(-delta))*(1.f/hat.scale);
        const V3 dir=F(q.Conjugated()*J(axis))*(1.f/hat.scale);
        const auto &mesh=*hat.mesh;
        for (size_t i=0;i<mesh.indices.size();i+=3) {
            const V3 a=mesh.vertices[mesh.indices[i]];
            const V3 e=mesh.vertices[mesh.indices[i+1]]-a,f=mesh.vertices[mesh.indices[i+2]]-a;
            const V3 perpendicular=Cross(dir,f);
            const float det=Dot(e,perpendicular);
            if (std::abs(det)<1e-8f) continue;
            const V3 offset=origin-a;
            const float u=Dot(offset,perpendicular)/det;
            if (u<0 || u>1) continue;
            const V3 across=Cross(offset,e);
            const float v=Dot(dir,across)/det;
            const float distance=Dot(f,across)/det;
            if (v<0 || u+v>1 || distance<0 || distance>=range) continue;
            range=distance;
            *body=hat.body.GetIndexAndSequenceNumber();
            found=true;
        }
    }
    if (!found) return false;
    *point=from+axis*range;
    return true;
}

void BulletImpulse(uint32_t body,V3 direction,V3 point,float damage) {
    auto hat=std::find_if(fallen.begin(),fallen.end(),[body](const auto &entry){
        return entry.second.body.GetIndexAndSequenceNumber()==body;
    });
    if (hat==fallen.end()) return;
    auto &bi=jolt_world::Bodies();
    auto id=hat->second.body;
    JPH::RVec3 p;
    JPH::Quat q;
    bi.GetPositionAndRotation(id,p,q);
    const V3 centre=jolt_world::ToWorld(F(p));
    constexpr float response=2.f;
    const float strength=std::min(.12f,damage*.002f)*response;
    V3 impulse=direction*strength;
    float reboundSpeed=0.f;
    const float below=Support(hat->second,q,{0,1,0});
    const V3 end=centre-V3{0,below+.035f,0};
    MmlHit ground{};
    V3 supportNormal{};
    if (StaticFloor(centre,end,ground)) {
        supportNormal={ground.normal[0],ground.normal[1],ground.normal[2]};
    } else {
        // A hat can rest on the Jolt surface of a corpse or vehicle.
        const JPH::RRayCast ray{p,J(end-centre)};
        JPH::RayCastResult hit;
        JPH::IgnoreSingleBodyFilter self(id);
        if (jolt_world::System()->GetNarrowPhaseQueryNoLock().CastRay(ray,hit,{},{},self) &&
            hit.mFraction>.001f) {
            supportNormal=F(bi.GetTransformedShape(hit.mBodyID).GetWorldSpaceSurfaceNormal(
                hit.mSubShapeID2,ray.GetPointOnRay(hit.mFraction)));
        }
    }
    const bool supported=supportNormal.y>.5f;
    if (supported) {
        const V3 n=supportNormal;
        const float compression=std::max(0.f,-Dot(direction,n));
        const V3 offset=point-centre;
        const V3 radial=offset-n*Dot(offset,n);
        const V3 tangent=direction-n*Dot(direction,n);
        V3 slide=tangent+Norm(-radial,{})*(.85f*compression);
        impulse=slide*(strength/std::max(1.f,Len(slide)));
        bi.AddImpulse(id,J(impulse),J(jolt_world::ToJolt(point)));

        // A compressed hat springs off the floor. Set the outgoing normal
        // speed rather than adding it to the previous hit's rebound.
        const auto normal=J(n);
        const auto velocity=bi.GetLinearVelocity(id);
        const float outgoing=velocity.Dot(normal);
        reboundSpeed=std::min(1.5f,damage*.045f)*compression*response;
        const V3 side=Norm(slide,{});
        const float sideSpeed=std::min(.9f,damage*.026f)*std::min(1.f,Len(slide))*response;
        const float sideGain=std::max(0.f,sideSpeed-velocity.Dot(J(side)));
        bi.SetLinearVelocity(id,velocity+normal*std::max(0.f,reboundSpeed-outgoing)+J(side*sideGain));
    } else {
        bi.AddImpulse(id,J(impulse),J(jolt_world::ToJolt(point)));
    }
    Log("headwear bullet impulse %.3f Ns at %.3f %.3f %.3f support %d rebound %.3f m/s",
        Len(impulse),point.x,point.y,point.z,supported,reboundSpeed);
}

void BodyContact(uint32_t propBody) {
    for (auto &[_,hat]:fallen) {
        if (hat.body.GetIndexAndSequenceNumber()!=propBody || hat.touchedRagdoll) continue;
        hat.touchedRagdoll=true;
        Log("headwear collided with ragdoll");
        break;
    }
}

void Drop(Ragdoll &body, float dt) {
    if (body.headwearChecked || !body.model || ready.empty()) return;
    V3 base,tip;
    float radius;
    body.BodyCapsule(Ragdoll::B_HEAD,&base,&tip,&radius);
    V3 up=Norm(tip-base);
    const V3 torso=(body.BodyPos(Ragdoll::B_CHEST)+body.BodyPos(Ragdoll::B_PELVIS))*.5f;
    auto &fit=body.hatFit;
    if (fit.tracking) {
        if (!(dt>0)) return;
        fit.track(body.HeadWorldPoint(fit.mount),torso,dt);
        if (!fit.slips(up,fit.hatMotion,dt)) return;
        // Test the native collision volume too: native props and the Jolt
        // ragdoll can see different vehicle geometry in the same place.
        constexpr float brim=.18f;
        V3 extent{brim,brim,brim}, lo=fit.lastHat-extent, hi=fit.lastHat+extent;
        MmlShape shapes[32];
        int count=H->world_shapes(M,&lo.x,&hi.x,MML_WORLD_MOVING|MML_WORLD_NO_PEOPLE,shapes,32);
        for (int i=0;i<std::min(count,32);++i) {
            const auto &s=shapes[i];
            if (!(s.classes&0x200)) continue;
            V3 d=fit.lastHat-V3{s.center[0],s.center[1],s.center[2]};
            if (s.kind==MML_SHAPE_SPHERE && Len(d)<=brim+s.half[0]) return;
            if (s.kind==MML_SHAPE_BOX) {
                float squared=0;
                for (int k=0;k<3;++k) {
                    const float *a=s.axes+k*3;
                    float gap=std::max(0.f,std::abs(Dot(d,{a[0],a[1],a[2]}))-s.half[k]);
                    squared+=gap*gap;
                }
                if (squared<=brim*brim) return;
            }
        }
    }
    body.headwearChecked = true;
    void *neck = game::FindChild(body.model,"neck");
    if (!neck) return;
    std::vector<void *> children;
    game::EnumFrames(neck,&children);
    for (void *frame : children) {
        const char *full = game::at<const char *>(frame,game::kFrameName);
        if (!full) continue;
        if (game::at<void *>(frame,game::kFrameParent)!=neck) continue;
        const char *dot = strrchr(full,'.');
        std::string name = hats::lower(dot ? dot+1 : full);
        // A wearer still in Mafia's cache from 0.1.15 has an open crown.
        if (!name.ends_with("_scalp")) continue;
        name.resize(name.size()-6);
        if (!ready.contains(name) || fallen.contains(frame)) continue;
        for (void *f=frame; f; f=game::at<void *>(f,game::kFrameParent)) {
            if (!(game::at<unsigned>(f,game::kFrameFlags)&1)) return;
            if (f==body.model) break;
        }
        float world[16];
        if (!H->frame_world(M,frame,world)) return;
        // The mesh still carries the previous render pose during a physics tick.
        // Transfer its neck-local placement to the current simulated neck.
        const float *nw=game::WorldMatrix(neck);
        M3 nativeNeck=M3::FromWorld(nw);
        V3 local=nativeNeck.T().Apply(V3{world[12]-nw[12],world[13]-nw[13],world[14]-nw[14]});
        M3 bones[19]; V3 origins[19];
        body.PhysicalPose(bones,origins);
        auto f=std::find(body.Frames(),body.Frames()+19,neck);
        if (f==body.Frames()+19) return;
        auto idx=f-body.Frames();
        V3 position=origins[idx]+bones[idx].Apply(local);
        if (!fit.tracking) {
            const V3 size=ready.at(name).dimensions;
            const float scale=Len(M3::Raw(world).r[0]);
            fit.depth=.60f*size.y*scale;
            fit.leverage=size.y/std::max(.001f,std::min(size.x,size.z));
            fit.mount=body.HeadLocalPoint(position);
            fit.track(position,torso,dt);
            body.headwearChecked=false;
            return;
        }
        const Q rotation=QuatOf(M3::FromWorld(world)*nativeNeck.T()*bones[idx]);
        const float scale = Len(M3::Raw(world).r[0]);
        auto &model=ready.at(name);
        if (!model.hull) {
            JPH::Array<JPH::Vec3> points;
            points.reserve(model.mesh.vertices.size());
            for (V3 vertex:model.mesh.vertices) points.push_back(J(vertex));
            auto cooked=JPH::ConvexHullShapeSettings(points,.002f).Create();
            if (cooked.IsValid()) model.hull=cooked.Get();
        }
        if (!model.hull) return;
        JPH::RefConst<JPH::Shape> shape=model.hull;
        if (scale!=1.f) {
            auto scaled=shape->ScaleShape(JPH::Vec3::sReplicate(scale));
            if (!scaled.IsValid()) return;
            shape=scaled.Get();
        }
        JPH::Quat q(-rotation.x,-rotation.y,-rotation.z,rotation.w);
        // Resolve overlap only within the hat's own bounds. If there is no
        // room yet, keep it worn until the moving head opens a clear pose.
        if (!ClearRelease(shape,q,model.mesh.radius*scale,position)) {
            body.headwearChecked=false;
            return;
        }
        void *prop=H->prop_create(M,(name+"_lined").c_str(),nullptr,&position.x,&rotation.w,scale);
        if (!prop) return;
        if (!H->frame_watch(M,frame,gone,nullptr)) {
            H->prop_remove(M,prop);
            return;
        }
        const V3 velocity=fit.hatMotion;
        JPH::BodyCreationSettings settings(shape,J(jolt_world::ToJolt(position)),q,
                                           JPH::EMotionType::Dynamic,jolt_world::kDynamic);
        settings.mOverrideMassProperties=JPH::EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass=hatMass;
        settings.mLinearVelocity=J(velocity);
        settings.mMotionQuality=JPH::EMotionQuality::LinearCast;
        settings.mFriction=.78f;
        settings.mRestitution=.05f;
        settings.mAngularDamping=1.5f;
        settings.mLinearDamping=.55f;
        auto id=jolt_world::Bodies().CreateAndAddBody(settings,JPH::EActivation::Activate);
        if (id.IsInvalid()) {
            H->frame_watch(M,frame,nullptr,nullptr);
            H->prop_remove(M,prop);
            return;
        }
        fallen.emplace(frame,FallenHat{prop,body.pedestrian ? body.actor : nullptr,body.model,
                                       body.pedestrian && !body.puppet,id,position,position,&model.mesh,scale});
        show(frame,false);
        Log("headwear detached: %s ragdoll %u age %.3f up_y %.3f lift %.3f m at %.3f %.3f %.3f velocity %.3f %.3f %.3f torso %.3f %.3f %.3f",
            name.c_str(),body.serial,body.age,up.y,body.hatFit.lift,position.x,position.y,position.z,
            velocity.x,velocity.y,velocity.z,fit.torsoMotion.x,fit.torsoMotion.y,fit.torsoMotion.z);
        return;
    }
}

void Reset() {
    auto entries = std::move(fallen);
    fallen.clear();
    for (auto [frame,hat] : entries) {
        H->frame_watch(M,frame,nullptr,nullptr);
        show(frame,true);
        Remove(hat);
    }
    for (const auto &hat:retired) Remove(hat);
    retired.clear();
}
} // namespace headwear
