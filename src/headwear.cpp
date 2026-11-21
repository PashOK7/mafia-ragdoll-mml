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
std::unordered_map<std::string,V3> ready;
bool prepared = false;
std::unordered_map<void *, void *> fallen;
std::vector<void *> retired;

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
        V3 dimensions{};
        if (!hats::split(source,marker+"_scalp",person,prop,dimensions)) continue;
        const std::string propPath = dir+marker+"_lined.4ds";
        const std::string propName = "models\\"+marker+"_lined.4ds";
        if (otherProvider(propName,propPath) || !save(propPath,prop) || !save(personPath,person)) continue;
        if (!H->file_add(M,propName.c_str(),propPath.c_str())) continue;
        if (!H->file_add(M,gameName.c_str(),personPath.c_str())) {
            H->file_add(M,propName.c_str(),nullptr);
            continue;
        }
        ready.emplace(marker,dimensions);
    }
    Log("detachable headwear prepared for %u installed character models",unsigned(ready.size()));
}

void Tick() {
    if (!prepared) Prepare();
    for (void *prop : retired) H->prop_remove(M,prop);
    retired.clear();
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
        const V3 position=origins[idx]+bones[idx].Apply(local);
        if (!fit.tracking) {
            const V3 size=ready.at(name);
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
        void *prop = H->prop_create(M,(name+"_lined").c_str(),nullptr,&position.x,&rotation.w,scale);
        if (!prop) return;
        V3 velocity=fit.hatMotion;
        // Mafia puts a prop created at zero velocity to sleep in mid-air.
        // Start a stationary, newly unsupported hat with its first gravity step.
        const float gravityStep=9.81f*std::clamp(dt,.001f,.05f);
        if (Len(velocity)<gravityStep) velocity.y=std::min(velocity.y,-gravityStep);
        MmlPropPhysics physics{1,{velocity.x,velocity.y,velocity.z}};
        if (!H->prop_physics(M,prop,&physics) || !H->frame_watch(M,frame,gone,nullptr)) {
            H->prop_remove(M,prop);
            return;
        }
        fallen.emplace(frame,prop);
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
    for (auto [frame,prop] : entries) {
        H->frame_watch(M,frame,nullptr,nullptr);
        show(frame,true);
        H->prop_remove(M,prop);
    }
    for (void *prop : retired) H->prop_remove(M,prop);
    retired.clear();
}
} // namespace headwear
