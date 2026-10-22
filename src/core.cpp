// The ragdolls (core.h): mafia-ragdoll's core.cpp, its ragdoll part, on the loader's services.
//
// Who becomes a ragdoll: every person or traffic pedestrian whose life goes out near the player, however it went out
// (a bullet, a car, a fall, a script: their alive flag or health is watched every tick, a hit only adds its push), and
// a living one knocked over by a car or by a body (he gets up the game's way once his body lies still). The body
// starts from the pose the game drew at the end of the last tick, before it reacted to whatever is killing him, with
// the motion his animation had (the last two poses). It lives in the rigid-body world (jolt_world, jolt_ragdoll) with
// the static world as the game draws it, the cars as bodies of their own mass, the living people near it as their
// body parts; its bones are written over the game's animation every frame (RENDER_BEGIN, bone_set), and for the
// muscles the game's own death animation, as the game plays it under the body, is the pose they pull towards.
//
// What the loader gives in place of mafia-ragdoll's own hooks: humans_list and pedestrians_list (its constructor hook
// and its walk of the pedestrian managers), HUMAN_HIT / PEDESTRIAN_HIT (its Hit hooks), hit_filter (a bullet in a
// corpse is the mod's, not the game's living person's hit logic), bullet_shape (bullets meet the bodies where they
// lie), human_hit (the game's own knock-down for a body's blow), human_get_up (the game's get-up from the pose last
// drawn), frame_watch (frame_life: the frames LS3D frees), scene_pieces (the scenery), actor_meshes (the cars' bodies),
// hook (the dodge fix: the pedestrians' car sweep). The trams, the people dying seated in cars, the cars feeling the
// bodies are mafia-ragdoll's as they were.
#include "fmt.h"
#include "core.h"
#include <windows.h>
#include <intrin.h>
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "game.h"
#include "jolt_ragdoll.h"
#include "jolt_world.h"
#include "knock.h"
#include "ragcam.h"
#include "mod.h"
#include "ragdoll.h"
#include "scenery.h"
#include "vehicles.h"

using namespace mod;

namespace core {
namespace {

#define LOG(...) mod::Log(__VA_ARGS__)

// ---- settings (config.json; the game's Options > Mods)
struct Config {
    bool enabled = true;          // a ragdoll on death
    bool carKnockdown = true;     // a car knocks living people over as ragdolls (the player too)
    float carPush = 0.7f;         // the share of the car's speed given to the body where it is struck (no car body)
    bool carFeel = true;          // cars feel the bodies they push
    float trackRadius = 60.0f;    // m: people this close to the player have their pose remembered
    bool bullets = true;          // bullets strike the bodies on the ground
    float kick = 1.0f;            // N s a point of the game's damage: a bullet's momentum
    float whole = 0.6f;           // the share of a blow that moves the whole body
    float radius = 0.3f;          // m: the parts this close to a blow share the rest
    float maxPart = 8.0f;         // m/s: the most one blow speeds up a single part
    float most = 120.0f;          // N s: the most one hit gives (1.5 m/s to the whole of an 80 kg body)
    float remember = 0.3f;        // s: bullets in someone alive given to his body if he dies within this
    bool draw = false;            // the bodies drawn (debug)
    bool blastKnockdown = true;   // an explosion knocks the living near it over as ragdolls
    float blast = 1.0f;           // the blast's push times this (a grenade's 400 of force: 12 m/s at its centre)
} g_cfg;

std::vector<std::unique_ptr<Ragdoll>> g_ragdolls;
unsigned g_tick;
DWORD g_playerRisingUntil;   // the player gets up (human_get_up) until then, or 0
float g_lastDt = 1.0f / 60.0f;

// The last two poses of every person near the player: a ragdoll starts from the pose before the game reacted, with
// the motion the animation had.
struct PoseHistory {
    Ragdoll::PoseSnap snap;
    V3 pts[2][Ragdoll::COUNT];
    float dt[2] = {0, 0};
    int count = 0;
    unsigned seen = 0;
};
std::unordered_map<void *, PoseHistory> g_poses;

template <class F> bool Guarded(F &f) {
    __try { f(); return true; } __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}

// ---- people and pedestrians
constexpr int kPedPos = 0x10, kPedModel = 0x40, kPedHealth = 0x15C, kPedActive = 0x0C;
std::unordered_set<void *> g_peds;   // the traffic's elements this tick
std::vector<MmlPedestrian> g_pedList;
std::vector<void *> g_humanList;

void ListPeople() {
    int n = H->humans_list(M, nullptr, 0);
    g_humanList.resize(std::max(n, 0) + 32);
    n = H->humans_list(M, g_humanList.data(), (int)g_humanList.size());
    g_humanList.resize(std::min<size_t>(std::max(n, 0), g_humanList.size()));
    int p = H->pedestrians_list(M, nullptr, 0);
    g_pedList.resize(std::max(p, 0) + 32);
    for (MmlPedestrian &x : g_pedList) x.size = sizeof x;
    p = H->pedestrians_list(M, g_pedList.data(), (int)g_pedList.size());
    g_pedList.resize(std::min<size_t>(std::max(p, 0), g_pedList.size()));
    g_peds.clear();
    for (const MmlPedestrian &x : g_pedList) g_peds.insert(x.element);
}
bool IsPed(void *key) { return g_peds.count(key) != 0; }
void *ModelOf(void *key) { return game::at<void *>(key, IsPed(key) ? kPedModel : game::kActorFrame); }
bool IsDead(void *key) { return IsPed(key) ? game::at<float>(key, kPedHealth) <= 0.0f : !game::at<uint8_t>(key, game::kActorAlive); }

std::unordered_map<void *, bool> g_isHuman;
bool HumanCheck(void *actor) {
    void *f = game::at<void *>(actor, game::kActorFrame);
    return f && game::FindChild(f, "r_thigh") && game::FindChild(f, "l_hand");
}
bool IsHuman(void *actor) {
    if (g_isHuman.count(actor)) return true;
    bool ok = false;
    auto f = [&] { ok = HumanCheck(actor); };
    if (!Guarded(f) || !ok) return false;   // (not kept: a pooled person may get his model later)
    g_isHuman[actor] = true;
    return true;
}

// ---- the ragdolls
Ragdoll *Find(void *actor) {
    for (auto &r : g_ragdolls) if (r->actor == actor) return r.get();
    return nullptr;
}

// the frames LS3D frees (frame_watch): forgotten at the next tick or frame, before anything of ours reads them
std::unordered_set<void *> g_freed;
std::unordered_set<void *> g_watched;
void FrameGone(void *frame, void *) {
    g_freed.insert(frame);
    g_watched.erase(frame);
}
void Watch(void *frame) {
    if (frame && g_watched.insert(frame).second) H->frame_watch(M, frame, FrameGone, nullptr);
}
void WatchRagdoll(const Ragdoll *rd) {
    Watch(rd->Model());
    for (int i = 0; i < 20; i++) Watch(rd->Frames()[i]);
}

void Unshape(void *actor) { H->bullet_shape(M, actor, nullptr, 0); }

void Drop(void *actor) {
    for (size_t i = 0; i < g_ragdolls.size(); i++)
        if (g_ragdolls[i]->actor == actor) {
            g_ragdolls.erase(g_ragdolls.begin() + i);
            Unshape(actor);
            LOG("ragdoll dropped %p", actor);
            return;
        }
}
void EraseAt(size_t i) {
    Unshape(g_ragdolls[i]->actor);
    g_ragdolls.erase(g_ragdolls.begin() + i);
}

void Purge() {
    if (g_freed.empty()) return;
    for (size_t i = 0; i < g_ragdolls.size();) {
        Ragdoll *rd = g_ragdolls[i].get();
        if (g_freed.count(rd->Model()) || rd->UsesAny(g_freed)) {
            LOG("ragdoll %p: the game freed its model's frames (rebuilt: a restart, a respawn): dropped", rd->actor);
            EraseAt(i);
            continue;
        }
        i++;
    }
    for (auto it = g_poses.begin(); it != g_poses.end();) {
        bool gone = false;
        for (void *f : it->second.snap.frames) gone = gone || (f && g_freed.count(f));
        it = gone ? g_poses.erase(it) : std::next(it);
    }
    g_freed.clear();
}

// A ragdoll whose person the game brought back (a mission restarted) or whose pedestrian element the traffic gave
// to a new pedestrian: the game's again.
bool AliveAgain(const Ragdoll *rd) {
    if (rd->puppet) return false;
    if (rd->pedestrian) return !IsPed(rd->actor) || !game::at<uint8_t>(rd->actor, kPedActive) || !IsDead(rd->actor);
    return !IsDead(rd->actor);
}
bool SafeAliveAgain(const Ragdoll *rd) {
    bool r = true;
    auto f = [&] { r = AliveAgain(rd); };
    return !Guarded(f) || r;
}

bool SafeCapture(void *key, V3 *pts) {
    bool ok = false;
    auto f = [&] {
        V3 fwd;
        void *frames[20];
        std::string err;
        V3 dir = IsPed(key) ? V3{} : game::at<V3>(key, game::kActorFacing);
        ok = Ragdoll::Capture(ModelOf(key), IsPed(key) ? nullptr : &dir, pts, &fwd, frames, &err);
    };
    return Guarded(f) && ok;
}
bool SafeSnapshot(void *key, Ragdoll::PoseSnap *snap) {
    bool ok = false;
    auto f = [&] {
        std::string err;
        V3 dir = IsPed(key) ? V3{} : game::at<V3>(key, game::kActorFacing);
        ok = Ragdoll::Snapshot(ModelOf(key), IsPed(key) ? nullptr : &dir, snap, &err);
    };
    return Guarded(f) && ok;
}

// The remembered pose to measure the motion against: the newest one that differs from the pose now.
const V3 *PreviousPose(void *actor, float *dt) {
    auto it = g_poses.find(actor);
    if (it == g_poses.end() || it->second.count == 0) return nullptr;
    PoseHistory &h = it->second;
    V3 now[Ragdoll::COUNT];
    if (!SafeCapture(actor, now)) return nullptr;
    if (Len(now[Ragdoll::PELVIS] - h.pts[0][Ragdoll::PELVIS]) + Len(now[Ragdoll::HEAD] - h.pts[0][Ragdoll::HEAD]) > 1e-5f) {
        *dt = g_lastDt;
        return h.pts[0];
    }
    if (h.count < 2) return nullptr;
    *dt = h.dt[0];
    return h.pts[1];
}

// The car a person sits in: the vehicle he uses (C_human +0x98, a car) when his model hangs under its model.
void *SeatCarOf(void *actor) {
    void *car = nullptr;
    auto f = [&] {
        void *c = game::at<void *>(actor, game::kHumanVehicle);
        if (!c || game::at<uint32_t>(c, game::kActorType) != 4) return;
        void *carFrame = game::at<void *>(c, game::kActorFrame);
        void *fr = game::at<void *>(actor, game::kActorFrame);
        for (int i = 0; fr && i < 24; i++, fr = game::at<void *>(fr, game::kFrameParent))
            if (fr == carFrame) { car = c; return; }
    };
    Guarded(f);
    return car;
}

// ---- people in a tram (a vehicle they walk in, type 8): the game keeps such a person in the tram's space; a ragdoll
// lives in the world, so he is taken out the game's own way when it starts and put back when he gets up on its floor
// (game_build.h humanPlatform, tramCarOf; C_human +0x9C the vehicle to go into, +0x584 which of its cars: both builds)
constexpr int kHumanToVehicle = 0x9C, kHumanTramCar = 0x584;
std::unordered_map<void *, void *> g_gettingUpInTram;
void *TramOf(void *h) {
    void *v = game::at<void *>(h, game::kHumanVehicle);
    return v && game::at<uint32_t>(v, game::kActorType) == 8 ? v : nullptr;
}
bool SafePlatform(void *h, bool in) {
    auto f = [&] { reinterpret_cast<void(__thiscall *)(void *, char)>(game::Exe().humanPlatform)(h, in ? 1 : 0); };
    return Guarded(f);
}
int SafeTramCar(void *tram, const V3 &p) {
    float pos[3] = {p.x, p.y, p.z};
    int r = -1;
    auto f = [&] { r = reinterpret_cast<int(__thiscall *)(void *, float *)>(game::Exe().tramCarOf)(tram, pos); };
    return Guarded(f) ? r : -1;
}
void *LeaveTram(void *h) {
    if (!game::Exe().humanPlatform || IsPed(h)) return nullptr;
    void *tram = TramOf(h);
    if (!tram) return nullptr;
    bool ok = SafePlatform(h, false);
    LOG("%p out of tram %p for its ragdoll (the game's way out)%s", h, tram, ok ? "" : ": FAULTED");
    return ok ? tram : nullptr;
}
void BackIntoTram(void *h, void *tram) {
    if (!tram) return;
    game::at<void *>(h, kHumanToVehicle) = tram;
    SafePlatform(h, true);
}
bool GettingUp(void *h) {
    MmlHumanAnim s{};
    s.size = sizeof s;
    return H->human_anim_state(M, h, &s) && s.state == 6;
}
void EnterTramUnder(void *h, const V3 &pelvis) {
    if (!game::Exe().humanPlatform || !game::Exe().tramCarOf || IsPed(h) || TramOf(h)) return;
    void *v = const_cast<void *>(jolt_world::VehicleAround(pelvis + V3{0, 0.3f, 0}, 1.5f, 4.0f));
    if (!v || game::at<uint32_t>(v, game::kActorType) != 8) return;
    int car = SafeTramCar(v, pelvis);
    if (car < 0) return;
    game::at<int>(h, kHumanTramCar) = car;
    game::at<void *>(h, kHumanToVehicle) = v;
    bool ok = SafePlatform(h, true);
    LOG("%p got up on the floor of tram %p (its car %d): back in it (the game's way in)%s", h, v, car, ok ? "" : ": FAULTED");
    if (ok) g_gettingUpInTram[h] = v;
}
// someone put back in a tram walks off its floor while his get-up animation moves him: out of it, into the street
void WatchTramGetUps() {
    for (auto it = g_gettingUpInTram.begin(); it != g_gettingUpInTram.end();) {
        void *h = it->first;
        if (!GettingUp(h) || TramOf(h) != it->second) { it = g_gettingUpInTram.erase(it); continue; }
        V3 feet = game::ActorWorldPos(h);
        if (jolt_world::VehicleAround(feet + V3{0, 0.5f, 0}, 1.5f, 4.0f) == it->second) { ++it; continue; }
        bool ok = SafePlatform(h, false);
        LOG("%p walked off tram %p's floor while getting up: out of it, into the street%s", h, it->second, ok ? "" : ": FAULTED");
        it = g_gettingUpInTram.erase(it);
    }
}
void SafeWatchTramGetUps() {
    auto f = [&] { WatchTramGetUps(); };
    if (!Guarded(f)) g_gettingUpInTram.clear();
}

// ---- bullets in bodies (mafia-ragdoll's hits.cpp, its blows; the loader gives the bodies' bullets)
struct Kept { void *actor; V3 J, at; bool hasAt; DWORD tick; };
std::vector<Kept> g_kept;
unsigned g_corpseHits, g_shapesGiven;

// A hit's momentum (N s): the game's damage times the kick, held to the strongest one shot gives (a script's or a
// test's damage of 1000 threw a body 12 m; a shotgun's pellets are each a hit of their own)
float Momentum(float damage) { return std::min(damage * g_cfg.kick, g_cfg.most); }
void Kick(Ragdoll *rd, const V3 &dir, const V3 &at, float damage) {
    if (!rd || damage <= 0 || Len(dir) < 1e-4f) return;
    rd->Kick(Norm(dir) * Momentum(damage), at, g_cfg.whole, g_cfg.radius, g_cfg.maxPart);
}
void Remember(void *actor, const V3 &dir, const V3 *at, float damage) {
    if (!actor || damage <= 0 || Len(dir) < 1e-4f || g_cfg.remember <= 0) return;
    if (g_kept.size() >= 256) g_kept.erase(g_kept.begin());
    g_kept.push_back({actor, Norm(dir) * Momentum(damage), at ? *at : V3{0, 0, 0}, at != nullptr, GetTickCount()});
}
void TakeRemembered(void *actor, Ragdoll *rd) {
    DWORD now = GetTickCount(), keep = (DWORD)(g_cfg.remember * 1000.0f);
    for (size_t i = 0; i < g_kept.size();) {
        const Kept &k = g_kept[i];
        if (k.actor != actor) { i++; continue; }
        if (rd && now - k.tick <= keep) rd->Kick(k.J, k.hasAt ? k.at : rd->Center(), g_cfg.whole, g_cfg.radius, g_cfg.maxPart);
        g_kept.erase(g_kept.begin() + i);
    }
}
void KeptTick() {
    DWORD now = GetTickCount(), keep = (DWORD)(g_cfg.remember * 1000.0f) + 100;
    g_kept.erase(std::remove_if(g_kept.begin(), g_kept.end(), [&](const Kept &k) { return now - k.tick > keep; }), g_kept.end());
}

// every body's parts for the bullets, where they are this tick
void GiveShapes() {
    MmlCapsule caps[Ragdoll::BCOUNT];
    for (auto &r : g_ragdolls) {
        if (!g_cfg.bullets) { Unshape(r->actor); continue; }
        for (int b = 0; b < Ragdoll::BodyCount(); b++) {
            V3 a, e;
            float rad;
            r->BodyCapsule(b, &a, &e, &rad);
            caps[b] = {{a.x, a.y, a.z}, {e.x, e.y, e.z}, rad};
        }
        if (H->bullet_shape(M, r->actor, caps, Ragdoll::BodyCount())) g_shapesGiven++;
    }
}

// ---- cars, tracked every tick from their model frame (the speed of a car that hits someone)
struct CarTrack { V3 pos, vel; unsigned tick; };
std::unordered_map<void *, CarTrack> g_carTracks;
void TrackCars(float dt) {
    void *pl = game::Player();
    if (!pl || dt <= 0) return;
    V3 pp = game::ActorWorldPos(pl);
    static std::vector<void *> all;
    vehicles::ListVehicles(pp, 80.0f, all);
    for (void *a : all) {
        void *m = game::at<void *>(a, game::kActorFrame);
        if (!m) continue;
        const float *w = game::WorldMatrix(m);
        V3 p{w[12], w[13], w[14]};
        auto it = g_carTracks.find(a);
        if (it != g_carTracks.end() && g_tick - it->second.tick == 1) {
            V3 v = (p - it->second.pos) * (1.0f / dt);
            it->second.vel = it->second.vel * 0.5f + v * 0.5f;   // (the frames' jitter smoothed)
        } else if (it == g_carTracks.end()) {
            g_carTracks[a] = {p, {0, 0, 0}, g_tick};
            continue;
        } else {
            it->second.vel = {0, 0, 0};
        }
        it->second.pos = p;
        it->second.tick = g_tick;
    }
    for (auto it = g_carTracks.begin(); it != g_carTracks.end();) it = g_tick - it->second.tick > 30 ? g_carTracks.erase(it) : std::next(it);
}
bool SafeTrackCars(float dt) {
    auto f = [&] { TrackCars(dt); };
    return Guarded(f);
}
V3 CarVelocity(void *car) {
    auto it = g_carTracks.find(car);
    return it != g_carTracks.end() && g_tick - it->second.tick <= 2 ? it->second.vel : V3{0, 0, 0};
}

// The bodies' answer to the cars (jolt_world::CarKicks): what the solver's contacts did to a driven car in the step
// goes into the game's car: its velocity (moved to the game's centre of mass) and its turning (the game's sign: a
// point moves at v - w x r), held to 3 m/s and 2 rad/s a step.
void JoltCarKicks() {
    const game::ExeBuild &x = game::Exe();
    for (const jolt_world::CarKick &k : jolt_world::CarKicks()) {
        void *car = const_cast<void *>(k.car);
        if (game::at<uint32_t>(car, game::kActorType) != 4) continue;
        uint8_t *veh = static_cast<uint8_t *>(car) + x.vehicle;
        float m = game::at<float>(veh, x.vehMass);
        if (!(m > 300.0f && m < 20000.0f)) continue;
        V3 dv = k.dv + Cross(k.dw, game::at<V3>(veh, x.vehCentre) - k.com), dw = k.dw;
        if (Len(dv) > 3.0f) dv = dv * (3.0f / Len(dv));
        if (Len(dw) > 2.0f) dw = dw * (2.0f / Len(dw));
        game::at<V3>(veh, x.vehVelocity) += dv;
        game::at<V3>(veh, x.vehTurn) -= dw;
    }
}
bool SafeCarReactions() {
    auto f = [&] { JoltCarKicks(); };
    return Guarded(f);
}

// ---- making a ragdoll
Ragdoll *Start(void *actor, bool puppet, std::string *err) {
    if (Ragdoll *r = Find(actor)) return r;
    void *leftTram = LeaveTram(actor);
    auto r = std::make_unique<Ragdoll>();
    r->puppet = puppet;
    float dt = 0;
    const V3 *prev = nullptr;
    const Ragdoll::PoseSnap *startPose = nullptr;
    // the pose at the end of the last tick, before the game reacted to what is killing him (a hit's reaction moves
    // the model and starts falls the body should not inherit)
    auto hist = g_poses.find(actor);
    if (hist != g_poses.end() && hist->second.count > 0 && g_tick - hist->second.seen <= 1) {
        startPose = &hist->second.snap;
        if (hist->second.count > 1) { prev = hist->second.pts[1]; dt = hist->second.dt[0]; }
    } else {
        prev = PreviousPose(actor, &dt);
    }
    bool ped = IsPed(actor);
    V3 dir = ped ? V3{} : game::at<V3>(actor, game::kActorFacing);
    bool inited = false;
    auto init = [&] { inited = r->Init(actor, ModelOf(actor), ped ? nullptr : &dir, err, prev, dt, startPose); };
    if (!Guarded(init) || !inited) {
        LOG("ragdoll init failed on %p: %s", actor, err->c_str());
        BackIntoTram(actor, leftTram);
        return nullptr;
    }
    r->pedestrian = ped;
    r->keyVtable = game::at<uint32_t>(actor, 0);
    V3 pv = r->BodyVel(Ragdoll::B_PELVIS);
    LOG("ragdoll started on %s %p (puppet=%d, pose from %s, motion from %s, dt %.3f; pelvis %.2f %.2f %.2f m/s)", ped ? "pedestrian" : "person",
        actor, puppet, startPose ? "last tick" : "now", prev ? "history" : "rest", dt, pv.x, pv.y, pv.z);
    g_ragdolls.push_back(std::move(r));
    Ragdoll *made = g_ragdolls.back().get();
    void *car = ped ? nullptr : SeatCarOf(actor);
    if (car) LOG("ragdoll %p dies sitting in car %p: its body stays in the car's cabin, held to the seat", actor, car);
    std::string jerr;
    if (!jolt_ragdoll::Add(made, &jerr, car)) {
        LOG("ragdoll %p: Jolt could not make its body (%s): left to the game's animation; %s", actor, jerr.c_str(), jolt_world::Describe().c_str());
        g_ragdolls.pop_back();
        *err = "Jolt could not make the body: " + jerr;
        BackIntoTram(actor, leftTram);
        return nullptr;
    }
    WatchRagdoll(made);
    return made;
}

// ---- what the game moves, for the rigid-body world: the cars around the player, the people near a ragdoll as their
// 15 body parts from this tick's pose
void JoltMovers(std::vector<smoke::Mover> &out) {
    out.clear();
    void *pl = game::Player();
    if (!pl) return;
    V3 pp = game::ActorWorldPos(pl);
    vehicles::GatherVehicles(pp - V3{40, 15, 40}, pp + V3{40, 15, 40}, out);
    std::vector<V3> bodies;
    for (auto &r : g_ragdolls) bodies.push_back(r->BodyPos(Ragdoll::B_PELVIS));
    if (bodies.empty()) return;
    for (auto &kv : g_poses) {
        if (kv.second.count == 0 || g_tick - kv.second.seen > 1) continue;
        if (Find(kv.first)) continue;   // (a fresh ragdoll's pose as capsules would stop it dead)
        V3 p = kv.second.snap.pts[Ragdoll::PELVIS];
        bool close = false;
        for (const V3 &c : bodies) close = close || Len(c - p) < 6.0f;
        if (!close) continue;
        Ragdoll pose;
        pose.Setup(kv.second.snap.pts, kv.second.snap.fwd);
        for (int b = 0; b < Ragdoll::BodyCount(); b++) {
            smoke::Mover m;
            m.key = static_cast<const uint8_t *>(kv.first) + b;
            m.kind = smoke::Mover::CAPSULE;
            pose.BodyCapsule(b, &m.p0, &m.p1, &m.r);
            m.owner = kv.first;
            out.push_back(m);
        }
    }
}
void JoltStep(const std::vector<Ragdoll *> &jolt, float dt) {
    void *pl = game::Player();
    if (!jolt_world::Running()) {
        if (!g_cfg.enabled || !jolt_ragdoll::Enabled() || !pl) return;
        std::string err;
        if (!jolt_world::Start(mod::Log, game::ActorWorldPos(pl), &err)) {
            static bool told;
            if (!told) { told = true; LOG("jolt: the world did not start (%s); no ragdolls until it does", err.c_str()); }
            return;
        }
    }
    static std::vector<smoke::Mover> movers;
    JoltMovers(movers);
    V3 pp = pl ? game::ActorWorldPos(pl) : V3{};
    jolt_ragdoll::Step(jolt, dt, movers, pl ? &pp : nullptr);
}
bool SafeJoltStep(const std::vector<Ragdoll *> &jolt, float dt) {
    auto f = [&] { JoltStep(jolt, dt); };
    return Guarded(f);
}
bool SafeSnapshotNow(Ragdoll *r) {
    bool ok = false;
    auto f = [&] { ok = r->SnapshotNow(&r->anim); };
    return Guarded(f) && ok;
}
bool SafeApply(Ragdoll *r) {
    auto f = [&] { r->Apply(); };
    return Guarded(f);
}
// The bones' worst turn since our last write (degrees): the engine's animation played on the model meanwhile.
float SafeOverwritten(Ragdoll *r) {
    const char *bone = "";
    float ow = 0;
    auto f = [&] { ow = r->Overwritten(&bone); };
    return Guarded(f) ? ow : 180.0f;
}
// A body at rest whose bones nobody turned since our write keeps the pose as it stands: written twice at rest (the
// tick's and the frame's, so the model's update has the last of its motion), then left be until it wakes or the
// engine writes them (0.1 degrees: a write read back differs by float rounding, an animation's frame by far more).
bool AtRest(Ragdoll *r, float overwritten) {
    if (!r->sleeping || overwritten > 0.1f) { r->restWrites = 0; return false; }
    if (r->restWrites >= 2) return true;
    r->restWrites++;
    return false;
}

// ---- every tick: the poses of the people near the player remembered, and the deaths caught (people die through
// many paths: a hit, a fall, a script, the water; a ragdoll starts whenever the alive flag drops, a hit only adds its
// push)
std::unordered_map<void *, uint8_t> g_alive;
std::unordered_set<void *> g_failed;   // bodies a ragdoll could not start on (until they live again)
void TrackPoses(float dt) {
    void *pl = game::Player();
    if (!pl) return;
    V3 pp = game::ActorWorldPos(pl);
    std::vector<void *> actors;
    for (void *h : g_humanList) actors.push_back(h);
    for (const MmlPedestrian &p : g_pedList) if (p.model && p.spawned) actors.push_back(p.element);
    float r2 = g_cfg.trackRadius * g_cfg.trackRadius;
    for (void *h : actors) {
        bool ped = IsPed(h);
        if (!ped && !IsHuman(h)) continue;
        uint8_t alive = !IsDead(h);
        V3 pos;
        if (ped) { const float *m = game::WorldMatrix(ModelOf(h)); pos = {m[12], m[13], m[14]}; }
        else pos = game::ActorWorldPos(h);   // (a driver's actor position is in his car's frame)
        V3 d = pos - pp;
        if (h != pl && Dot(d, d) > r2) { g_alive[h] = alive; continue; }   // (remembered at any distance)
        g_alive[h] = alive;
        if (alive) g_failed.erase(h);
        // every dead body near the player is a ragdoll, however it died: a corpse is never left frozen in its last pose
        if (!alive && g_cfg.enabled && !Find(h) && !g_failed.count(h)) {
            std::string err;
            if (Start(h, false, &err)) LOG("dead body %p adopted", h);
            else g_failed.insert(h);
        }
        if (Find(h)) continue;
        Ragdoll::PoseSnap snap;
        if (!SafeSnapshot(h, &snap)) continue;
        PoseHistory &ph = g_poses[h];
        memcpy(ph.pts[1], ph.pts[0], sizeof ph.pts[0]);
        memcpy(ph.pts[0], snap.pts, sizeof snap.pts);
        ph.snap = snap;
        ph.dt[1] = ph.dt[0];
        ph.dt[0] = dt;
        ph.count = std::min(ph.count + 1, 2);
        ph.seen = g_tick;
        Watch(ModelOf(h));
    }
    for (auto it = g_poses.begin(); it != g_poses.end();) it = g_tick - it->second.seen > 2 ? g_poses.erase(it) : std::next(it);
}

// ---- bodies knock living people over (knock.h): the game's own car hit, with no car and no damage, knocks the
// person over its way (his knockdown and get-up); OnHit, told by g_bodyKnock that this hit is a body's, starts the
// ragdoll, and the body's momentum goes into it where it struck. A pedestrian: pedHit type 5 with a strength over 16
// (no car: no health taken); a person: his Hit type 8 with the blow's speed over 3 m/s.
void *g_bodyKnock;
bool CallKnock(void *h, bool ped, const V3 &dir, const V3 &at) {
    MmlHumanHit hit{};
    if (ped) {
        hit.type = 5;
        hit.dir = {dir.x, dir.y, dir.z};
        hit.part = 5;
        hit.damage = 20.0f;
    } else {
        V3 v = dir * 4.0f;
        hit.type = 8;
        hit.dir = {v.x, v.y, v.z};
        hit.pos = {at.x, at.y, at.z};
    }
    return H->human_hit(M, h, &hit) != 0;
}
void KnockByBodies(float dt) {
    static std::vector<knock::Knock> knocks;
    knock::Tick(dt, jolt_world::PersonBlows(), knocks);
    for (const knock::Knock &k : knocks) {
        void *h = k.who;
        const bool ped = IsPed(h);
        const char *result;
        if (!g_cfg.enabled) result = "ragdolls off";
        else if (Find(h)) result = "a body already";
        else if (!ped && !IsHuman(h)) result = "not a person";
        else if (IsDead(h)) result = "dead";
        else if (!ped && SeatCarOf(h)) result = "in a car";
        else {
            V3 d = Norm(k.J, {0, 0, 1});
            g_bodyKnock = h;
            bool ok = CallKnock(h, ped, d, k.at);
            g_bodyKnock = nullptr;
            if (Ragdoll *rd = Find(h)) {
                rd->Kick(k.J, k.at, 0.3f, 0.25f, 3.0f);   // (the body's momentum where it struck)
                result = "knocked over";
            } else {
                result = ok ? "the game did not knock him over" : "the game's hit was refused";
            }
        }
        knock::Note(k, result);
    }
}

// A living man whose ragdoll was taken off gets up the game's way (the loader blends from the pose last drawn); the
// player's camera stays on his body meanwhile.
void GetUp(void *actor) {
    if (H->human_get_up(M, actor, 600)) {
        LOG("%p gets up", actor);
        if (actor == H->human_player(M)) g_playerRisingUntil = GetTickCount() + 5000;
    } else {
        LOG("%p could not get up the game's way: handed back as he lies", actor);
    }
}

// What the ragdoll camera circles (ragcam.h): the player's body while it is a ragdoll (its pelvis, a little up), and
// while he gets up (his action queue, state 6, then the get-up's own track, state 4 with the game's get-up from the
// back, 111, or the front, 338: both builds; 5 s the most) his lower back; else nothing.
constexpr int kGetUpBack = 111, kGetUpFront = 338;
bool PlayerBodyPoint(V3 *p) {
    void *pl = H->human_player(M);
    if (!pl) return false;
    if (Ragdoll *rd = Find(pl)) { *p = rd->BodyPos(Ragdoll::B_PELVIS) + V3{0, 0.25f, 0}; return true; }
    if (!g_playerRisingUntil) return false;
    MmlHumanAnim a{};
    a.size = sizeof a;
    MmlBone b[MML_BONE_COUNT];
    bool rising = H->human_anim_state(M, pl, &a) &&
                  (a.state == 6 || (a.state == 4 && (a.playing == kGetUpBack || a.playing == kGetUpFront)));
    if (static_cast<int>(GetTickCount() - g_playerRisingUntil) >= 0 || !rising ||
        H->human_skeleton(M, pl, b, MML_BONE_COUNT) < MML_BONE_COUNT || !b[MML_BONE_BACK1].frame) {
        g_playerRisingUntil = 0;   // up: the game's camera
        return false;
    }
    *p = V3{b[MML_BONE_BACK1].world[12], b[MML_BONE_BACK1].world[13], b[MML_BONE_BACK1].world[14]};
    return true;
}

void TickBody(float dt) {
    g_tick++;
    Purge();
    ListPeople();
    scenery::Tick();
    KeptTick();
    SafeWatchTramGetUps();
    for (size_t i = 0; i < g_ragdolls.size();) {
        Ragdoll *rd = g_ragdolls[i].get();
        // an actor freed and its memory reused by another object
        if (!rd->pedestrian && game::at<uint32_t>(rd->actor, 0) != rd->keyVtable) {
            LOG("actor %p was replaced by another object, ragdoll dropped", rd->actor);
            EraseAt(i);
            continue;
        }
        // knocked over: died meanwhile (a corpse now), or lies still (the game gets him back)
        if (rd->knockdown && IsDead(rd->actor)) { rd->knockdown = false; rd->puppet = false; }
        if (rd->knockdown && rd->age > 2.5f && (rd->sleeping || rd->age > 10.0f)) {
            void *actor = rd->actor;
            bool ped = rd->pedestrian;
            V3 pelvis = rd->BodyPos(Ragdoll::B_PELVIS);
            EraseAt(i);   // (no more of our writes: the get-up blends from the pose last drawn)
            GetUp(actor);
            if (!ped) EnterTramUnder(actor, pelvis);
            continue;
        }
        if (AliveAgain(rd)) {
            LOG("%s %p alive again", rd->pedestrian ? "pedestrian" : "person", rd->actor);
            EraseAt(i);
            continue;
        }
        i++;
    }
    // every ragdoll steps in the one world: their bodies meet each other
    std::vector<Ragdoll *> jolt;
    for (auto &r : g_ragdolls) if (jolt_ragdoll::Has(r.get())) jolt.push_back(r.get());
    bool joltOk = SafeJoltStep(jolt, dt);
    if (!joltOk) {
        LOG("jolt world faulted: its ragdolls dropped, the world made anew on the next tick");
        jolt_ragdoll::Clear();
        jolt_world::Stop();
        for (size_t i = 0; i < g_ragdolls.size();) {
            if (std::find(jolt.begin(), jolt.end(), g_ragdolls[i].get()) == jolt.end()) { i++; continue; }
            EraseAt(i);
        }
    }
    if (g_cfg.carFeel) SafeCarReactions();
    if (joltOk) KnockByBodies(dt);
    for (size_t i = 0; i < g_ragdolls.size();) {
        Ragdoll *rd = g_ragdolls[i].get();
        if (AtRest(rd, SafeOverwritten(rd))) { i++; continue; }
        if (SafeApply(rd)) {
            // the game's idea of where the body is: the actor's position or the element's, in the model's parent's space
            game::at<V3>(rd->actor, rd->pedestrian ? kPedPos : game::kActorPos) = game::ToParentPoint(rd->Model(), rd->rootPos);
            i++;
        } else {
            LOG("ragdoll %p faulted while writing, dropped", rd->actor);
            EraseAt(i);
        }
    }
    GiveShapes();
    TrackPoses(dt);
    SafeTrackCars(dt);
    V3 body;
    ragcam::Follow(PlayerBodyPoint(&body) ? &body : nullptr);
    g_lastDt = dt;
}

// ---- the dodge fix (the original game's bug): a pedestrian who jumps out of a car's way moves with the dodge
// animation, its model, not its element's position (+0x10), which stays where it stood; the game tests cars against
// that position, so a car passing the spot hits a pedestrian who is not there and puts him back under it. While the
// game sweeps a car against the pedestrians, each stands where its model is (put back after); one the car does hit is
// moved there for good first; and the pedestrian step's sphere sweep stopping against a moving car is no stop.
using PedCarSweepFn = char(__fastcall *)(void *, void *, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t,
                                         uint32_t, uint32_t, uint32_t);
using PedCarHitFn = char(__fastcall *)(void *, void *, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, void *);
using SphereSweepFn = int(__fastcall *)(void *, void *, V3 *, V3 *, float, V3 *, V3 *, int, int);
PedCarSweepFn oPedCarSweep;
PedCarHitFn oPedCarHit;
SphereSweepFn oSphereSweep;
unsigned g_dodgeFixes, g_carStops;
struct PedSwap { uint8_t *e; V3 was; };

bool ModelPosition(uint8_t *e, V3 *out) {
    void *model = game::at<void *>(e, kPedModel);
    if (!model) return false;
    const float *m = game::WorldMatrix(model);
    if (!m) return false;
    *out = {m[12], m[13], m[14]};
    return true;
}
void SwapToModels(std::vector<PedSwap> &swaps) {
    for (const MmlPedestrian &p : g_pedList) {
        uint8_t *e = static_cast<uint8_t *>(p.element);
        V3 at;
        if (!e[kPedActive] || !ModelPosition(e, &at)) continue;
        V3 &pos = game::at<V3>(e, kPedPos);
        if (Len(at - pos) < 0.05f) continue;
        swaps.push_back({e, pos});
        pos = at;
    }
}
char __fastcall hkPedCarSweep(void *self, void *edx, uint32_t a1, uint32_t a2, uint32_t a3, uint32_t a4, uint32_t a5, uint32_t a6,
                              uint32_t a7, uint32_t a8, uint32_t a9, uint32_t a10) {
    static std::vector<PedSwap> swaps;
    swaps.clear();
    auto f = [&] { SwapToModels(swaps); };
    if (!Guarded(f)) {
        for (const PedSwap &s : swaps) game::at<V3>(s.e, kPedPos) = s.was;
        swaps.clear();
    }
    char r = oPedCarSweep(self, edx, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10);
    for (const PedSwap &s : swaps) game::at<V3>(s.e, kPedPos) = s.was;
    return r;
}
char __fastcall hkPedCarHit(void *self, void *edx, uint32_t a1, uint32_t a2, uint32_t a3, uint32_t a4, uint32_t a5, void *model) {
    if (model) {
        auto f = [&] {
            for (const MmlPedestrian &p : g_pedList) {
                if (p.model != model) continue;
                uint8_t *e = static_cast<uint8_t *>(p.element);
                V3 at;
                if (!ModelPosition(e, &at)) return;
                V3 &pos = game::at<V3>(e, kPedPos);
                if (Len(at - pos) > 0.3f && g_dodgeFixes++ < 50)
                    LOG("pedestrian %p hit by a car %.1f m from where the game had it (a dodge): moved to its model first", e, Len(at - pos));
                pos = at;
                return;
            }
        };
        Guarded(f);
    }
    return oPedCarHit(self, edx, a1, a2, a3, a4, a5, model);
}
bool AgainstMovingCar(const V3 &contact) {
    for (auto &kv : g_carTracks) {
        if (g_tick - kv.second.tick > 2 || Len(kv.second.vel) < 1.5f) continue;
        V3 d = contact - kv.second.pos;
        d.y = 0;
        if (Len(d) < 3.5f) return true;   // within a car's length of its middle
    }
    return false;
}
int __fastcall hkSphereSweep(void *self, void *edx, V3 *start, V3 *delta, float radius, V3 *outPos, V3 *outNormal, int flags, int x) {
    uintptr_t ra = reinterpret_cast<uintptr_t>(_ReturnAddress());
    int hit = oSphereSweep(self, edx, start, delta, radius, outPos, outNormal, flags, x);
    if (hit && (ra == game::Exe().pedStepReturn1 || ra == game::Exe().pedStepReturn2) && start && delta && outPos && outNormal) {
        V3 contact = *outPos - *outNormal * radius;
        if (AgainstMovingCar(contact)) {
            if (g_carStops++ < 40) LOG("pedestrian step: its sweep stopped against a moving car: no stop, it stays where it is");
            return 0;
        }
    }
    return hit;
}

// ---- a bullet in a corpse: the mod's (the game's Hit would run a living person's hit logic on him; a dead
// pedestrian's element would be woken and cry out)
int HitFilter(void *person, int, MmlHumanHit *hit, void *) {
    Ragdoll *rd = Find(person);
    if (!rd || !g_cfg.enabled || hit->was_alive || hit->type == 2 || hit->type == 3) return MML_HIT_GAME;   // (a blast: OnExplosion)
    if (hit->shape || hit->type == 0) {
        V3 d{hit->dir.x, hit->dir.y, hit->dir.z}, at{hit->pos.x, hit->pos.y, hit->pos.z};
        if (Len(at) < 1e-3f) at = rd->Center();
        Kick(rd, d, at, hit->damage);
        g_corpseHits++;
        return MML_HIT_TAKEN;
    }
    return MML_HIT_GAME;
}

int ActorType(void *a) {
    if (!a) return -1;
    int t = -1;
    auto f = [&] { t = game::at<int>(a, game::kActorType); };
    Guarded(f);
    return t;
}
bool IsVehicle(void *a) { int t = ActorType(a); return t == 4 || t == 8 || t == 0x16 || t == 0x1E; }

void Hooks() {
    const game::ExeBuild &x = game::Exe();
    bool a = x.pedCarSweep && H->hook(M, x.pedCarSweep, reinterpret_cast<void *>(&hkPedCarSweep), reinterpret_cast<void **>(&oPedCarSweep));
    bool b = x.pedCarHit && H->hook(M, x.pedCarHit, reinterpret_cast<void *>(&hkPedCarHit), reinterpret_cast<void **>(&oPedCarHit));
    bool c = x.sphereSweep && H->hook(M, x.sphereSweep, reinterpret_cast<void *>(&hkSphereSweep), reinterpret_cast<void **>(&oSphereSweep));
    LOG("ragdoll: the dodge fix (the pedestrians' car sweep, their car hit, their step's sweep): %s", a && b && c ? "on" : "not known for this build");
}

} // namespace

// ---- the events
void Tick(float dt) {
    if (!game::g_exe) return;
    try {
        TickBody(dt);
    } catch (const std::exception &e) {
        static unsigned n;
        if (n++ < 20) LOG("tick: exception '%s' (the rest of this tick skipped)", e.what());
    }
}

void Render() {
    Purge();
    for (auto &r : g_ragdolls) {
        // the game brought him back since our tick: the game's (the tick drops the ragdoll)
        if (SafeAliveAgain(r.get())) continue;
        // the engine played the model's animation (a death animation) since our write: the muscles' next target
        float ow = SafeOverwritten(r.get());
        if (ow > 0.5f && jolt_ragdoll::Has(r.get()) && SafeSnapshotNow(r.get())) r->animSerial++;
        if (!AtRest(r.get(), ow)) SafeApply(r.get());
    }
    if (g_cfg.draw)
        for (auto &r : g_ragdolls)
            for (int b = 0; b < Ragdoll::BodyCount(); b++) {
                V3 a, e;
                float rad;
                r->BodyCapsule(b, &a, &e, &rad);
                MmlDrawShape s{};
                s.size = sizeof s;
                s.kind = MML_DRAW_CAPSULE;
                s.a[0] = a.x; s.a[1] = a.y; s.a[2] = a.z;
                s.b[0] = e.x; s.b[1] = e.y; s.b[2] = e.z;
                s.radius = rad;
                s.color = r->sleeping ? 0xC08080FFu : r->knockdown ? 0xC0FFC000u : 0xC000FF00u;
                s.flags = MML_DRAW_ON_TOP;
                H->draw_world(M, &s);
            }
}

void OnHit(const MmlEvent &ev) {
    if (!ev.hit || ev.hit->taken || !ev.actor) return;
    void *human = ev.actor;
    const MmlHumanHit &h = *ev.hit;
    bool wasAlive = h.was_alive != 0, alive = h.alive != 0;
    int type = h.type;
    V3 dir{h.dir.x, h.dir.y, h.dir.z}, pos{h.pos.x, h.pos.y, h.pos.z};
    void *attacker = h.attacker;
    float dmg = h.damage;
    // a vehicle struck him: a pedestrian's type 5 (pedCarHit), a person's 8 (a car) or 2 (a tram, a plane) with the
    // relative velocity as dir; a body's blow (KnockByBodies) comes through the game's car hit with no car: not a car's
    const bool body = human == g_bodyKnock;
    bool car = !body && (type == 5 || ((type == 8 || type == 2) && IsVehicle(attacker)));
    // an explosion: C_game::NewExplosion (0x5AAE10) hits every actor near it with type 2, its centre as the place
    // (seen: a grenade's 400 gave 67 of damage at 5 m); 3 is kept for what else may call it so
    const bool blast = !body && !car && (type == 2 || type == 3);
    V3 carVel = car && attacker ? CarVelocity(attacker) : V3{0, 0, 0};
    if (!g_cfg.enabled) return;
    ListPeople();   // (a hit between ticks: the pedestrians as they are)
    bool hasPos = Len(pos) > 1e-3f;
    bool bullet = type == 0 && !car;
    Ragdoll *rd = Find(human);
    if (!rd) {
        bool died = wasAlive && !alive;
        // (someone alive in a car's seat is the game's: a crash's hit on its driver, type 5, made him a ragdoll that
        // got up on the street under his own car, still seated, and died there)
        bool knocked = alive && ((car && g_cfg.carKnockdown) || body || (blast && g_cfg.blastKnockdown)) &&
                       (IsPed(human) || !SeatCarOf(human));
        if (!died && !knocked) {
            if (!blast && !car) Remember(human, dir, hasPos ? &pos : nullptr, dmg);   // (a shotgun's pellets)
            return;
        }
        std::string err;
        rd = Start(human, knocked, &err);
        if (!rd) return;
        rd->knockdown = knocked;
        LOG("hit %p type %d damage %.1f alive %d->%d%s: a ragdoll", human, type, dmg, wasAlive, alive,
            car ? Fmt(", a car at %.1f m/s", Len(carVel)).c_str() : body ? ", a body's blow" : "");
        if (died) TakeRemembered(human, rd);
    } else if (car) {
        return;   // a ragdoll already: the car's own body pushes it
    }
    if (body) return;   // the blow is the caller's (KnockByBodies): the body's momentum where it struck
    V3 at = hasPos ? pos : rd->Center();
    if (!hasPos && !car && attacker && Len(dir) > 0.5f) {
        // no place known: the part nearest the line from the attacker's chest
        V3 from = game::at<V3>(attacker, game::kActorPos) + V3{0, 1.4f, 0};
        at = rd->ClosestToLine(from, from + Norm(dir) * 200.0f);
    }
    if (car) {
        // the car is a body in the world moving with the game's car: it strikes the ragdoll itself this tick
        if (jolt_ragdoll::Has(rd) && attacker && jolt_world::HasKinematic(attacker)) return;
        float sp = Len(carVel);
        V3 push = sp > 1.0f ? carVel * (1.0f / sp) : Norm(dir, {0, 0, 1});
        if (sp <= 1.0f && type != 5 && Len(dir) > 1.0f) sp = Len(dir);
        if (sp <= 1.0f) sp = std::max(4.0f, dmg * 0.2f);
        rd->Impulse(push + V3{0, 0.3f, 0}, at, sp * g_cfg.carPush, 0.7f, 0.6f);
        return;
    }
    if (blast) return;   // an explosion: its blast throws every body near it (OnExplosion, the event after the hits)
    if (bullet || dmg > 0) Kick(rd, dir, at, dmg);
}

// An explosion (the game's NewExplosion, after its damage: those it killed or knocked over are ragdolls already):
// every body within its radius and a half thrown away from it. The speed at its centre grows with the root of its
// force (a grenade's 400: 12 m/s, a car's larger one more), at most 25 m/s; a third of it upwards.
void OnExplosion(const MmlExplosion &x) {
    if (!g_cfg.enabled || g_cfg.blast <= 0) return;
    V3 c{x.pos.x, x.pos.y, x.pos.z};
    float reach = std::max(x.radius, 1.0f) * 1.5f;
    float speed = std::min(12.0f * std::sqrt(std::max(x.damage, 0.0f) / 400.0f), 25.0f) * g_cfg.blast;
    unsigned thrown = 0;
    for (auto &r : g_ragdolls) thrown += r->Blast(c, reach, speed, 0.35f) ? 1 : 0;
    LOG("explosion at %.1f %.1f %.1f (radius %.1f, force %.0f): %u bodies thrown, %.1f m/s at its centre", c.x, c.y, c.z,
        x.radius, x.damage, thrown, speed);
}

void OnActorDestroyed(void *actor) {
    if (Find(actor)) Drop(actor);
    knock::Forget(actor);
    g_kept.erase(std::remove_if(g_kept.begin(), g_kept.end(), [&](const Kept &k) { return k.actor == actor; }), g_kept.end());
    g_gettingUpInTram.erase(actor);
    g_poses.erase(actor);
    g_isHuman.erase(actor);
    g_alive.erase(actor);
    g_failed.erase(actor);
    vehicles::Forget(actor);
    g_carTracks.erase(actor);
}

void OnMissionClose() {
    ragcam::Stop();
    g_playerRisingUntil = 0;
    if (!g_ragdolls.empty()) LOG("mission close: dropping %u ragdolls", (unsigned)g_ragdolls.size());
    for (auto &r : g_ragdolls) Unshape(r->actor);
    g_ragdolls.clear();
    jolt_ragdoll::Clear();
    jolt_world::Stop();   // (the next mission's world is made around the player)
    scenery::Clear();
    vehicles::Clear();
    knock::Clear();
    g_gettingUpInTram.clear();
    g_poses.clear();
    g_isHuman.clear();
    g_alive.clear();
    g_failed.clear();
    g_carTracks.clear();
    g_kept.clear();
    g_freed.clear();
    g_watched.clear();   // (the loader drops a mission's watches with its frames: they are told as freed first)
}

void Settings() {
    auto num = [](const char *k, float def) { return static_cast<float>(H->config_number(M, k, def)); };
    auto flag = [](const char *k, bool def) { return H->config_number(M, k, def ? 1 : 0) != 0; };
    Config d;
    g_cfg.enabled = flag("enabled", d.enabled);
    g_cfg.carKnockdown = flag("car_knockdown", d.carKnockdown);
    g_cfg.carPush = num("car_push", d.carPush);
    g_cfg.carFeel = flag("car_feel", d.carFeel);
    g_cfg.trackRadius = num("track_radius", d.trackRadius);
    g_cfg.bullets = flag("bullets", d.bullets);
    g_cfg.kick = num("bullet_kick", d.kick);
    g_cfg.whole = num("bullet_whole", d.whole);
    g_cfg.radius = num("bullet_radius", d.radius);
    g_cfg.maxPart = num("bullet_max_part", d.maxPart);
    g_cfg.most = num("bullet_most", d.most);
    g_cfg.remember = num("bullet_remember", d.remember);
    g_cfg.draw = flag("debug_draw", d.draw);
    g_cfg.blastKnockdown = flag("blast_knockdown", d.blastKnockdown);
    g_cfg.blast = num("blast_push", d.blast);
    ragcam::Settings();
    // the physics' own tables (jolt_ragdoll's muscles and joints, knock's thresholds): "jolt.<name>", "knock.<name>"
    for (auto [prefix, list, set] : {std::make_tuple("jolt.", &jolt_ragdoll::Tunes, &jolt_ragdoll::SetTune),
                                     std::make_tuple("knock.", &knock::Tunes, &knock::SetTune)}) {
        std::vector<tunables::Info> rows;
        list(rows);
        for (const tunables::Info &t : rows) {
            std::string key = std::string(prefix) + t.name;
            double v = H->config_number(M, key.c_str(), t.def);
            if (v != t.value) set(t.name.c_str(), static_cast<float>(v), nullptr);
        }
    }
}

void Start() {
    jolt_ragdoll::Init(mod::Log);
    knock::Init(mod::Log);
    Settings();
    Hooks();
    H->hit_filter(M, HitFilter, nullptr);
}

void Stop() {
    ragcam::Stop();
    for (auto &r : g_ragdolls) Unshape(r->actor);
    g_ragdolls.clear();
    jolt_ragdoll::Clear();
    jolt_world::Stop();
}

std::string State() {
    std::string list;
    for (auto &r : g_ragdolls) {
        V3 p = r->BodyPos(Ragdoll::B_PELVIS);
        float fastest = 0;
        for (int b = 0; b < Ragdoll::BodyCount(); b++) fastest = std::max(fastest, Len(r->BodyVel(b)));
        list += Fmt("%s{\"actor\":\"0x%08X\",\"pedestrian\":%s,\"knockdown\":%s,\"puppet\":%s,\"sleeping\":%s,\"age\":%.2f,"
                    "\"pelvis\":[%.3f,%.3f,%.3f],\"fastest_part\":%.4f}",
                    list.empty() ? "" : ",", (unsigned)(uintptr_t)r->actor, r->pedestrian ? "true" : "false", r->knockdown ? "true" : "false",
                    r->puppet ? "true" : "false", r->sleeping ? "true" : "false", r->age, p.x, p.y, p.z, fastest);
    }
    return Fmt("{\"enabled\":%s,\"ragdolls\":[%s],\"tracked\":%u,\"pedestrians\":%u,\"world\":%s,\"corpse_hits\":%u,\"dodge_fixes\":%u,"
               "\"car_stops\":%u,\"scenery\":\"%s\"}",
               g_cfg.enabled ? "true" : "false", list.c_str(), (unsigned)g_poses.size(), (unsigned)g_pedList.size(),
               jolt_world::Running() ? "true" : "false", g_corpseHits, g_dodgeFixes, g_carStops, scenery::Describe().c_str());
}

std::string Exec(const std::string &cmd, const std::string &args, bool *ok) {
    *ok = true;
    if (cmd == "start") {   // a ragdoll on someone alive (a puppet: no death; `drop` gives him back)
        void *who = reinterpret_cast<void *>(static_cast<uintptr_t>(strtoul(args.c_str(), nullptr, 16)));
        ListPeople();
        bool known = IsPed(who);
        for (void *h : g_humanList) known = known || h == who;
        if (!who || !known) { *ok = false; return "start: a person (humans.list) or a pedestrian (pedestrians.list) by its address"; }
        std::string err;
        Ragdoll *rd = Start(who, true, &err);
        if (!rd) { *ok = false; return "start: " + err; }
        return Fmt("{\"started\":\"0x%08X\"}", (unsigned)(uintptr_t)who);
    }
    if (cmd == "drop") {
        void *who = reinterpret_cast<void *>(static_cast<uintptr_t>(strtoul(args.c_str(), nullptr, 16)));
        if (!Find(who)) { *ok = false; return "drop: no ragdoll on that one"; }
        bool getup = Find(who)->puppet || Find(who)->knockdown;
        Drop(who);
        if (getup) GetUp(who);
        return "{\"dropped\":true}";
    }
    if (cmd == "jolt") return jolt_world::Describe() + "\n" + jolt_ragdoll::Describe();
    if (cmd == "knock") {
        std::string out;
        knock::Exec("knock", args, &out);
        return out;
    }
    *ok = false;
    return "unknown";
}

} // namespace core
