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
#include "headwear.h"
#include "dogs.h"
#include <windows.h>
#include <intrin.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
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
#include "pull_reach.h"
#include "jolt_world.h"
#include "knock.h"
#include "ragcam.h"
#include "mod.h"
#include "ragdoll.h"
#include "scenery.h"
#include "vehicles.h"
#include "pedestrian_position.h"
#include "mml/path.h"

using namespace mod;

namespace core {
namespace {

#define LOG(...) mod::Log(__VA_ARGS__)

// ---- settings (config.json; the game's Options > Mods)
struct HitStrength {
    float perDamage; // N s per point of native damage
    float maxImpulse; // N s per impact
    float maxPart; // m/s added locally in a tick
};
constexpr HitStrength kOtherHit{1.f, 120.f, 8.f};
struct Config {
    bool enabled = true;          // a ragdoll on death
    bool carKnockdown = true;     // a car knocks living people over as ragdolls (the player too)
    bool carFeel = true;          // cars feel the bodies they push
    float trackRadius = 60.0f;    // m: people this close to the player have their pose remembered
    bool bullets = true;          // bullets strike the bodies on the ground
    HitStrength shot{2.f, 240.f, 16.f};
    float whole = 0.6f;           // the share of a blow that moves the whole body
    float radius = 0.3f;          // m: the parts this close to a blow share the rest
    float remember = 0.3f;        // s: bullets in someone alive given to his body if he dies within this
    bool draw = false;            // the bodies drawn (debug)
    bool blastKnockdown = true;   // use physics for Tommy's native heavy blast fall
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
    V3 pedestrianPos{}, pedestrianVelocity{};
    int pedestrianAnimation = -1;
    void *pedestrianModel = nullptr;
};
std::unordered_map<void *, PoseHistory> g_poses;
struct RenderedPose { Ragdoll::PoseSnap pose{}; void *model = nullptr; unsigned tick = 0; uint64_t ms = 0; };
std::unordered_map<void *, RenderedPose> g_lastDrawn;

struct HandoverPose {
    unsigned tick = 0, stage = 0;
    float age = 0, error = 0, fastest = 0;
    V3 physical{}, drawn{}, element{};
};
struct HandoverContact {
    unsigned tick = 0;
    float age = 0;
    jolt_world::CarContactInfo contact{};
};
enum class TrajectoryStage { PreviousWorldDraw, BeforeHit, NativeHit, AfterHit, TickBegin, Physics, TickEnd, RenderBegin, WorldDrawn, Count };
struct PedestrianSample {
    uint64_t ms = 0;
    unsigned tick = 0;
    TrajectoryStage stage{};
    int animation = -1;
    float health = 0;
    V3 element{}, root{}, pelvis{}, body{}, velocity{};
    bool hasPelvis = false, physical = false, hasElement = true;
};
struct CarHandover {
    void *actor = nullptr, *model = nullptr;
    unsigned serial = 0, tick = 0;
    int animation = -1;
    bool preHitPose = false;
    bool open = false, captured = false, started = false, history = false, committed = false, stepped = false;
    Ragdoll::PoseSnap beforePose{};
    V3 elementBefore{}, modelBefore{}, initial{}, bodyStart{}, elementAfter{}, firstStep{};
    float poseGap = 0;
    Ragdoll *body = nullptr;
    std::array<HandoverPose, 128> poses{};
    unsigned poseCount = 0, lastPoseTick[2] = {};
    bool sampled[2] = {};
    float nextPoseAge[2] = {}, maxDrawError = 0;
    bool lastDrawnCaptured = false, worldSampled = false;
    unsigned lastDrawnTick = 0, lastWorldTick = 0;
    V3 lastDrawnPelvis{}, lastDrawnRoot{};
    float lastDrawnGap = 0, nextWorldAge = 0, maxWorldError = 0;
    std::array<HandoverContact, 192> contacts{};
    unsigned contactCount = 0, contactsDropped = 0, worldContactsDropped = 0;
    uint64_t traceStart = 0;
    void *pelvisFrame = nullptr, *car = nullptr;
    V3 carOrigin{}, carRight{}, carForward{}, hitVelocity{};
    float damage = 0;
    bool carBasis = false, nativeEvent = false, traceEnded = false, savedJump = false, savedEnd = false;
    const char *startResult = "not_attempted";
    std::array<PedestrianSample, 256> trajectory{};
    unsigned trajectoryCount = 0, trajectoryDropped = 0;
    unsigned firstJump = 0;
    std::array<uint64_t, static_cast<unsigned>(TrajectoryStage::Count)> lastSampleMs{};
    std::array<unsigned, static_cast<unsigned>(TrajectoryStage::Count)> lastSampleTick{};
};
std::array<CarHandover, 16> g_handovers;
unsigned g_handoverCount;

CarHandover *PendingHandover(void *actor) {
    for (unsigned i = 0; i < std::min<unsigned>(g_handoverCount, g_handovers.size()); ++i) {
        auto &entry = g_handovers[(g_handoverCount - 1 - i) % g_handovers.size()];
        if (entry.open && entry.actor == actor) return &entry;
    }
    return nullptr;
}

template <class F> bool Guarded(F &f) {
    __try { f(); return true; } __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}

// ---- people and pedestrians
constexpr int kPedPos = 0x10, kPedModel = 0x40, kPedHealth = 0x15C, kPedAnimation = 0x160, kPedActive = 0x0C;
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

bool FinitePoint(const V3 &p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }

const char *StageName(TrajectoryStage stage) {
    switch (stage) {
    case TrajectoryStage::PreviousWorldDraw: return "previous_world_draw";
    case TrajectoryStage::BeforeHit: return "before_hit";
    case TrajectoryStage::NativeHit: return "after_native_hit";
    case TrajectoryStage::AfterHit: return "after_hit_handler";
    case TrajectoryStage::TickBegin: return "after_native_tick";
    case TrajectoryStage::Physics: return "after_physics_step";
    case TrajectoryStage::TickEnd: return "after_ragdoll_tick";
    case TrajectoryStage::RenderBegin: return "before_render_pose";
    case TrajectoryStage::WorldDrawn: return "after_world_draw";
    case TrajectoryStage::Count: break;
    }
    return "unknown";
}

std::string TracePoint(const V3 &p) {
    return FinitePoint(p) ? Fmt("[%.5f,%.5f,%.5f]", p.x, p.y, p.z) : "null";
}

std::string TrajectoryJson(const CarHandover &entry) {
    std::string out = Fmt("{\"pid\":%u,\"serial\":%u,\"actor\":\"0x%p\",\"model\":\"0x%p\","
                          "\"car\":\"0x%p\",\"started\":%s,\"start_result\":\"%s\",\"native_event\":%s,"
                          "\"damage\":%.3f,\"first_jump_index\":%u,\"dropped\":%u,\"ended\":%s,",
                          (unsigned)GetCurrentProcessId(), entry.serial, entry.actor, entry.model, entry.car,
                          entry.started ? "true" : "false", entry.startResult, entry.nativeEvent ? "true" : "false",
                          entry.damage, entry.firstJump, entry.trajectoryDropped, entry.traceEnded ? "true" : "false");
    out += "\"hit_velocity\":" + TracePoint(entry.hitVelocity);
    if (entry.carBasis) {
        out += ",\"car_origin\":" + TracePoint(entry.carOrigin) + ",\"car_right\":" + TracePoint(entry.carRight) +
               ",\"car_forward\":" + TracePoint(entry.carForward);
    }
    out += ",\"samples\":[";
    for (unsigned i = 0; i < entry.trajectoryCount; ++i) {
        const auto &p = entry.trajectory[i];
        out += Fmt("%s{\"ms\":%lld,\"tick\":%u,\"stage\":\"%s\",\"animation\":%d,\"health\":%.4f,",
                   i ? "," : "", static_cast<long long>(p.ms) - static_cast<long long>(entry.traceStart), p.tick,
                   StageName(p.stage), p.animation, p.health);
        out += "\"element\":" + (p.hasElement ? TracePoint(p.element) : "null") + ",\"root\":" + TracePoint(p.root) +
               ",\"pelvis\":" + (p.hasPelvis ? TracePoint(p.pelvis) : "null") +
               ",\"body\":" + (p.physical ? TracePoint(p.body) : "null") +
               ",\"body_velocity\":" + (p.physical ? TracePoint(p.velocity) : "null") + "}";
    }
    return out + "]}";
}

bool WriteTrace(const char *name, const std::string &data, bool history = false) {
    const std::string path = std::string(H->mod_data_dir(M)) + name;
    const std::string temporary = path + ".new";
    wchar_t destination[1024], source[1024];
    if (!mml_widen(path.c_str(), destination, 1024) || !mml_widen(temporary.c_str(), source, 1024)) return false;
    FILE *file = mml_fopen(temporary.c_str(), "wb");
    if (!file) return false;
    const bool written = fwrite(data.data(), 1, data.size(), file) == data.size();
    const bool closed = fclose(file) == 0;
    if (written && closed && history && GetFileAttributesW(destination) != INVALID_FILE_ATTRIBUTES) {
        const std::wstring previous = std::wstring(destination)+L".prev";
        if (GetFileAttributesW(previous.c_str()) != INVALID_FILE_ATTRIBUTES)
            CopyFileW(previous.c_str(),(previous+L"2").c_str(),FALSE);
        if (ReplaceFileW(destination,source,previous.c_str(),0,nullptr,nullptr)) return true;
    }
    if (written && closed && MoveFileExW(source, destination, MOVEFILE_REPLACE_EXISTING)) return true;
    DeleteFileW(source);
    return false;
}

bool SaveTrajectory(const CarHandover &entry, const char *name) {
    return WriteTrace(name, TrajectoryJson(entry));
}

std::string TraceString(const std::string &value) {
    std::string out = "\"";
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') out += '\\';
        if (c >= 32 && c < 127) out += static_cast<char>(c);
        else out += Fmt("\\u%04x", unsigned(c));
    }
    return out + '"';
}

struct ArmPoints {
    V3 shoulder[2], elbow[2], wrist[2];

    void Read(const Ragdoll::PoseSnap &pose) {
        for (int i = 0; i < 2; ++i) {
            const int arm = i ? Ragdoll::RARM : Ragdoll::LARM;
            shoulder[i] = pose.pts[arm];
            elbow[i] = pose.pts[arm + 1];
            wrist[i] = pose.pts[arm + 2];
        }
    }
    std::string Json() const {
        auto pair = [](const V3 (&p)[2]) { return "[" + TracePoint(p[0]) + "," + TracePoint(p[1]) + "]"; };
        return "{\"shoulder\":" + pair(shoulder) + ",\"elbow\":" + pair(elbow) + ",\"wrist\":" + pair(wrist) + "}";
    }
};

struct PullSample {
    float t, error;
    V3 left, right, coat, chest, pelvis, feet[2], origin, side;
    V3 head, drawnHead, drawnNose;
    Ragdoll::Quat neck;
    bool hands, carPose, drawn;
    V3 contact[2]{}, drawnWrist[2]{};
    float ikWeight = 0;
    ArmPoints nativeArms{}, drawnArms{};
    bool armsRecorded = false;
    const char *phase = "animation";
    float releaseWeight{};
    const char *gripPart = "";
    std::string cabinContacts;
};

struct PullTrace {
    Ragdoll *body = nullptr;
    unsigned bodySerial = 0;
    void *car = nullptr;
    V3 left{}, right{};
    float t = 0, settling = 0, wheelEdge = 0;
    std::string carModel;
    bool hands = false;
    std::vector<PullSample> rows;
} g_pull;

void EndPullTrace(const char *reason) {
    if (!g_pull.body) return;
    jolt_world::WatchDriver(nullptr, nullptr);
    std::string out = Fmt("{\"end\":\"%s\",\"body_serial\":%u,\"outer_tyre_x_m\":%.4f,", reason, g_pull.bodySerial, g_pull.wheelEdge);
    out += "\"car_model\":" + TraceString(g_pull.carModel) + ",\"samples\":[";
    for (const auto &s : g_pull.rows) {
        if (out.back() != '[') out += ',';
        out += Fmt("{\"t\":%.5f,\"gap\":%.5f", s.t, s.error);
        out += ",\"left\":" + (s.hands ? TracePoint(s.left) : "null") + ",\"right\":" + (s.hands ? TracePoint(s.right) : "null");
        out += ",\"coat\":" + (s.error >= 0 ? TracePoint(s.coat) : "null") + ",\"chest\":" + TracePoint(s.chest);
        out += ",\"pelvis\":" + TracePoint(s.pelvis) + ",\"feet\":[" + TracePoint(s.feet[0]) + "," + TracePoint(s.feet[1]) + "]";
        out += ",\"cabin_contacts\":[" + s.cabinContacts + "]";
        out += ",\"head\":" + TracePoint(s.head);
        out += Fmt(",\"neck_q\":[%.6f,%.6f,%.6f,%.6f]", s.neck.w, s.neck.x, s.neck.y, s.neck.z);
        out += ",\"drawn_head\":" + (s.drawn ? TracePoint(s.drawnHead) : "null");
        out += ",\"drawn_nose\":" + (s.drawn ? TracePoint(s.drawnNose) : "null");
        if (s.armsRecorded)
            out += ",\"arm_phase\":\"" + std::string(s.phase) + "\",\"native_arms\":" + s.nativeArms.Json() +
                   ",\"drawn_arms\":" + s.drawnArms.Json();
        if (s.ikWeight > 0)
            out += Fmt(",\"ik_weight\":%.5f,\"grip_part\":\"%s\"", s.ikWeight, s.gripPart) +
                ",\"grip_contacts\":[" + TracePoint(s.contact[0]) + "," + TracePoint(s.contact[1]) + "]" +
                ",\"drawn_wrists\":[" + TracePoint(s.drawnWrist[0]) + "," + TracePoint(s.drawnWrist[1]) + "]";
        if (s.releaseWeight > 0) out += Fmt(",\"release_weight\":%.5f",s.releaseWeight);
        out += ",\"car\":" + (s.carPose ? TracePoint(s.origin) : "null") + ",\"side\":" + (s.carPose ? TracePoint(s.side) : "null") + "}";
    }
    if (!WriteTrace("driver-pull.json", out + "]}", true)) LOG("could not save driver-pull.json");
    g_pull = {};
}

void BeginPullTrace(Ragdoll *body, const void *car) {
    EndPullTrace("next pull");
    g_pull.body = body;
    g_pull.bodySerial = body->serial;
    g_pull.car = const_cast<void *>(car);
    g_pull.rows.reserve(400);
    jolt_world::WatchDriver(body, car);
    char name[512]{};
    if (MML_HAS(H, model_file) && H->model_file && H->model_file(M, ModelOf(g_pull.car), name, sizeof name))
        g_pull.carModel = name;
    smoke::Mover mover;
    if (!vehicles::MoverOf(g_pull.car, &mover) || !mover.hullParts) return;
    const float side = Dot(body->BodyPos(Ragdoll::B_PELVIS) - mover.c, mover.ax[0]) >= 0 ? 1.f : -1.f;
    for (size_t i = 0; i < mover.hullParts->size() && i < 64; ++i)
        if (mover.wheelParts & (uint64_t(1) << i))
            for (const V3 &p : (*mover.hullParts)[i])
                if (side * p.x > side * g_pull.wheelEdge) g_pull.wheelEdge = p.x;
}

void SamplePull(float dt) {
    if (!g_pull.body || dt <= 0) return;
    const auto &body = *g_pull.body;
    PullSample s{};
    s.t = g_pull.t += dt;
    s.hands = g_pull.hands;
    s.left = g_pull.left;
    s.right = g_pull.right;
    g_pull.hands = false;
    s.error = jolt_ragdoll::GripDistance(&body, &s.coat);
    s.head = body.BodyPos(Ragdoll::B_HEAD);
    s.neck = body.NeckRotation();
    s.chest = body.BodyPos(Ragdoll::B_CHEST);
    s.pelvis = body.BodyPos(Ragdoll::B_PELVIS);
    s.feet[0] = body.BodyPos(Ragdoll::B_LFOOT);
    s.feet[1] = body.BodyPos(Ragdoll::B_RFOOT);
    V3 axes[3];
    s.carPose = g_pull.car && vehicles::Pose(g_pull.car, &s.origin, axes);
    if (s.carPose) s.side = axes[0];
    for (const auto &contact : jolt_world::DriverContacts()) {
        if (!s.cabinContacts.empty()) s.cabinContacts += ',';
        const std::string name = vehicles::TriangleVisual(g_pull.car, jolt_world::CabinVersion(g_pull.car), contact.triangle);
        s.cabinContacts += Fmt("{\"part\":%u,\"triangle\":%u,\"depth_m\":%.5f,", contact.part, contact.triangle, contact.depth);
        s.cabinContacts += "\"mesh\":" + TraceString(name) + ",\"point\":" + TracePoint(contact.point) +
                          ",\"normal\":" + TracePoint(contact.normal) + "}";
    }
    g_pull.rows.push_back(std::move(s));
    if (!body.carExtraction) g_pull.settling += dt;
    if (g_pull.settling >= 1.5f) EndPullTrace("completed");
    else if (g_pull.rows.size() >= 2048) EndPullTrace("sample limit");
}

void TracePedestrian(CarHandover &entry, TrajectoryStage stage, uint64_t now) {
    if (!entry.actor || entry.traceEnded) return;
    if (now - entry.traceStart > 1600 || !IsPed(entry.actor)) {
        entry.traceEnded = true;
        return;
    }
    PedestrianSample sample{};
    sample.tick = g_tick;
    sample.stage = stage;
    sample.ms = now;
    bool valid = false;
    auto read = [&] {
        if (ModelOf(entry.actor) != entry.model || !game::at<uint8_t>(entry.actor, kPedActive)) return;
        const float *root = game::WorldMatrix(entry.model);
        if (!root) return;
        sample.element = game::at<V3>(entry.actor, kPedPos);
        sample.root = {root[12], root[13], root[14]};
        sample.health = game::at<float>(entry.actor, kPedHealth);
        sample.animation = game::at<int>(entry.actor, kPedAnimation);
        if (entry.pelvisFrame) {
            if (const float *pelvis = game::WorldMatrix(entry.pelvisFrame)) {
                sample.pelvis = {pelvis[12], pelvis[13], pelvis[14]};
                sample.hasPelvis = FinitePoint(sample.pelvis);
            }
        }
        if (Ragdoll *body = Find(entry.actor); body && body->Model() == entry.model && jolt_ragdoll::Has(body)) {
            sample.body = body->BodyPos(Ragdoll::B_PELVIS);
            sample.velocity = body->BodyVel(Ragdoll::B_PELVIS);
            sample.physical = true;
        }
        valid = FinitePoint(sample.root) && FinitePoint(sample.element) && std::isfinite(sample.health);
    };
    if (!Guarded(read) || !valid) { entry.traceEnded = true; return; }
    if (entry.trajectoryCount == entry.trajectory.size()) { ++entry.trajectoryDropped; return; }
    if (entry.trajectoryCount && !entry.firstJump) {
        const auto &previous = entry.trajectory[entry.trajectoryCount - 1];
        const float distance = std::max({Len(sample.root - previous.root), previous.hasElement ? Len(sample.element - previous.element) : 0.0f,
            sample.hasPelvis && previous.hasPelvis ? Len(sample.pelvis - previous.pelvis) : 0.0f,
            sample.physical && previous.physical ? Len(sample.body - previous.body) : 0.0f});
        if (distance >= .75f && now - previous.ms <= 200) entry.firstJump = entry.trajectoryCount;
    }
    entry.trajectory[entry.trajectoryCount++] = sample;
}

unsigned g_lastSavedTrajectory;
void PersistTrajectory(CarHandover &entry) {
    if (entry.firstJump && !entry.savedJump) {
        entry.savedJump = true;
        if (!SaveTrajectory(entry, "pedestrian-jump.json")) LOG("pedestrian %p: could not save jump trace", entry.actor);
    }
    if (entry.traceEnded && !entry.savedEnd && entry.nativeEvent && entry.serial > g_lastSavedTrajectory) {
        entry.savedEnd = true;
        g_lastSavedTrajectory = entry.serial;
        if (!SaveTrajectory(entry, "pedestrian-contact.json")) LOG("pedestrian %p: could not save contact trace", entry.actor);
    }
}

void TracePedestrians(TrajectoryStage stage) {
    const uint64_t now = GetTickCount64();
    for (auto &entry : g_handovers) {
        if (!entry.actor) continue;
        if (entry.traceEnded) { PersistTrajectory(entry); continue; }
        const auto index = static_cast<unsigned>(stage);
        const uint64_t last = entry.lastSampleMs[index];
        // Bound render passes per tick. After the initial burst, keep up to 30 samples/s per phase.
        if (last && (entry.lastSampleTick[index] == g_tick || (now - entry.traceStart > 200 && now - last < 33))) continue;
        entry.lastSampleMs[index] = now;
        entry.lastSampleTick[index] = g_tick;
        TracePedestrian(entry, stage, now);
        PersistTrajectory(entry);
    }
}

// the frames LS3D frees (frame_watch): forgotten at the next tick or frame, before anything of ours reads them
std::unordered_set<void *> g_freed;
std::unordered_set<void *> g_watched;
void FrameGone(void *frame, void *) {
    g_freed.insert(frame);
    g_watched.erase(frame);
    for (auto &entry : g_handovers) {
        if (entry.model == frame || entry.pelvisFrame == frame) entry.traceEnded = true;
    }
}
void Watch(void *frame) {
    if (frame && g_watched.insert(frame).second) H->frame_watch(M, frame, FrameGone, nullptr);
}
void WatchRagdoll(const Ragdoll *rd) {
    Watch(rd->Model());
    for (int i = 0; i < 20; i++) Watch(rd->Frames()[i]);
}

void Unshape(void *actor) { H->bullet_shape(M, actor, nullptr, 0); }
void StopHandoverPose(void *actor) {
    if (g_pull.body && g_pull.body->actor == actor) EndPullTrace("body removed");
    for (auto &entry : g_handovers) if (entry.actor == actor) entry.body = nullptr;
}

void Drop(void *actor) {
    for (size_t i = 0; i < g_ragdolls.size(); i++)
        if (g_ragdolls[i]->actor == actor) {
            StopHandoverPose(actor);
            g_ragdolls.erase(g_ragdolls.begin() + i);
            Unshape(actor);
            LOG("ragdoll dropped %p", actor);
            return;
        }
}
void EraseAt(size_t i) {
    StopHandoverPose(g_ragdolls[i]->actor);
    Unshape(g_ragdolls[i]->actor);
    g_ragdolls.erase(g_ragdolls.begin() + i);
}

void PurgeOccupantFrames(std::span<Ragdoll *const> rebound);
void Purge(std::span<Ragdoll *const> rebound = {}) {
    if (g_freed.empty()) return;
    PurgeOccupantFrames(rebound);
    for (size_t i = 0; i < g_ragdolls.size();) {
        Ragdoll *rd = g_ragdolls[i].get();
        if (std::find(rebound.begin(), rebound.end(), rd) == rebound.end() &&
            (g_freed.count(rd->Model()) || rd->UsesAny(g_freed))) {
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
        Ragdoll::PoseSnap pose;
        std::string err;
        V3 dir = IsPed(key) ? V3{} : game::at<V3>(key, game::kActorFacing);
        ok = Ragdoll::Snapshot(ModelOf(key), IsPed(key) ? nullptr : &dir, &pose, &err);
        if (ok) memcpy(pts, pose.pts, sizeof pose.pts);
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
const V3 *PreviousPose(const PoseHistory &h, const V3 *now, float *dt) {
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

std::unordered_map<const void *, smoke::CabinInterior> g_wreckInteriors;

void RememberWreckInterior(void *car, void *model, const V3 *points) {
    if (!car || g_wreckInteriors.contains(car)) return;
    char file[512];
    if (!H->model_file(M, model, file, sizeof file)) return;
    const char *base = std::strrchr(file, '\\');
    if (_stricmp(base ? base + 1 : file, "ohorelec01.i3d")) return;
    V3 origin, axes[3];
    if (!vehicles::Pose(car, &origin, axes)) return;
    auto local = [&](V3 point) {
        const V3 delta = point - origin;
        return V3{0, Dot(delta, axes[1]), Dot(delta, axes[2])};
    };
    // Sample the centreline from the footwell to the neck. A seat or dashboard
    // can obscure a lower door panel from a single point higher in the cabin.
    const V3 pelvis = points[Ragdoll::PELVIS], neck = points[Ragdoll::NECK];
    const V3 feet = (points[Ragdoll::RANKLE] + points[Ragdoll::LANKLE]) * .5f;
    g_wreckInteriors.try_emplace(car, smoke::CabinInterior{
        local(neck), local((pelvis + neck) * .5f), local(pelvis), local((pelvis + feet) * .5f)});
}

// ---- people in a tram (a vehicle they walk in, type 8): the game keeps such a person in the tram's space; a ragdoll
// lives in the world, so he is taken out the game's own way when it starts and put back when he gets up on its floor
// (game_build.h humanPlatform, tramCarOf; C_human +0x9C the vehicle to go into, +0x584 which of its cars: both builds)
constexpr int kHumanToVehicle = 0x9C, kHumanTramCar = 0x584;
struct RailPassenger { void *tram; int car; float deck; };
std::unordered_map<void *, RailPassenger> g_railPassengers;
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
// The game keeps one private collision grid in each carriage's own coordinates.
struct Cabin {
    void *frame;
    void *grid;
    bool Open(void *tram, int car) {
        if (car < 0 || car > game::at<int>(tram, game::kRailwayVagonCount)) return false;
        auto grids = game::at<uint8_t *>(tram, 0xFC);
        auto cars = game::at<void **>(tram, game::kRailwayVagons);
        frame = car == 0 ? game::at<void *>(tram, game::kActorFrame) : cars ? cars[car-1] : nullptr;
        grid = grids ? grids + 16 * car : nullptr;
        return frame && grid;
    }
    V3 Local(V3 world) const {
        const float *m = game::WorldMatrix(frame);
        return M3::Raw(m).Inverse().Apply(world - V3{m[12], m[13], m[14]});
    }
    V3 World(V3 local) const {
        const float *m = game::WorldMatrix(frame);
        return M3::Raw(m).Apply(local) + V3{m[12], m[13], m[14]};
    }
    bool Floor(V3 from, float depth, float *height) const {
        using Ray = void *(__thiscall *)(void *, void *, const V3 *, const V3 *, V3 *, V3 *, int);
        const V3 delta{0, -depth, 0};
        V3 hit{}, normal{};
        auto ray = reinterpret_cast<Ray>(game::Exe().testLineInVehicle);
        if (!ray(game::Collision(), grid, &from, &delta, &hit, &normal, 2) || normal.y < .5f) return false;
        *height = hit.y;
        return true;
    }
    bool Standing(V3 feet, float *height) const {
        // The same ray and exclusion mask as C_human::MovementIntern on a railway.
        return Floor(feet + V3{0, .5f, 0}, 1.5f, height);
    }
};

void EnterTramUnder(void *h, const V3 &pelvis) {
    if (!game::Exe().humanPlatform || IsPed(h) || TramOf(h)) return;
    // Resolve the fitted root against the carriage's private walking geometry. Render meshes
    // include steps and overhangs and do not cover every trailing carriage.
    std::vector<void *> nearby;
    vehicles::ListVehicles(pelvis, 80.f, nearby);
    const V3 root = game::ActorWorldPos(h);
    void *v = nullptr;
    int car = -1;
    float deck = 0, distance = 1.f;
    for (void *candidate : nearby) {
        if (game::at<int>(candidate, game::kActorType) != game::kActorTypeRailway) continue;
        const int index = SafeTramCar(candidate, pelvis);
        Cabin cabin{};
        float floor;
        if (!cabin.Open(candidate, index)) continue;
        const V3 local = cabin.Local(root);
        if (!cabin.Standing(local, &floor)) continue;
        const float gap = local.y - floor;
        if (gap < -.1f || std::fabs(gap) >= distance) continue;
        v = candidate;
        car = index;
        deck = floor;
        distance = std::fabs(gap);
    }
    if (!v) return;
    game::at<int>(h, kHumanTramCar) = car;
    game::at<void *>(h, kHumanToVehicle) = v;
    if (!SafePlatform(h, true) || TramOf(h) != v) return;
    g_railPassengers[h] = {v, car, deck};
    if (h == H->human_player(M)) {
        g_playerRisingUntil = 0;
        ragcam::Stop();
        MmlGameCamera camera{};
        camera.size = sizeof camera;
        if (MML_HAS(H, game_camera) && H->game_camera && H->game_camera(M, &camera) && camera.mode == 17) {
            // The native track now moves with the carriage. The hidden visual blend still targets the old world pelvis.
            if (MML_HAS(H, human_get_up_cancel) && H->human_get_up_cancel) H->human_get_up_cancel(M, h);
            // Intern_UseRailway(false) shows this same player frame when he leaves.
            void *model = game::at<void *>(h, game::kActorFrame);
            using SetOn = void(__stdcall *)(void *, bool);
            reinterpret_cast<SetOn>((*reinterpret_cast<void ***>(model))[0x24 / 4])(model, false);
        }
    }
    LOG("%p recovered in railway %p, carriage %d, native floor %.3f", h, v, car, deck);
}

// A steep railway brace can be the first intersection above a walkable track or street.
bool CrossedGround(const V3 &feet, float deckY, MmlHit *ground) {
    float from[3] = {feet.x, deckY, feet.z};
    const float to[3] = {feet.x, feet.y, feet.z};
    while (from[1] > to[1] && H->world_ray(M, from, to, MML_WORLD_STATIC, nullptr, ground)) {
        const float y = ground->pos[1];
        if (!(y <= from[1] && y >= to[1])) break;
        const unsigned material = ground->flags >> 24;
        if (material == 31 || material == 44) return false;  // Native water entry handles the fall.
        if (ground->normal[1] > .5f) return true;
        const float next = y - .01f;
        if (!(next < from[1])) break;
        from[1] = next;
    }
    return false;
}

bool WatchPassenger(void *h, RailPassenger *known) {
    void *tram = TramOf(h);
    if (!tram || IsDead(h)) return false;
    const int car = game::at<int>(h, kHumanTramCar);
    if (known && (known->tram != tram || known->car != car)) return false;
    Cabin cabin{};
    void *model = game::at<void *>(h, game::kActorFrame);
    if (!model || !cabin.Open(tram, car) || game::at<void *>(model, game::kFrameParent) != cabin.frame) return false;
    // Native boarding, exits, stationary passengers and an upward jump keep their owner.
    if (game::at<int>(h, game::Exe().humanState) != game::kHumanStateAir ||
        game::at<float>(h, game::kHumanFallSpeed) <= 0) return true;
    V3 feet = game::at<V3>(h, game::kActorPos);
    float deck;
    if (cabin.Standing(feet, &deck)) {
        if (known) known->deck = deck;
        return true;
    }
    // An unsupported foot can be jumping or stepping. Only leave once below a confirmed deck.
    // On hot load use the railway's own centre-floor probe from C_railway::GameInit.
    if (known) deck = known->deck;
    else if (!cabin.Floor({0, .8f, 0}, 1.6f, &deck)) return true;
    if (feet.y >= deck - .5f) return true;

    const V3 world = cabin.World(feet);
    const V3 rail = cabin.World({feet.x, deck, feet.z});
    // Only recover a surface the passenger has already crossed. Over open air the native fall continues.
    MmlHit ground{};
    const bool crossed = CrossedGround(world, rail.y, &ground);
    if (!SafePlatform(h, false) || TramOf(h)) return true;
    if (MML_HAS(H, human_get_up_cancel) && H->human_get_up_cancel) H->human_get_up_cancel(M, h);
    if (crossed) {
        const MmlVec3 at{ground.pos[0], ground.pos[1] + .03f, ground.pos[2]};
        if (H->actor_teleport(M, h, &at, nullptr)) game::at<float>(h, game::kHumanFallSpeed) = 0;
    }
    LOG("%p left unsupported railway %p, carriage %d, feet %.3f below deck%s", h, tram, car,
        deck - feet.y, crossed ? " (crossed world surface)" : "");
    return false;
}

void SafeWatchTramGetUps() {
    auto observe = [](void *actor, RailPassenger *known) {
        bool keep = false;
        auto check = [&] { keep = WatchPassenger(actor, known); };
        return Guarded(check) && keep;
    };
    void *player = H->human_player(M);
    if (player && !g_railPassengers.count(player)) observe(player, nullptr);
    for (auto it = g_railPassengers.begin(); it != g_railPassengers.end();)
        if (observe(it->first, &it->second)) ++it;
        else it = g_railPassengers.erase(it);
}

// ---- bullets in bodies (mafia-ragdoll's hits.cpp, its blows; the loader gives the bodies' bullets)
struct Kept { void *actor; V3 J, at; bool hasAt; DWORD tick; float maxPart; };
std::vector<Kept> g_kept;
unsigned g_corpseHits, g_shapesGiven;
struct BulletContact { void *actor; V3 at; int shape, part; float localDv; unsigned tick; };
BulletContact g_lastBullet{};

// A hit's momentum (N s): the game's damage times the kick, held to the strongest one shot gives (a script's or a
// test's damage of 1000 threw a body 12 m; a shotgun's pellets are each a hit of their own)
float Momentum(float damage, const HitStrength &strength) { return std::min(damage * strength.perDamage, strength.maxImpulse); }
float Kick(Ragdoll *rd, const V3 &dir, const V3 &at, float damage, int part, const HitStrength &strength) {
    if (!rd || damage <= 0 || Len(dir) < 1e-4f) return 0;
    return rd->Kick(Norm(dir) * Momentum(damage, strength), at, g_cfg.whole, g_cfg.radius, strength.maxPart, part);
}
void Remember(void *actor, const V3 &dir, const V3 *at, float damage, const HitStrength &strength) {
    if (!actor || damage <= 0 || Len(dir) < 1e-4f || g_cfg.remember <= 0) return;
    if (g_kept.size() >= 256) g_kept.erase(g_kept.begin());
    g_kept.push_back({actor, Norm(dir) * Momentum(damage, strength), at ? *at : V3{0, 0, 0}, at != nullptr, GetTickCount(), strength.maxPart});
}
void TakeRemembered(void *actor, Ragdoll *rd) {
    DWORD now = GetTickCount(), keep = (DWORD)(g_cfg.remember * 1000.0f);
    for (size_t i = 0; i < g_kept.size();) {
        const Kept &k = g_kept[i];
        if (k.actor != actor) { i++; continue; }
        if (rd && now - k.tick <= keep) rd->Kick(k.J, k.hasAt ? k.at : rd->Center(), g_cfg.whole, g_cfg.radius, k.maxPart);
        g_kept.erase(g_kept.begin() + i);
    }
}
void KeptTick() {
    DWORD now = GetTickCount(), keep = (DWORD)(g_cfg.remember * 1000.0f) + 100;
    g_kept.erase(std::remove_if(g_kept.begin(), g_kept.end(), [&](const Kept &k) { return now - k.tick > keep; }), g_kept.end());
}

// Register corpse parts only. Living actors, including temporary knockdowns, use the game's bullet collision.
void GiveShapes() {
    MmlCapsule caps[Ragdoll::BulletShapeCount()]{};
    for (auto &r : g_ragdolls) {
        if (!g_cfg.enabled || !g_cfg.bullets || !IsDead(r->actor)) {
            Unshape(r->actor);
            r->shapeAtRest = false;
            continue;
        }
        // the loader keeps the capsules until they are given again: a body asleep gives them once
        if (r->sleeping && r->shapeAtRest) continue;
        r->BulletCapsules(caps);
        bool registered = H->bullet_shape(M, r->actor, caps, Ragdoll::BulletShapeCount()) != 0;
        if (registered) g_shapesGiven++;
        r->shapeAtRest = registered && r->sleeping;
    }
}

// ---- cars, tracked every tick from their model frame (the speed of a car that hits someone)
struct CarTrack { V3 pos, vel; unsigned tick; float speed = 0; unsigned reported = 0; float joltDelta = 0; };
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
        float speed = game::at<uint32_t>(a, game::kActorType) == 4 ?
            Len(game::at<V3>(static_cast<uint8_t *>(a) + game::Exe().vehicle, game::Exe().vehVelocity)) : 0.0f;
        auto it = g_carTracks.find(a);
        if (it != g_carTracks.end() && g_tick - it->second.tick == 1) {
            CarTrack &track = it->second;
            // Rare hard-stop reports need evidence from the same frame, rather than inferring from a later
            // ragdoll event. Logging has no collision policy and is limited to one report per car per 60 ticks.
            if (track.speed - speed > 1.0f && track.speed > 1.5f && g_tick - track.reported > 60 &&
                Len(p - track.pos) < 1.0f) {
                void *nearest = nullptr;
                float distance = 6.0f;
                V3 element{}, model{};
                int animation = -1;
                for (const MmlPedestrian &ped : g_pedList) {
                    if (!ped.model || !ped.spawned) continue;
                    const float *w = game::WorldMatrix(ped.model);
                    V3 at{w[12],w[13],w[14]};
                    V3 stored = game::at<V3>(ped.element, kPedPos);
                    float d = std::min(Len(at - p), Len(stored - p));
                    if (d >= distance) continue;
                    nearest = ped.element;
                    distance = d;
                    model = at;
                    element = stored;
                    animation = game::at<int>(ped.element, 0x160);
                }
                if (nearest) {
                    LOG("car hard-stop %p: %.2f -> %.2f m/s (dt %.3f), Jolt delta %.3f m/s; nearest pedestrian %p "
                        "%.2f m, anim %d, element/model gap %.3f m, physical %d; car at %.2f %.2f %.2f",
                        a, track.speed, speed, dt, track.joltDelta, nearest, distance, animation,
                        Len(element-model), Find(nearest) != nullptr, p.x,p.y,p.z);
                    track.reported = g_tick;
                }
            }
            V3 v = (p - it->second.pos) * (1.0f / dt);
            it->second.vel = it->second.vel * 0.5f + v * 0.5f;   // (the frames' jitter smoothed)
        } else if (it == g_carTracks.end()) {
            g_carTracks[a] = {p, {0, 0, 0}, g_tick, speed};
            continue;
        } else {
            it->second.vel = {0, 0, 0};
        }
        it->second.pos = p;
        it->second.tick = g_tick;
        it->second.speed = speed;
        it->second.joltDelta = 0;
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
        auto track = g_carTracks.find(car);
        if (track != g_carTracks.end()) track->second.joltDelta += Len(dv);
        game::at<V3>(veh, x.vehVelocity) += dv;
        game::at<V3>(veh, x.vehTurn) -= dw;
    }
}
bool SafeCarReactions() {
    auto f = [&] { JoltCarKicks(); };
    return Guarded(f);
}

// intern_ThrowMeFromCar reparents the victim to the world, clears the vehicle and plays
// one of four dead-occupant tracks (1.2: 588B40, 1.0: 4A90B0). The player's paired
// pull lasts longer than the victim's clip, so keep the grip until Tommy lets go.
constexpr int kHumanPlaying = 0x70; // C_human::PersonAnim, also read by MML human_anim_state
bool BeingPulledFromCar(void *human) {
    if (game::at<void *>(human, game::kHumanVehicle)) return false;
    const int clip = game::at<int>(human, kHumanPlaying);
    return clip >= 312 && clip <= 315;
}

struct RemoveOccupantProp {
    void operator()(void *prop) const { H->prop_remove(M, prop); }
};

// Roster occupants have no C_human. An exploded traffic car's occupant keeps
// its physical body and adopts an owned charred model.
struct Occupant {
    void *carrier{}, *record{}, *model{}, *frame{};
    void *clone{};
    unsigned cloneTick{};
    int slot{}, carriage{};
    bool rail{}, driver{}, sampleDraw{};
    float drawnNeckError = -1;
    std::unique_ptr<void, RemoveOccupantProp> wreckModel;
    std::unique_ptr<Ragdoll> body;
    struct LocalFrame { Q rotation; V3 position; };
    std::array<LocalFrame,20> nativePose{};
    bool posedForDraw{};
};
std::vector<Occupant> g_occupants;

// NewExplosion runs inside CarExplosion before it registers the replacement.
// Both builds set the replacement's C_car wreck flag at +0x65.
constexpr unsigned kCarIsWreck = 0x65;
constexpr float kWreckSearchRadius = 4.f;
struct WreckSearch {
    V3 origin;
    std::vector<void *> existing;
};
std::unordered_map<void *, WreckSearch> g_wreckSearches;

std::vector<void *> WrecksNear(V3 at) {
    const int count = H->world_near(M, &at.x, kWreckSearchRadius, MML_ACTOR_CAR, nullptr, 0);
    if (count <= 0) return {};
    std::vector<void *> candidates(count);
    const int found = H->world_near(M, &at.x, kWreckSearchRadius, MML_ACTOR_CAR, candidates.data(), count);
    candidates.resize(std::clamp(found, 0, count));
    std::erase_if(candidates, [](void *car) { return !game::at<uint8_t>(car, kCarIsWreck); });
    return candidates;
}

struct RailSample {
    uintptr_t model, carrier;
    unsigned tick, serial, mesh, shotTick;
    int carriage, phase;
    float dt, age, speed, target, poseError, drawnError, shotDv;
    V3 origin, pelvis, head, relativeVelocity, cabinVelocity, cabinAngular;
    jolt_world::RailwayStep contacts;
};
std::deque<RailSample> g_railTrace;

void SampleRailway(float dt) {
    if (dt <= 0.f) return;
    for (const auto &o : g_occupants) {
        if (!o.rail || !jolt_ragdoll::SeatedCar(o.body.get())) continue;
        const V3 pelvis = o.body->BodyPos(Ragdoll::B_PELVIS);
        jolt_world::RailwayStep step;
        if (!jolt_world::RailwaySample(o.frame, o.body.get(), pelvis, &step)) continue;
        const float *matrix = game::WorldMatrix(o.frame);
        const M3 inverse = M3::FromWorld(matrix).T();
        const V3 origin{matrix[12],matrix[13],matrix[14]};
        const V3 head = o.body->BodyPos(Ragdoll::B_HEAD);
        const bool shot = g_lastBullet.actor == o.model && g_tick-g_lastBullet.tick <= 1;
        g_railTrace.push_back({
            reinterpret_cast<uintptr_t>(o.model), reinterpret_cast<uintptr_t>(o.carrier),
            g_tick, o.body->serial, jolt_world::CabinVersion(o.frame), shot ? g_lastBullet.tick : 0u,
            o.carriage, game::at<int>(o.carrier,0xdc),
            dt, o.body->age, game::at<float>(o.carrier,0xd4), game::at<float>(o.carrier,0xd0),
            Len(step.cabinPosition-origin), o.drawnNeckError, shot ? g_lastBullet.localDv : 0.f,
            origin, inverse.Apply(pelvis-origin), inverse.Apply(head-origin),
            inverse.Apply(o.body->BodyVel(Ragdoll::B_PELVIS)-step.pointVelocity),
            inverse.Apply(step.cabinVelocity), inverse.Apply(step.cabinAngular), step
        });
        constexpr std::size_t keptSamples = 960;
        if (g_railTrace.size() > keptSamples) g_railTrace.pop_front();
    }
}

std::string RailTrace(const std::string &args) {
    const uintptr_t model = strtoul(args.c_str(), nullptr, 16);
    std::string out = "[";
    for (const auto &s : g_railTrace) {
        if (model && s.model != model) continue;
        if (out.size() > 1) out += ',';
        out += Fmt("{\"tick\":%u,\"dt\":%.4f,\"age\":%.3f,\"model\":\"0x%08X\",\"body_serial\":%u,"
                   "\"carrier\":\"0x%08X\",\"carriage\":%d,\"phase\":%d,\"speed\":%.4f,\"target\":%.4f,"
                   "\"mesh\":%u,\"pose_error\":%.4f,\"drawn_neck_error\":%.4f,\"origin\":[%.4f,%.4f,%.4f],"
                   "\"pelvis\":[%.4f,%.4f,%.4f],\"head\":[%.4f,%.4f,%.4f],"
                   "\"relative_velocity\":[%.4f,%.4f,%.4f],\"cabin_velocity\":[%.4f,%.4f,%.4f],"
                   "\"cabin_angular\":[%.4f,%.4f,%.4f],\"contacts\":%u,\"end_contacts\":%u,"
                   "\"end_approach\":%.4f,\"end_point_z\":%.4f,\"outside_contacts\":%u,\"outside_depth\":%.4f,"
                   "\"outside_layer\":%u,\"outside_visual\":\"0x%08X\",\"solid_panels\":%u,"
                   "\"shot_tick\":%u,\"shot_local_dv\":%.4f}",
                   s.tick,s.dt,s.age,(unsigned)s.model,s.serial,(unsigned)s.carrier,s.carriage,s.phase,s.speed,s.target,
                   s.mesh,s.poseError,s.drawnError,s.origin.x,s.origin.y,s.origin.z,
                   s.pelvis.x,s.pelvis.y,s.pelvis.z,s.head.x,s.head.y,s.head.z,
                   s.relativeVelocity.x,s.relativeVelocity.y,s.relativeVelocity.z,
                   s.cabinVelocity.x,s.cabinVelocity.y,s.cabinVelocity.z,s.cabinAngular.x,s.cabinAngular.y,s.cabinAngular.z,
                   s.contacts.contacts,s.contacts.endContacts,s.contacts.endApproach,s.contacts.endPointZ,
                   s.contacts.outsideContacts,s.contacts.outsideDepth,s.contacts.outsideLayer,
                   (unsigned)(uintptr_t)s.contacts.outsideVisual,s.contacts.solidPanels,s.shotTick,s.shotDv);
    }
    return out + ']';
}

void ApplyRailPose(Occupant &o) {
    if (o.posedForDraw) return;
    void *const *frames=o.body->Frames();
    for (int i=0;i<20;++i) {
        const auto *local=&game::at<float>(frames[i],game::kFrameLocal);
        game::WorldMatrix(frames[i]);
        o.nativePose[i]={QuatOf(M3::FromWorld(local)),{local[12],local[13],local[14]}};
    }
    o.posedForDraw=true;
    o.body->Apply();
}

void RestoreRailPose(Occupant &o) {
    if (!o.posedForDraw) return;
    o.posedForDraw=false;
    if (g_freed.count(o.model) || o.body->UsesAny(g_freed)) return;
    void *const *frames=o.body->Frames();
    for (int i=0;i<20;++i) {
        game::SetLocalRotation(frames[i],o.nativePose[i].rotation);
        game::SetLocalPosition(frames[i],o.nativePose[i].position);
    }
    using Update=void (__stdcall *)(void *);
    reinterpret_cast<Update>((*reinterpret_cast<void ***>(o.model))[0x18/4])(o.model);
}
// Trailing carriages have separate model frames outside the railway actor. Keep
// their visual triangles in node space and refit moved or hidden parts (doors).
struct RailPart {
    std::string name;
    std::vector<V3> triangles;
    V3 origin{};
    M3 axes{};
    bool visible = false;
    bool metroShell = false;
};
struct RailMesh {
    std::vector<RailPart> parts;
    std::vector<V3> triangles;
    std::vector<V3> panels;
    unsigned version = 0;
    ULONGLONG refresh = 0;
};
std::unordered_map<void *, RailMesh> g_carriageMeshes;

template<class Visit> bool RosterContains(uintptr_t vector, Visit visit) {
    auto first = *reinterpret_cast<void ***>(vector), last = *reinterpret_cast<void ***>(vector + 4);
    return first && last >= first && last - first <= 1024 && std::any_of(first, last, visit);
}

void PruneOccupants() {
    std::erase_if(g_occupants, [](const Occupant &o) {
        if (o.wreckModel) return false;
        bool present = false;
        auto check = [&] {
            const uintptr_t vector = o.rail ? reinterpret_cast<uintptr_t>(o.carrier) + 0x254 : game::Exe().trafficElements;
            if (!RosterContains(vector, [&](void *p) { return p == o.record; })) return;
            const int state = o.rail ? game::at<int>(o.record, 0) : 0;
            // Native animation completion moves the body to 18 (passenger) or 20 (driver).
            present = game::at<void *>(o.record, o.slot) == o.model &&
                (o.rail ? (state == 17 || state == 18 || (o.driver && state == 20)) :
                 game::at<uint8_t>(o.record, 0) && game::at<void *>(o.record, 12) == o.carrier &&
                 (!o.driver || game::at<uint8_t>(o.carrier, game::g_exe == &game::kExe10 ? 0x2071 : 0x2131)));
        };
        return !Guarded(check) || !present;
    });
}

void PurgeOccupantFrames(std::span<Ragdoll *const> rebound) {
    std::erase_if(g_occupants, [&](const Occupant &o) {
        return std::find(rebound.begin(), rebound.end(), o.body.get()) == rebound.end() &&
               (g_freed.count(o.model) || g_freed.count(o.frame) || o.body->UsesAny(g_freed));
    });
    for (void *frame : g_freed) g_carriageMeshes.erase(frame);
}

bool ReadRailParts(void *frame, RailMesh &mesh) {
    char file[512]{};
    if (!H->model_file(M, frame, file, sizeof file)) return false;
    int handle = H->model_open(M, file);
    if (!handle) return false;
    struct CloseModel { int handle; ~CloseModel() { H->model_close(M, handle); } } close{handle};
    int count = H->model_nodes(M, handle, nullptr, 0);
    if (count < 1 || count > 4096) return false;
    std::vector<MmlModelNode> nodes(count);
    H->model_nodes(M, handle, nodes.data(), count);
    for (int i = 0; i < count; ++i) {
        const auto &node = nodes[i];
        if (node.frame_type != 1 || (node.visual_type != 0 && node.visual_type != 1)) continue;
        MmlModelMesh geometry{};
        if (!H->model_mesh(M, handle, i, 0, &geometry) || !geometry.pos || !geometry.index) continue;
        RailPart part;
        part.name = node.name;
        part.metroShell = part.name == "metro" && node.parent < 0;
        for (int t = 0; t < geometry.triangles; ++t) {
            const uint16_t *face = geometry.index + 3*t;
            if (std::max({face[0], face[1], face[2]}) >= geometry.vertices) continue;
            for (int j = 0; j < 3; ++j) {
                const float *p = geometry.pos + 3*face[j];
                part.triangles.push_back({p[0], p[1], p[2]});
            }
        }
        if (!part.triangles.empty()) mesh.parts.push_back(std::move(part));
    }
    return !mesh.parts.empty();
}

void RefitRailMesh(RailMesh &mesh) {
    mesh.triangles.clear();
    mesh.panels.clear();
    for (const auto &part : mesh.parts) {
        if (!part.visible) continue;
        float low = 0.f, high = 0.f;
        if (part.metroShell) for (V3 p : part.triangles) {
            const float z = (part.origin + part.axes.Apply(p)).z;
            low = std::min(low, z);
            high = std::max(high, z);
        }
        for (std::size_t i=0; i<part.triangles.size(); i+=3) {
            const V3 a = part.origin + part.axes.Apply(part.triangles[i]);
            const V3 b = part.origin + part.axes.Apply(part.triangles[i+1]);
            const V3 c = part.origin + part.axes.Apply(part.triangles[i+2]);
            const float z = (a.z+b.z+c.z)/3.f;
            const V3 normal = Cross(b-a,c-a);
            // Use the fixed shell's transverse panels. Windows and doors
            // retain the openings and moving parts of the original mesh.
            const bool panel = part.metroShell && (z < low*.78f || z > high*.78f) &&
                               std::abs(normal.z) > .86f*Len(normal);
            auto &out = panel ? mesh.panels : mesh.triangles;
            out.insert(out.end(),{a,b,c});
        }
    }
    ++mesh.version;
}

bool RailMover(const Occupant &o, smoke::Mover &m) {
    if (!o.carriage) {
        // The driver's car is an actor. Share the same posed visual mesh as on-foot ragdolls.
        if (!vehicles::MoverOf(o.carrier, &m)) return false;
    } else {
        const float *w = game::WorldMatrix(o.frame);
        M3 axes = M3::FromWorld(w);
        axes.Orthonormalize();
        const M3 inverse = axes.T();
        m.c = {w[12], w[13], w[14]};
        for (int i=0; i<3; ++i) m.ax[i] = axes.r[i];
        auto &mesh = g_carriageMeshes[o.frame];
        if (mesh.parts.empty() && !ReadRailParts(o.frame, mesh)) return false;
        const auto now = GetTickCount64();
        if (now >= mesh.refresh) {
            bool changed = false;
            for (auto &part : mesh.parts) {
                void *visual = game::FindChild(o.frame, part.name.c_str());
                bool visible = visual != nullptr;
                for (void *f = visual; f && f != o.frame && visible; f = game::at<void *>(f, game::kFrameParent))
                    visible = (game::at<unsigned>(f, game::kFrameFlags) & 1) != 0;
                changed |= visible != part.visible;
                if (visible) {
                    const float *pw = game::WorldMatrix(visual);
                    V3 origin = inverse.Apply(V3{pw[12], pw[13], pw[14]} - m.c);
                    M3 basis = M3::Raw(pw) * inverse;
                    bool moved = !part.visible || Len(origin - part.origin) > .003f;
                    for (int j=0; j<3; ++j) moved |= Len(basis.r[j] - part.axes.r[j]) > .003f;
                    if (moved) { part.origin = origin; part.axes = basis; changed = true; }
                }
                part.visible = visible;
            }
            if (changed) RefitRailMesh(mesh);
            mesh.refresh = now + 80;
        }
        if (mesh.triangles.empty()) return false;
        m.kind = smoke::Mover::HULL;
        m.hull = &mesh.triangles;
        m.cabinPanels = &mesh.panels;
        m.hullIsTriangles = true;
        m.shapeVersion = mesh.version;
    }
    m.key = o.frame;
    m.cabinOnly = true;
    return true;
}

struct TransportShot {
    void *carrier;
    void *model = nullptr;
    bool created = false, hitPerson = false;
};
thread_local TransportShot *g_transportShot;

void StartOccupant(Occupant o,const smoke::Mover *railMover=nullptr) {
    if (!g_cfg.enabled || !o.model || !game::Player() || !jolt_ragdoll::Enabled()) return;
    for (const auto &other : g_occupants) if (other.model == o.model) return;
    smoke::Mover carMover;
    if (!o.rail && !vehicles::MoverOf(o.carrier, &carMover)) return;
    std::string err;
    Ragdoll::PoseSnap live;
    o.body = std::make_unique<Ragdoll>();
    if (!Ragdoll::Snapshot(o.model, nullptr, &live, &err) || !o.body->Init(o.model, o.model, live, &err)) {
        LOG("occupant %p model %p skeleton: %s", o.carrier, o.model, err.c_str());
        return;
    }
    o.body->railOccupant = o.rail;
    if (railMover) {
        // Each carriage follows its own tangent through a curve. The game's
        // speed at +D4 is the scalar used by C_railway::GetActorSpeed.
        const float speed=game::at<float>(o.carrier,0xd4);
        if (std::fabs(speed)<45.f) {
            const V3 velocity=railMover->ax[2]*speed;
            o.body->SetMotionFrom(nullptr,0,&velocity);
        }
    } else {
        const auto track=g_carTracks.find(o.carrier);
        if (track!=g_carTracks.end() && g_tick-track->second.tick<=2)
            o.body->SetMotionFrom(nullptr,0,&track->second.vel);
    }
    if (!jolt_ragdoll::Add(o.body.get(), &err, o.rail ? o.frame : o.carrier)) {
        LOG("occupant %p model %p physics: %s", o.carrier, o.model, err.c_str());
        return;
    }
    WatchRagdoll(o.body.get());
    if (g_transportShot && g_transportShot->model == o.model) g_transportShot->created = true;
    LOG("occupant ragdoll at death: %s %s model %p carrier %p carriage %d body %u", o.rail ? "rail" : "traffic",
        o.driver ? "driver" : "passenger", o.model, o.carrier, o.carriage, o.body->serial);
    g_occupants.push_back(std::move(o));
}

// The transport Hit virtual owns the shot geometry. Its native occupant callback
// identifies the model behind the glass, including on subsequent hits. Keep the
// association within one call so another shot, a blast or an adjacent rider cannot inherit it.
using TransportHitFn = int(__fastcall *)(void *, void *, int, V3 *, V3 *, V3 *, float, void *, int, void *);
TransportHitFn originalCarHit, originalRailHit;
void ApplyTransportShot(const TransportShot &shot, const V3 &from, const V3 &dir, float damage) {
    const V3 delta = Norm(dir) * 100.f; // native vehicle Hit traces 100 m inside the vehicle
    Ragdoll *target = nullptr;
    V3 point{};
    int part = -1;
    float closest = Len(delta);
    for (auto &o : g_occupants) {
        if (o.carrier != shot.carrier || (shot.model && o.model != shot.model)) continue;
        if (!jolt_ragdoll::SeatedCar(o.body.get())) continue;
        V3 candidate{};
        const int segment = jolt_ragdoll::ShotPart(o.body.get(), from, delta, &candidate);
        if (segment < 0) {
            // Native skin and the simplified physical segments can differ slightly
            // on the lethal shot. Only that positively identified new victim qualifies.
            if (!shot.created || o.model != shot.model) continue;
            candidate = o.body->ClosestToLine(from, from + delta);
        }
        const float distance = Len(candidate - from);
        if (distance >= closest) continue;
        if (!shot.created && jolt_ragdoll::CabinBlocksShot(o.rail ? o.frame : o.carrier, from, candidate)) continue;
        target = o.body.get();
        point = candidate;
        part = segment;
        closest = distance;
    }
    if (!target) return;
    const float dv = Kick(target, dir, point, damage, part, g_cfg.shot);
    g_lastBullet = {target->Model(), point, part, part, dv, g_tick};
    ++g_corpseHits;
}

constexpr int kShotExcludeArray=0x2B8, kShotExcludeCount=0x2BC, kShotExcludeMode=0x2B0;
struct ShotExclusions {
    void **original=nullptr;
    int count=0,mode=0;
    bool active=false;
    std::array<void *,256> objects{};
} g_shotExclusions;

void SetShotExclusions(void **objects,int count,int mode) {
    using Set=void (__thiscall *)(void *,void **,int,int);
    reinterpret_cast<Set>(game::Exe().collExclude)(game::Collision(),objects,count,mode);
}

void BeforePhysicsBulletStretch(MmlMidContext *, void *) {
    if (!g_cfg.enabled || !g_cfg.bullets) return;
    auto exclude=[] {
        void *coll=game::Collision();
        auto &saved=g_shotExclusions;
        if (saved.active) return;
        const int count=game::at<int>(coll,kShotExcludeCount);
        auto **original=game::at<void **>(coll,kShotExcludeArray);
        if (count<0 || count>=(int)saved.objects.size() || (count && !original)) return;
        if (count) std::copy_n(original,count,saved.objects.begin());
        const auto total=dogs::AppendShotExclusions(saved.objects.data(),count,saved.objects.size());
        if (total==(unsigned)count) return;
        saved.original=original;
        saved.count=count;
        saved.mode=game::at<int>(coll,kShotExcludeMode);
        SetShotExclusions(saved.objects.data(),(int)total,saved.mode);
        saved.active=true;
    };
    Guarded(exclude);
}

// Compare physical bodies with the first native moving hit and the static
// impact computed when the shot was created.
void PhysicsBulletStretch(MmlMidContext *ctx, void *) {
    if (g_shotExclusions.active) {
        auto &saved=g_shotExclusions;
        saved.active=false;
        SetShotExclusions(saved.original,saved.count,saved.mode);
    }
    if (!g_cfg.enabled || !g_cfg.bullets) return;
    auto trace = [&] {
        void *stack = reinterpret_cast<void *>(ctx->esp);
        auto *gameObject = reinterpret_cast<void *>(ctx->ebp);
        auto *begin = game::at<uint8_t *>(gameObject, game::kGameShots);
        auto *end = game::at<uint8_t *>(gameObject, game::kGameShots + 4);
        const unsigned index = game::at<unsigned>(stack, 0x38);
        if (!begin || end <= begin || index >= static_cast<unsigned>((end - begin) / game::kShotSize)) return;
        auto *shot = begin + index * game::kShotSize;
        const V3 from = game::at<V3>(stack, 0x40), delta = game::at<V3>(stack, 0x18);
        const float length = Len(delta), damage = game::at<float>(shot, game::kShotDamage);
        if (!(length > 0.0001f && damage > 0)) return;
        const V3 axis = delta * (1.f / length);
        float limit = length;
        // collRayTest's third argument is its output hit position (ESP+B0 on
        // return at 5A935C / 5E4CAC). It is valid only for a non-null primitive.
        if (ctx->eax)
            limit = std::min(limit, Dot(game::at<V3>(stack, 0xB0) - from, axis));
        if (game::at<uint8_t>(shot, game::kShotHits))
            limit = std::min(limit, Dot(game::at<V3>(shot, game::kShotHitPos) - from, axis));
        if (!(limit > 0)) return;
        const V3 origin = game::at<V3>(shot, game::kShotFrom);
        Occupant *target = nullptr;
        int part = -1;
        V3 point{};
        for (auto &o : g_occupants) {
            if (!o.rail && !o.wreckModel) continue;
            const V3 hip = o.body->BodyPos(Ragdoll::B_PELVIS);
            const V3 nearest = from + axis * std::clamp(Dot(hip - from, axis), 0.f, limit);
            if (Len(hip - nearest) > 2.0f) continue;
            V3 contact{};
            const int segment = jolt_ragdoll::ShotPart(o.body.get(), from, axis * limit, &contact);
            if (segment < 0 || jolt_ragdoll::CabinBlocksShot(o.rail ? o.frame : o.carrier, origin, contact,
                    jolt_ragdoll::SeatedCar(o.body.get()) != nullptr)) continue;
            target = &o;
            limit = Len(contact - from);
            point = contact;
            part = segment;
        }
        const auto humanInFront = [&](V3 point) {
            if (Dot(point-from,axis)<=0.f) return false;
            for (const auto &r:g_ragdolls) {
                if (Len(r->Center()-point)>2.f) continue;
                V3 contact{};
                if (jolt_ragdoll::ShotPart(r.get(),from,point-from,&contact)>=0) return true;
            }
            return false;
        };
        dogs::ShotHit dogHit{};
        const bool hitDog = dogs::TraceShot(from,axis,limit,dogHit) && !humanInFront(dogHit.point);
        if (hitDog) limit=Len(dogHit.point-from);
        V3 hatPoint{};
        uint32_t hatBody=0;
        const bool hatHit=headwear::Shot(from,axis,limit,&hatPoint,&hatBody) && !humanInFront(hatPoint);
        if (hatHit) {
            headwear::BulletImpulse(hatBody,axis,hatPoint,damage);
            point=hatPoint;
        } else if (hitDog) {
            dogs::HitBody(dogHit,axis,Momentum(damage,g_cfg.shot),damage,
                          game::at<void *>(shot,game::kShotShooter));
            point=dogHit.point;
        } else if (target) {
            const float dv = Kick(target->body.get(), axis, point, damage, part, g_cfg.shot);
            g_lastBullet = {target->model, point, part, part, dv, g_tick};
            ++g_corpseHits;
        } else {
            return;
        }
        // Finish this shot at the resolved contact using TickShoot's ordinary end-of-range
        // path. Both builds cache the endpoint and squared range on the stack.
        // A wall or another person farther down the ray must not receive it too.
        const V3 way = point - origin;
        game::at<V3>(shot, game::kShotWay) = way;
        game::at<V3>(shot, game::kShotNow) = point;
        game::at<void *>(shot, game::kShotPrim) = nullptr;
        game::at<uint8_t>(shot, game::kShotHits) = 0;
        game::at<V3>(stack, 0xBC) = point;
        game::at<float>(stack, 0x14C) = Dot(way, way);
        ctx->eax = 0;
    };
    Guarded(trace);
}

int TransportHit(TransportHitFn original, void *carrier, void *edx, int type, V3 *dir, V3 *from, V3 *pos,
                 float damage, void *attacker, int flags, void *frame) {
    TransportShot shot{carrier};
    auto previous = g_transportShot;
    g_transportShot = type == 0 && dir && from && damage > 0 && g_cfg.enabled && g_cfg.bullets ? &shot : nullptr;
    V3 start{}, direction{};
    if (g_transportShot) { start = *from; direction = *dir; }
    int result;
    __try {
        result = original(carrier, edx, type, dir, from, pos, damage, attacker, flags, frame);
    } __finally {
        g_transportShot = previous;
    }
    if (!shot.hitPerson && Len(direction) > 1e-5f && !g_occupants.empty())
        ApplyTransportShot(shot, start, direction, damage);
    return result;
}

int __fastcall CarHit(void *car, void *edx, int type, V3 *dir, V3 *from, V3 *pos, float damage, void *attacker, int flags, void *frame) {
    return TransportHit(originalCarHit, car, edx, type, dir, from, pos, damage, attacker, flags, frame);
}
int __fastcall RailwayHit(void *rail, void *edx, int type, V3 *dir, V3 *from, V3 *pos, float damage, void *attacker, int flags, void *frame) {
    return TransportHit(originalRailHit, rail, edx, type, dir, from, pos, damage, attacker, flags, frame);
}

void TrafficDeath(MmlMidContext *ctx, void *) {
    auto capture = [&] {
        if (!g_cfg.enabled) return;
        void *car = reinterpret_cast<void *>(ctx->ecx);
        RosterContains(game::Exe().trafficElements, [&](void *record) {
            if (!record || !game::at<uint8_t>(record, 0) || game::at<void *>(record, 12) != car) return false;
            for (int slot : {28, 32}) {
                void *model = game::at<void *>(record, slot);
                if (!model || model != reinterpret_cast<void *>(ctx->edx)) continue;
                if (g_transportShot && g_transportShot->carrier == car) g_transportShot->model = model;
                Occupant o;
                o.carrier = car; o.record = record; o.model = model; o.slot = slot;
                o.driver = slot == 28;
                StartOccupant(std::move(o));
            }
            return true;
        });
    };
    Guarded(capture);
}

void RailDeath(MmlMidContext *ctx, void *) {
    auto capture = [&] {
        if (!g_cfg.enabled) return;
        Occupant o;
        o.record = reinterpret_cast<void *>(ctx->ecx);
        o.carrier = reinterpret_cast<void **>(ctx->esp)[1];
        if (!o.carrier || !o.record) return;
        int state = game::at<int>(o.record, 0);
        if (state == 22) state = game::at<int>(o.record, 0x44);
        o.rail = true; o.slot = 4;
        o.model = game::at<void *>(o.record, o.slot);
        if (g_transportShot && g_transportShot->carrier == o.carrier) g_transportShot->model = o.model;
        if (state == 17) return;
        o.driver = o.record == static_cast<uint8_t *>(o.carrier) + 0x2b0;

        o.carriage = game::at<int>(o.record, 0x18);
        Cabin cabin{};
        if (!o.model || !cabin.Open(o.carrier, o.carriage)) return;
        o.frame = cabin.frame;
        Watch(o.frame);
        smoke::Mover mover;
        if (!RailMover(o, mover)) return;
        StartOccupant(std::move(o),&mover);
    };
    Guarded(capture);
}

void OccupantMovers(std::vector<smoke::Mover> &out) {
    for (const auto &o : g_occupants) {
        if (!jolt_ragdoll::SeatedCar(o.body.get())) continue;
        void *key = o.rail ? o.frame : o.carrier;
        if (std::any_of(out.begin(), out.end(), [&](const auto &m) { return m.key == key; })) continue;
        smoke::Mover m;
        if (o.rail ? !RailMover(o, m) : !vehicles::MoverOf(o.carrier, &m)) continue;
        out.push_back(m);
    }
}

// In Do_ThrowCocotFromCar, immediately after the virtual frame duplicate:
// ESI is Corovacka's driver model, EDI its clone and EBP the exact car.
void TrafficDriverCloned(MmlMidContext *ctx, void *) {
    for (auto &o : g_occupants) {
        if (o.rail || !o.driver || o.model != reinterpret_cast<void *>(ctx->esi) ||
            o.carrier != reinterpret_cast<void *>(ctx->ebp)) continue;
        o.clone = reinterpret_cast<void *>(ctx->edi);
        o.cloneTick = g_tick;
        break;
    }
}

bool TakeClonedDriver(void *human) {
    void *model = ModelOf(human);
    for (auto it = g_occupants.begin(); it != g_occupants.end(); ++it) {
        if (!it->clone || it->clone != model || it->cloneTick != g_tick || it->carrier != SeatCarOf(human)) continue;
        Ragdoll::PoseSnap live;
        std::string err;
        if (!Ragdoll::Snapshot(model, nullptr, &live, &err) || !it->body->RebindModel(model, live)) return false;
        auto body = std::move(it->body);
        LOG("traffic carjack: model %p cloned to %p, car %p, actor %p keeps body %u at age %.3f",
            it->model, model, it->carrier, human, body->serial, body->age);
        g_occupants.erase(it);
        body->actor = human;
        body->keyVtable = game::at<uint32_t>(human, 0);
        WatchRagdoll(body.get());
        body->Apply();
        g_ragdolls.push_back(std::move(body));
        return true;
    }
    return false;
}

// ---- making a ragdoll
Ragdoll *Start(void *actor, bool puppet, std::string *err, const void *struckBy = nullptr) {
    if (Ragdoll *r = Find(actor)) return r;
    // Traffic can promote the same model to a human when Tommy opens the door.
    // Keep its Jolt bodies, their velocities and their existing seat constraint.
    if (!puppet && !IsPed(actor)) {
        const void *model = ModelOf(actor);
        auto occupant = std::find_if(g_occupants.begin(), g_occupants.end(), [&](const Occupant &o) {
            return !o.rail && o.model == model;
        });
        if (occupant != g_occupants.end()) {
            auto body = std::move(occupant->body);
            g_occupants.erase(occupant);
            body->actor = actor;
            body->keyVtable = game::at<uint32_t>(actor, 0);
            g_ragdolls.push_back(std::move(body));
            return g_ragdolls.back().get();
        }
    }
    // The host must stop its previous handover before a new body captures/poses the skeleton.
    if (MML_HAS(H, human_get_up_cancel) && H->human_get_up_cancel) H->human_get_up_cancel(M, actor);
    void *leftTram = LeaveTram(actor);
    auto r = std::make_unique<Ragdoll>();
    r->puppet = puppet;
    r->vehicleImpact = struckBy != nullptr;
    r->impactVehicle = const_cast<void *>(struckBy);
    float dt = 0;
    const V3 *prev = nullptr;
    const Ragdoll::PoseSnap *startPose = nullptr;
    bool ped = IsPed(actor);
    V3 pedestrianVelocity{};
    CarHandover *handover = ped ? PendingHandover(actor) : nullptr;
    Ragdoll::PoseSnap currentPose{};
    bool preHitPose = false;
    const auto hist = g_poses.find(actor);
    V3 dir = ped ? V3{} : game::at<V3>(actor, game::kActorFacing);
    bool inited = false;
    auto init = [&] {
        void *model = ModelOf(actor);
        if (!Ragdoll::Snapshot(model, ped ? nullptr : &dir, &currentPose, err)) return;
        // Native model changes can keep the root while replacing its children and moving it to another parent.
        // Only a continuous binding can supply the pre-reaction pose or any part of its velocity history.
        const bool continuous = hist != g_poses.end() && hist->second.count > 0 &&
                                (!ped || hist->second.pedestrianModel == model) &&
                                currentPose.SameBinding(hist->second.snap);
        const bool recent = continuous && g_tick - hist->second.seen <= 1;
        if (recent) {
            startPose = &hist->second.snap;
            if (hist->second.count > 1) { prev = hist->second.pts[1]; dt = hist->second.dt[0]; }
            if (ped) pedestrianVelocity = hist->second.pedestrianVelocity;
        } else if (!ped && continuous) {
            prev = PreviousPose(hist->second, currentPose.pts, &dt);
        }
        if (handover && handover->captured && handover->model == model) {
            // Capture before native collision reaction, after this tick's dodge animation advanced.
            preHitPose = currentPose.SameBinding(handover->beforePose);
            startPose = preHitPose ? &handover->beforePose : nullptr;
            if (preHitPose && recent && hist->second.pedestrianAnimation == handover->animation) {
                prev = hist->second.pts[0];
                dt = g_lastDt;
            } else {
                prev = nullptr;
                dt = 0;
                pedestrianVelocity = {};
            }
        }
        inited = r->Init(actor, model, currentPose, err, startPose);
    };
    if (!Guarded(init) || !inited) {
        if (handover) handover->startResult = "skeleton_init_failed";
        LOG("ragdoll init failed on %p: %s", actor, err->c_str());
        BackIntoTram(actor, leftTram);
        return nullptr;
    }
    r->pedestrian = ped;
    r->SetMotionFrom(prev, dt, ped ? &pedestrianVelocity : nullptr);
    r->keyVtable = game::at<uint32_t>(actor, 0);
    V3 pv = r->BodyVel(Ragdoll::B_PELVIS);
    LOG("ragdoll started on %s %p (puppet=%d, pose from %s, motion from %s, dt %.3f; pelvis %.2f %.2f %.2f m/s)", ped ? "pedestrian" : "person",
        actor, puppet, preHitPose ? "before car hit" : startPose ? "last tick" : "now",
        prev ? "history" : "rest", dt, pv.x, pv.y, pv.z);
    if (ped && prev && dt > 1e-4f) {
        const V3 visual = (startPose->pts[Ragdoll::PELVIS] - prev[Ragdoll::PELVIS]) * (1.0f / dt);
        if (Len(visual - pedestrianVelocity) > 8.0f)
            LOG("pedestrian %p motion seed (animation %d): native %.2f %.2f %.2f, visual pelvis %.2f %.2f %.2f m/s",
                actor, hist->second.pedestrianAnimation, pedestrianVelocity.x, pedestrianVelocity.y, pedestrianVelocity.z,
                visual.x, visual.y, visual.z);
    }
    g_ragdolls.push_back(std::move(r));
    Ragdoll *made = g_ragdolls.back().get();
    void *car = ped ? nullptr : SeatCarOf(actor);
    if (car) LOG("ragdoll %p dies sitting in car %p: its body stays in the car's cabin, held to the seat", actor, car);
    RememberWreckInterior(car, made->Model(), currentPose.pts);
    std::string jerr;
    if (!jolt_ragdoll::Add(made, &jerr, car, struckBy)) {
        if (handover) handover->startResult = "physics_body_failed";
        LOG("ragdoll %p: Jolt could not make its body (%s): left to the game's animation; %s", actor, jerr.c_str(), jolt_world::Describe().c_str());
        g_ragdolls.pop_back();
        *err = "Jolt could not make the body: " + jerr;
        BackIntoTram(actor, leftTram);
        return nullptr;
    }
    WatchRagdoll(made);
    if (auto *entry = PendingHandover(actor); entry && entry->model == made->Model()) {
        entry->started = true;
        entry->startResult = "started";
        entry->body = made;
        entry->history = prev != nullptr;
        entry->preHitPose = preHitPose;
        entry->initial = made->InitialPoints()[Ragdoll::PELVIS];
        entry->bodyStart = made->BodyPos(Ragdoll::B_PELVIS);
        if (entry->captured) {
            for (int i = 0; i < Ragdoll::COUNT; ++i)
                entry->poseGap = std::max(entry->poseGap, Len(made->InitialPoints()[i] - entry->beforePose.pts[i]));
            if (entry->poseGap > .3f)
                LOG("pedestrian %p car handover pose gap %.3f m (history %d)", actor, entry->poseGap, entry->history);
        }
    }
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
    auto addVehicle = [&](void *car, const Ragdoll &body) {
        if (!car || std::any_of(out.begin(), out.end(), [&](const smoke::Mover &m) { return m.key == car; })) return;
        V3 at, axes[3];
        if (!vehicles::Pose(car, &at, axes) || Len(at - body.BodyPos(Ragdoll::B_PELVIS)) > 40.0f) return;
        smoke::Mover mover;
        if (vehicles::MoverOf(car, &mover)) out.push_back(mover);
    };
    for (const auto &body : g_ragdolls) {
        addVehicle(body->impactVehicle, *body);
        addVehicle(const_cast<void *>(jolt_ragdoll::SeatedCar(body.get())), *body);
    }
    std::vector<V3> bodies;
    for (auto &r : g_ragdolls) bodies.push_back(r->BodyPos(Ragdoll::B_PELVIS));
    for (auto &kv : g_poses) {
        if (kv.second.count == 0 || g_tick - kv.second.seen > 1) continue;
        if (Find(kv.first)) continue;   // (a fresh ragdoll's pose as capsules would stop it dead)
        V3 p = kv.second.snap.pts[Ragdoll::PELVIS];
        bool close = false;
        for (const V3 &c : bodies) close = close || Len(c - p) < 6.0f;
        if (!close) close=headwear::Near(p) || dogs::Near(p);
        // Also give living bodies near a moving car as sensors. Native car/pedestrian contact can miss a waving
        // animation until it is already inside the car. These use the actual car hull and anatomical capsules.
        if (!close && IsPed(kv.first)) {
            for (const smoke::Mover &m : out) {
                if (m.kind == smoke::Mover::CAPSULE) break; // vehicle movers precede living capsules
                if (Dot(m.c - p, m.c - p) >= 81.0f) continue;
                auto track = g_carTracks.find(const_cast<void *>(m.key));
                if (track != g_carTracks.end() && g_tick - track->second.tick <= 2 && Len(track->second.vel) > 1.5f) {
                    close = true;
                    break;
                }
            }
        }
        if (!close) continue;
        Ragdoll pose;
        pose.Setup(kv.second.snap.pts, kv.second.snap.fwd, kv.second.snap.rot);
        for (int b = 0; b < Ragdoll::BodyCount(); b++) {
            smoke::Mover m;
            m.key = static_cast<const uint8_t *>(kv.first) + b;
            m.kind = smoke::Mover::CAPSULE;
            pose.BodyCapsule(b, &m.p0, &m.p1, &m.r);
            m.owner = kv.first;
            m.partMass = Ragdoll::SegmentMass[b];
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
    V3 pp = pl ? game::ActorWorldPos(pl) : V3{};
    if (pl) dogs::PrepareStep(dt,pp);
    static std::vector<smoke::Mover> movers;
    JoltMovers(movers);
    OccupantMovers(movers);
    for (auto &mover : movers) {
        const auto interior = g_wreckInteriors.find(mover.key);
        if (interior != g_wreckInteriors.end()) mover.wreckInterior = &interior->second;
    }
    headwear::ApplyAirDrag(dt);
    jolt_ragdoll::Step(jolt, dt, movers, pl ? &pp : nullptr);
    SampleRailway(dt);
    dogs::FinishStep();
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
void SampleHandoverPose(CarHandover *entry, Ragdoll *r, unsigned stage) {
    if (!entry || entry->poseCount >= entry->poses.size()) return;
    Ragdoll::PoseSnap snap{};
    if (!r->SnapshotNow(&snap)) return;
    M3 rotations[19];
    V3 origins[19];
    r->PhysicalPose(rotations, origins);
    HandoverPose sample;
    sample.tick = g_tick;
    sample.stage = stage;
    sample.age = r->age;
    sample.physical = origins[18];
    sample.drawn = snap.org[18];
    sample.element = game::at<V3>(r->actor, r->pedestrian ? kPedPos : game::kActorPos);
    for (int i = 0; i < 19; ++i) sample.error = std::max(sample.error, Len(snap.org[i] - origins[i]));
    for (int i = 0; i < Ragdoll::BodyCount(); ++i) sample.fastest = std::max(sample.fastest, Len(r->BodyVel(i)));
    entry->poses[entry->poseCount++] = sample;
    if (stage == 4) entry->maxDrawError = std::max(entry->maxDrawError, sample.error);
}
bool SafeApply(Ragdoll *r, bool rendering = false) {
    CarHandover *entry = nullptr;
    const unsigned phase = rendering ? 1 : 0;
    for (unsigned i = 0; i < std::min<unsigned>(g_handoverCount, g_handovers.size()); ++i) {
        auto &candidate = g_handovers[(g_handoverCount - 1 - i) % g_handovers.size()];
        if (candidate.body != r || candidate.actor != r->actor || candidate.model != r->Model()) continue;
        if (r->age > 3.5f || candidate.poseCount + 2 > candidate.poses.size() ||
            (candidate.sampled[phase] && candidate.lastPoseTick[phase] == g_tick)) break;
        if (candidate.poseCount < 48 || r->age >= candidate.nextPoseAge[phase]) entry = &candidate;
        break;
    }
    if (entry) {
        entry->sampled[phase] = true;
        entry->lastPoseTick[phase] = g_tick;
        entry->nextPoseAge[phase] = r->age + .10f;
        auto sample = [&] { SampleHandoverPose(entry, r, rendering ? 3 : 1); };
        Guarded(sample);   // diagnostics must not prevent the ordinary pose write
    }
    auto f = [&] { r->Apply(); };
    const bool result = Guarded(f);
    if (result && entry) {
        auto sample = [&] { SampleHandoverPose(entry, r, rendering ? 4 : 2); };
        Guarded(sample);
    }
    return result;
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
        if (h != pl && Dot(d, d) > r2) continue;
        if (alive) g_failed.erase(h);
        // every dead body near the player is a ragdoll, however it died: a corpse is never left frozen in its last pose
        if (!alive && g_cfg.enabled && !Find(h) && !g_failed.count(h) && (ped || !BeingPulledFromCar(h))) {
            std::string err;
            if (Start(h, false, &err)) LOG("dead body %p adopted", h);
            else { LOG("dead body %p not adopted: %s", h, err.c_str()); g_failed.insert(h); }
        }
        if (Find(h)) continue;
        Ragdoll::PoseSnap snap;
        if (!SafeSnapshot(h, &snap)) continue;
        void *model = ModelOf(h);
        PoseHistory &ph = g_poses[h];
        if (ph.count && !snap.SameBinding(ph.snap)) ph.count = 0;
        if (ped) {
            const V3 position = game::at<V3>(h, kPedPos);
            const int animation = game::at<int>(h, kPedAnimation);
            const bool continuous = ph.count && g_tick - ph.seen == 1 && ph.pedestrianModel == model &&
                                    animation == ph.pedestrianAnimation && dt > 1e-4f;
            const V3 velocity = continuous ? (position - ph.pedestrianPos) * (1.0f / dt) : V3{};
            // The element position follows native movement. A clip switch, recycled element or teleport is
            // not a velocity sample. The rendered skeleton may advance even while this position stays still.
            const bool valid = continuous && Len(velocity) <= 12.0f;
            ph.pedestrianVelocity = valid ? velocity : V3{};
            ph.pedestrianPos = position;
            ph.pedestrianAnimation = animation;
            ph.pedestrianModel = model;
            if (!valid) ph.count = 0;
        }
        memcpy(ph.pts[1], ph.pts[0], sizeof ph.pts[0]);
        memcpy(ph.pts[0], snap.pts, sizeof snap.pts);
        ph.snap = snap;
        ph.dt[1] = ph.dt[0];
        ph.dt[0] = dt;
        ph.count = std::min(ph.count + 1, 2);
        ph.seen = g_tick;
        Watch(model);
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
void GetUp(void *actor, const Ragdoll *rd = nullptr) {
    int result = 0;
    if (rd && MML_HAS(H, human_get_up_pose) && H->human_get_up_pose) {
        M3 rot[19];
        V3 org[19];
        rd->PhysicalPose(rot, org);
        // The physics uses its original frame order; the SDK uses a parent-first named bone order.
        const int from[MML_BONE_COUNT] = {18,6,7,8,9,10,11,12,13,14,15,16,17,0,1,2,3,4,5,19};
        MmlBone pose[MML_BONE_COUNT]{};
        if (H->human_skeleton(M, actor, pose, MML_BONE_COUNT) != MML_BONE_COUNT) {
            LOG("%p could not get up: no current skeleton", actor);
            return;
        }
        M3 rootRotation = M3::FromWorld(pose[MML_BONE_ROOT].world);
        float low = rd->BodyPos(Ragdoll::B_PELVIS).y;
        for (int i = 0; i < Ragdoll::BodyCount(); i++) {
            V3 a, e;
            float radius;
            rd->BodyCapsule(i, &a, &e, &radius);
            low = std::min(low, std::min(a.y, e.y) - radius);
        }
        for (int i = 0; i < MML_BONE_COUNT; i++) {
            int j = from[i];
            M3 r = j < 19 ? rot[j] : rootRotation;
            V3 p = j < 19 ? org[j] : V3{org[18].x, low, org[18].z};
            const float world[16] = {r.r[0].x,r.r[0].y,r.r[0].z,0,r.r[1].x,r.r[1].y,r.r[1].z,0,
                                     r.r[2].x,r.r[2].y,r.r[2].z,0,p.x,p.y,p.z,1};
            memcpy(pose[i].world, world, sizeof world);
        }
        result = H->human_get_up_pose(M, actor, pose, MML_BONE_COUNT, 600);
    } else {
        result = H->human_get_up(M, actor, 600);
    }
    if (result) {
        LOG("%p gets up", actor);
        if (actor == H->human_player(M)) g_playerRisingUntil = GetTickCount() + 5000;
        if (rd && !rd->pedestrian) EnterTramUnder(actor, rd->BodyPos(Ragdoll::B_PELVIS));
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

const char *GetUpReason(const Ragdoll &body) {
    if (!body.knockdown || body.age <= 2.5f) return nullptr;
    if (body.sleeping) return "asleep";
    if (body.sleepTimer >= 0.30f) {
        const V3 p = body.BodyPos(Ragdoll::B_PELVIS);
        const float from[3] = {p.x, p.y + .15f, p.z}, to[3] = {p.x, p.y - .60f, p.z};
        MmlHit ground{};
        if (H->world_ray(M, from, to, MML_WORLD_ALL | MML_WORLD_NO_PEOPLE, body.actor, &ground) && ground.normal[1] > .5f)
            return "settled on support";
    }
    return body.age > 10.0f ? "timeout" : nullptr;
}

void TickBody(float dt) {
    g_tick++;
    g_wreckSearches.clear();
    Purge();
    PruneOccupants();
    ListPeople();
    headwear::Tick(g_pedList);
    TracePedestrians(TrajectoryStage::TickBegin);
    static std::vector<void *> railwayCabins;
    railwayCabins.clear();
    for (const auto &o : g_occupants)
        if (o.rail) railwayCabins.push_back(o.frame);
    scenery::Tick(railwayCabins);
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
        if (rd->puppet && IsDead(rd->actor)) { rd->knockdown = false; rd->puppet = false; }
        if (const char *reason = GetUpReason(*rd)) {
            void *actor = rd->actor;

            LOG("%p recovery after %.3f s: %s (quiet %.3f s)", actor, rd->age, reason, rd->sleepTimer);
            GetUp(actor, rd);   // copy the current physics before deleting it
            EraseAt(i);
            continue;
        }
        if (!rd->pedestrian && IsDead(rd->actor) && (rd->carExtraction || BeingPulledFromCar(rd->actor))) {
            void *player = game::Player();
            const int clip = player ? game::at<int>(player, kHumanPlaying) : 0;
            const bool pulling = clip == 306 || clip == 307;
            if (pulling && !rd->carExtraction) {
                if (const void *car = jolt_ragdoll::SeatedCar(rd)) BeginPullTrace(rd, car);
            }
            rd->carExtraction = pulling;
            V3 pose[Ragdoll::COUNT];
            if (!rd->carExtraction) jolt_ragdoll::FinishPull(rd);
            else if (SafeCapture(player, pose)) {
                if (g_pull.body == rd) {
                    g_pull.left = pose[Ragdoll::LHAND];
                    g_pull.right = pose[Ragdoll::RHAND];
                    g_pull.hands = true;
                }
                const V3 hands = (pose[Ragdoll::RHAND] + pose[Ragdoll::LHAND]) * .5f;
                if (FinitePoint(hands) && FinitePoint(pose[Ragdoll::PELVIS]))
                    jolt_ragdoll::PullFromSeat(rd, player, pose, dt);
            }
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
    for (const auto &o : g_occupants) jolt.push_back(o.body.get());
    bool joltOk = SafeJoltStep(jolt, dt);
    if (!joltOk) {
        g_occupants.clear();
        headwear::Reset();
        dogs::Clear();
        LOG("jolt world faulted: its ragdolls dropped, the world made anew on the next tick");
        jolt_ragdoll::Clear();
        jolt_world::Stop();
        for (size_t i = 0; i < g_ragdolls.size();) {
            if (std::find(jolt.begin(), jolt.end(), g_ragdolls[i].get()) == jolt.end()) { i++; continue; }
            EraseAt(i);
        }
    }
    if (joltOk) {
        headwear::Sync();
        SamplePull(dt);
        TracePedestrians(TrajectoryStage::Physics);
        for (auto &entry : g_handovers) {
            if (entry.started && entry.body && entry.body->age <= 3.5f) {
                entry.worldContactsDropped += jolt_world::CarContactsDropped();
                for (const auto &c : jolt_world::CarContacts()) {
                    if (c.owner != (uintptr_t)entry.body) continue;
                    if (entry.contactCount == entry.contacts.size()) { ++entry.contactsDropped; continue; }
                    entry.contacts[entry.contactCount++] = {g_tick, entry.body->age, c};
                }
            }
            if (!entry.started || entry.stepped) continue;
            Ragdoll *body = Find(entry.actor);
            if (!body || body != entry.body || std::find(jolt.begin(), jolt.end(), body) == jolt.end()) continue;
            entry.firstStep = body->BodyPos(Ragdoll::B_PELVIS);
            entry.stepped = true;
            const float moved = Len(entry.firstStep - entry.bodyStart);
            if (moved > .5f)
                LOG("pedestrian %p first physics step moved %.3f m (dt %.4f s)", entry.actor, moved, dt);
        }
    }
    if (joltOk && g_cfg.enabled && g_cfg.carFeel) SafeCarReactions();
    if (joltOk && g_cfg.enabled && g_cfg.carKnockdown && MML_HAS(H, human_vehicle_contact) && H->human_vehicle_contact) {
        // Copy first: a native hit may create a body, changing the physics world's registration containers.
        auto contacts = jolt_world::VehicleContacts();
        for (const auto &c : contacts) {
            void *who = const_cast<void *>(c.who);
            // This fallback repairs the traffic manager's missed sweep. Mission actors
            // (including cinematic doubles) receive vehicle hits through their own collision handler.
            if (!IsPed(who) || Find(who) || IsDead(who)) continue;
            // Native knockdown thresholds: pedestrian strength >16 (pedCarHit uses 4.2 * speed), C_human >3 m/s.
            // Minor contacts remain the native game's; repeatedly synthesizing them would accumulate damage.
            if (Len(c.velocity) <= 16.0f / 4.2f) continue;
            const float velocity[3] = {c.velocity.x,c.velocity.y,c.velocity.z}, at[3] = {c.at.x,c.at.y,c.at.z};
            if (H->human_vehicle_contact(M, who, const_cast<void *>(c.car), velocity, at))
                LOG("vehicle contact %p -> %p (%.2f m/s): native car hit after the physics step", c.car, who, c.closing);
        }
    }
    if (joltOk) KnockByBodies(dt);
    for (size_t i = 0; i < g_ragdolls.size();) {
        Ragdoll *rd = g_ragdolls[i].get();
        const bool headwearActive = joltOk && jolt_ragdoll::Has(rd);
        if (AtRest(rd, SafeOverwritten(rd))) {
            if (headwearActive) headwear::Advance(*rd,dt);
            i++;
            continue;
        }
        if (SafeApply(rd)) {
            if (headwearActive) headwear::Advance(*rd,dt);
            // the game's idea of where the body is: the actor's position or the element's, in the model's parent's space
            game::at<V3>(rd->actor, rd->pedestrian ? kPedPos : game::kActorPos) = game::ToParentPoint(rd->Model(), rd->rootPos);
            i++;
        } else {
            LOG("ragdoll %p faulted while writing, dropped", rd->actor);
            EraseAt(i);
        }
    }
    for (auto &o : g_occupants) {
        auto write = [&] {
            if (o.rail) ApplyRailPose(o);
            else o.body->Apply();
            if (joltOk) headwear::Advance(*o.body,dt);
        };
        Guarded(write);
        if (o.rail) {
            auto restore=[&] { RestoreRailPose(o); };
            Guarded(restore);
        }
    }
    headwear::FinishStep();
    TracePedestrians(TrajectoryStage::TickEnd);
    GiveShapes();
    TrackPoses(dt);
    SafeTrackCars(dt);
    V3 body;
    ragcam::Follow(PlayerBodyPoint(&body) ? &body : nullptr);
    g_lastDt = dt;
}

// ---- pedestrian dodge: keep the native car sweep and hit aligned with the current model. The movement
// sweep inside the pedestrian's own tick also needs the previously accepted root, as it otherwise tests
// the entire dodge from its clip origin again and can send the pedestrian back to a car behind him.
using PedTickFn = void(__fastcall *)(void *, void *, int);
using PedStepSweepFn = void *(__stdcall *)(const V3 *, const V3 *, float, V3 *, V3 *, int, int);
PedTickFn oPedTick;
PedStepSweepFn oPedStepSweep;
unsigned g_dodgeStepSweeps;

struct PedTickScope {
    void *element, *model;
    V3 accepted;
};
thread_local const PedTickScope *g_pedTick;

uintptr_t PedTickAddress() {
    // pedStepReturn1 is at the same offset in C_traffic_element::Tick in both builds.
    return game::Exe().pedStepReturn1 ? game::Exe().pedStepReturn1 - 0x949 : 0;
}

bool IsDodge(int animation) { return animation >= 0x142 && animation <= 0x149; }

void __fastcall hkPedTick(void *element, void *edx, int dt) {
    const auto *parent = g_pedTick;
    const PedTickScope step{element, game::at<void *>(element, kPedModel), game::at<V3>(element, kPedPos)};
    g_pedTick = &step;
    __try { oPedTick(element, edx, dt); }
    __finally { g_pedTick = parent; }
}

void *__stdcall hkPedStepSweep(const V3 *start, const V3 *delta, float radius, V3 *at, V3 *normal, int moving, int stat) {
    const auto &x = game::Exe();
    const uintptr_t caller = reinterpret_cast<uintptr_t>(_ReturnAddress());
    const PedTickScope *step = g_pedTick;
    if (!step || (caller != x.pedStepReturn1 && caller != x.pedStepReturn2) ||
        !IsDodge(game::at<int>(step->element, kPedAnimation)) ||
        step->model != game::at<void *>(step->element, kPedModel) ||
        !g_cfg.enabled || !g_cfg.carKnockdown)
        return oPedStepSweep(start, delta, radius, at, normal, moving, stat);

    // The engine's destination and collision response stay native. Move the start and shorten the delta
    // by the same amount. TestSphereB returns a world-space stopping point and the caller feeds its
    // difference from the proposed element position back into the animation's track offset.
    const V3 offset = step->accepted - game::at<V3>(step->element, 0x34);
    if (!FinitePoint(offset)) return oPedStepSweep(start, delta, radius, at, normal, moving, stat);
    const V3 previous = *start + offset, remaining = *delta - offset;
    ++g_dodgeStepSweeps;
    if (Dot(remaining, remaining) < 1e-10f) return nullptr;
    return oPedStepSweep(&previous, &remaining, radius, at, normal, moving, stat);
}

using PedCarSweepFn = char(__fastcall *)(void *, void *, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t,
                                         uint32_t, uint32_t, uint32_t);
using PedCarHitFn = char(__fastcall *)(void *, void *, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, void *);
PedCarSweepFn oPedCarSweep;
PedCarHitFn oPedCarHit;
unsigned g_dodgeFixes;
struct PedSweepScope {
    std::vector<PedestrianPositionSwap> *swaps;
    PedSweepScope *parent;
};
PedSweepScope *g_pedSweep;

void RestorePedPositions(const std::vector<PedestrianPositionSwap> &swaps) {
    for (const auto &swap : swaps) {
        auto restore = [&] { swap.Restore(game::at<void *>(swap.element, kPedModel),
                                          game::at<V3>(swap.element, kPedPos)); };
        Guarded(restore);
    }
}

bool CommitPedPosition(uint8_t *element, void *model, V3 *before) {
    bool committed = false;
    for (auto *scope = g_pedSweep; scope; scope = scope->parent) {
        for (auto &swap : *scope->swaps) {
            if (swap.element != element || swap.model != model) continue;
            *before = swap.before;
            swap.committed = true;
            committed = true;
        }
    }
    return committed;
}

char RunPedCarSweep(void *self, void *edx, const uint32_t *args,
                    std::vector<PedestrianPositionSwap> &swaps) {
    PedSweepScope scope{&swaps, g_pedSweep};
    g_pedSweep = &scope;
    __try {
        return oPedCarSweep(self, edx, args[0], args[1], args[2], args[3], args[4],
                           args[5], args[6], args[7], args[8], args[9]);
    } __finally {
        g_pedSweep = scope.parent;
        RestorePedPositions(swaps);
    }
}

bool ModelPosition(uint8_t *e, V3 *out) {
    void *model = game::at<void *>(e, kPedModel);
    if (!model) return false;
    const float *m = game::WorldMatrix(model);
    if (!m) return false;
    *out = {m[12], m[13], m[14]};
    return true;
}
void SwapToModels(std::vector<PedestrianPositionSwap> &swaps) {
    for (const MmlPedestrian &p : g_pedList) {
        uint8_t *e = static_cast<uint8_t *>(p.element);
        V3 at;
        if (!e[kPedActive] || !ModelPosition(e, &at)) continue;
        V3 &pos = game::at<V3>(e, kPedPos);
        if (Len(at - pos) < 0.05f) continue;
        swaps.push_back({e, game::at<void *>(e, kPedModel), pos, at});
        pos = at;
    }
}
char __fastcall hkPedCarSweep(void *self, void *edx, uint32_t a1, uint32_t a2, uint32_t a3, uint32_t a4, uint32_t a5, uint32_t a6,
                              uint32_t a7, uint32_t a8, uint32_t a9, uint32_t a10) {
    if (!g_cfg.enabled || !g_cfg.carKnockdown || H->mission_state() == MML_MISSION_LOADING)
        return oPedCarSweep(self, edx, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10);
    std::vector<PedestrianPositionSwap> swaps;
    auto f = [&] { SwapToModels(swaps); };
    if (!Guarded(f)) {
        RestorePedPositions(swaps);
        swaps.clear();
    }
    const uint32_t args[] = {a1, a2, a3, a4, a5, a6, a7, a8, a9, a10};
    return RunPedCarSweep(self, edx, args, swaps);
}
char __fastcall hkPedCarHit(void *self, void *edx, uint32_t a1, uint32_t a2, uint32_t a3, uint32_t a4, uint32_t a5, void *model) {
    CarHandover *trace = nullptr;
    unsigned serial = 0;
    if (g_cfg.enabled && model && H->mission_state() != MML_MISSION_LOADING) {
        auto f = [&] {
            // A traffic element may have spawned or changed model since the previous mod tick.
            ListPeople();
            for (const MmlPedestrian &p : g_pedList) {
                if (p.model != model) continue;
                uint8_t *e = static_cast<uint8_t *>(p.element);
                V3 at;
                if (!ModelPosition(e, &at)) return;
                V3 &pos = game::at<V3>(e, kPedPos);
                V3 before = pos;
                const bool correctPosition = g_cfg.enabled && g_cfg.carKnockdown;
                const bool committed = correctPosition && CommitPedPosition(e, model, &before);
                trace = &g_handovers[g_handoverCount++ % g_handovers.size()];
                *trace = {};
                serial = trace->serial = g_handoverCount;
                trace->tick = g_tick;
                trace->animation = game::at<int>(e, kPedAnimation);
                trace->actor = e;
                trace->model = model;
                trace->elementBefore = before;
                trace->modelBefore = at;
                trace->committed = committed;
                trace->captured = SafeSnapshot(e, &trace->beforePose);
                trace->traceStart = GetTickCount64();
                if (trace->captured) trace->pelvisFrame = trace->beforePose.frames[18];
                Watch(model);
                if (trace->pelvisFrame) Watch(trace->pelvisFrame);
                const auto drawn = g_lastDrawn.find(e);
                if (trace->captured && drawn != g_lastDrawn.end() && drawn->second.model == model &&
                    g_tick - drawn->second.tick <= 2 &&
                    std::equal(std::begin(drawn->second.pose.frames), std::end(drawn->second.pose.frames),
                               std::begin(trace->beforePose.frames))) {
                    trace->lastDrawnCaptured = true;
                    trace->lastDrawnTick = drawn->second.tick;
                    trace->lastDrawnPelvis = drawn->second.pose.org[18];
                    trace->lastDrawnRoot = drawn->second.pose.org[19];
                    auto &previous = trace->trajectory[trace->trajectoryCount++];
                    previous.ms = drawn->second.ms;
                    previous.tick = drawn->second.tick;
                    previous.stage = TrajectoryStage::PreviousWorldDraw;
                    previous.root = trace->lastDrawnRoot;
                    previous.pelvis = trace->lastDrawnPelvis;
                    previous.hasPelvis = true;
                    previous.hasElement = false;
                    for (int i = 0; i < Ragdoll::COUNT; ++i)
                        trace->lastDrawnGap = std::max(trace->lastDrawnGap,
                            Len(drawn->second.pose.pts[i] - trace->beforePose.pts[i]));
                }
                trace->open = true;
                TracePedestrian(*trace, TrajectoryStage::BeforeHit, trace->traceStart);
                if (correctPosition && Len(at - before) > 0.3f && g_dodgeFixes++ < 50)
                    LOG("pedestrian %p car hit uses its model position (native offset %.3f m)", e, Len(at - before));
                if (correctPosition) pos = at;
                return;
            }
        };
        Guarded(f);
    }
    const char result = oPedCarHit(self, edx, a1, a2, a3, a4, a5, model);
    if (trace && trace->serial == serial) {
        trace->open = false;
        auto read = [&] { trace->elementAfter = game::at<V3>(trace->actor, kPedPos); };
        Guarded(read);
        TracePedestrian(*trace, TrajectoryStage::AfterHit, GetTickCount64());
        PersistTrajectory(*trace);
    }
    return result;
}

// ---- a bullet in a corpse: the mod's (the game's Hit would run a living person's hit logic on him; a dead
// pedestrian's element would be woken and cry out)
int HitFilter(void *person, int, MmlHumanHit *hit, void *) {
    if (H->mission_state() == MML_MISSION_LOADING) return MML_HIT_GAME;
    Ragdoll *rd = Find(person);
    if (!rd || !g_cfg.enabled || hit->was_alive || hit->type == 2 || hit->type == 3) return MML_HIT_GAME;   // (a blast: OnExplosion)
    if (hit->shape || hit->type == 0) {
        if (!g_cfg.bullets) return MML_HIT_TAKEN; // preserve native corpse state without an impulse
        V3 d{hit->dir.x, hit->dir.y, hit->dir.z}, at{hit->pos.x, hit->pos.y, hit->pos.z};
        if (Len(at) < 1e-3f) at = rd->Center();
        int part = Ragdoll::BulletBody(hit->shape - 1);
        float dv = Kick(rd, d, at, hit->damage, part, g_cfg.shot);
        g_lastBullet = {person, at, hit->shape - 1, part, dv, g_tick};
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

bool NativeBlastFall(void *human, float damage) {
    // Both builds of C_human::Hit choose the blast fall only above 100 damage. Check the
    // resulting animation too: the native hit can take health without starting that reaction.
    if (!(damage > 100.0f)) return false;
    MmlHumanAnim anim{};
    anim.size = sizeof anim;
    if (!H->human_anim_state(M, human, &anim) || (anim.state != 4 && anim.state != 6)) return false;
    return (anim.playing >= 106 && anim.playing <= 108) || anim.playing == 114;
}

void Hooks() {
    const game::ExeBuild &x = game::Exe();
    bool a = x.pedCarSweep && H->hook(M, x.pedCarSweep, reinterpret_cast<void *>(&hkPedCarSweep), reinterpret_cast<void **>(&oPedCarSweep));
    bool b = x.pedCarHit && H->hook(M, x.pedCarHit, reinterpret_cast<void *>(&hkPedCarHit), reinterpret_cast<void **>(&oPedCarHit));
    LOG("ragdoll: pedestrian car sweep and hit correction: %s", a && b ? "on" : "not known for this build");
    const uintptr_t tick = PedTickAddress();
    const bool tickHook = tick && x.sphereSweep && H->hook(M, tick, reinterpret_cast<void *>(&hkPedTick), reinterpret_cast<void **>(&oPedTick));
    const bool stepHook = tickHook && H->hook(M, x.sphereSweep, reinterpret_cast<void *>(&hkPedStepSweep),
                                              reinterpret_cast<void **>(&oPedStepSweep));
    if (tickHook && !stepHook) H->unhook(M, tick);
    LOG("ragdoll: pedestrian dodge movement sweep: %s", stepHook ? "on" : "unavailable");
}

} // namespace

// ---- the events
void Tick(float dt) {
    if (!game::g_exe || !g_cfg.enabled) return;
    try {
        TickBody(dt);
    } catch (const std::exception &e) {
        static unsigned n;
        if (n++ < 20) LOG("tick: exception '%s' (the rest of this tick skipped)", e.what());
    }
}

// The native wrists drive the pull on the next tick. Restore the animated
// local rotations after world drawing so the corrected wrists never feed back.
struct PullArmPose {
    void *model{};
    void *frame[7]{};
    Q local[7]{};
    Q applied[7]{};
} g_pullArms;
struct {
    unsigned serial{};
    PullArmPose pose{};
    float age{}, duration{};
} g_lastAttached;

void RestorePullArms() {
    const auto saved = g_pullArms;
    g_pullArms = {};
    if (!saved.model || g_freed.count(saved.model)) return;
    void *player = game::Player();
    if (!player || ModelOf(player) != saved.model) return;
    for (int i = 0; i < 7; ++i) {
        if (!saved.frame[i] || g_freed.count(saved.frame[i])) continue;
        const Q current = game::at<Q>(saved.frame[i], game::kFrameRot);
        // A skipped world-draw callback can leave a newer native animation here.
        if (memcmp(&current, &saved.applied[i], sizeof current) == 0)
            H->bone_set(M, saved.frame[i], &saved.local[i].w, nullptr);
    }
}

void SetPullQuaternion(int slot, void *frame, const Q &q) {
    g_pullArms.frame[slot] = frame;
    g_pullArms.local[slot] = game::at<Q>(frame, game::kFrameRot);
    H->bone_set(M, frame, &q.w, nullptr);
    g_pullArms.applied[slot] = game::at<Q>(frame, game::kFrameRot);
}

void SetPullRotation(int slot, void *frame, const M3 &local) {
    SetPullQuaternion(slot, frame, QuatOf(local));
}

void PosePullArms(const Ragdoll *driver) {
    if (!game::Player() || g_pullArms.model) return;
    V3 contact[2];
    const bool released = g_lastAttached.serial == driver->serial && (!driver->carExtraction ||
        (!jolt_ragdoll::SeatedCar(driver) && jolt_ragdoll::GripDistance(driver,nullptr)<0));
    float weight;
    if (released) {
        const float t = (driver->age-g_lastAttached.age)/g_lastAttached.duration;
        if (t >= 1) { g_lastAttached={}; return; }
        weight = 1-t*t*(3-2*t);
    } else {
        if (!driver->carExtraction) return;
        weight = jolt_ragdoll::PullContacts(driver, contact);
    }
    Ragdoll::PoseSnap pose{};
    auto *row = g_pull.body == driver && !g_pull.rows.empty() && !g_pull.rows.back().drawn ? &g_pull.rows.back() : nullptr;
    if ((!row && weight <= 0) || !SafeSnapshot(game::Player(), &pose)) return;
    if (row) row->nativeArms.Read(pose);
    if (weight <= 0) return;
    g_pullArms.model = ModelOf(game::Player());
    if (released) {
        // Fade only the correction from the last held frame. Native animation
        // continues in all seven joints, including after the carjack clip ends.
        if (row) { row->phase = "release"; row->releaseWeight = weight; }
        const auto &last = g_lastAttached.pose;
        const int bones[] = {15,16,11,12,6,14,10};
        if (g_lastAttached.serial != driver->serial || last.model != g_pullArms.model) return;
        for (int i = 0; i < 7; ++i) {
            void *frame = pose.frames[bones[i]];
            if (!frame || last.frame[i] != frame) continue;
            float base[16], held[16], now[16];
            QuatMatrix(last.local[i],base);
            QuatMatrix(last.applied[i],held);
            QuatMatrix(game::at<Q>(frame,game::kFrameRot),now);
            const Q delta = QuatOf(M3::Raw(base).T()*M3::Raw(held));
            const float t = delta.w < 0 ? -weight : weight;
            Q q{1-weight+t*delta.w,t*delta.x,t*delta.y,t*delta.z};
            const float inv = 1.f / std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
            float residual[16];
            QuatMatrix({q.w*inv,q.x*inv,q.y*inv,q.z*inv},residual);
            SetPullRotation(i,frame,M3::Raw(now)*M3::Raw(residual));
        }
        return;
    }
    if (row) row->phase = jolt_ragdoll::SeatedCar(driver) ? "reach" : "hold";
    const PullReach plan(pose.pts, contact, weight);
    const auto &goal = plan.goal;
    const auto &lean = plan.lean;
    float offset = 0;
    M3 parentWorld[2];
    for (int i = 0; i < 2; ++i) {
        void *parent = game::at<void *>(pose.frames[i ? 10 : 14], game::kFrameParent);
        parentWorld[i] = M3::FromWorld(game::WorldMatrix(parent));
    }
    void *spineParent = game::at<void *>(pose.frames[6], game::kFrameParent);
    SetPullRotation(4, pose.frames[6], pose.rot[6] * lean * M3::FromWorld(game::WorldMatrix(spineParent)).T());
    for (int i = 0; i < 2; ++i) {
        const int arm = i == 0 ? 15 : 11; // upper arm, elbow and wrist in PoseSnap
        const V3 clav = plan.clav[i], pivot = plan.pivot;
        const M3 swing = plan.swing[i];
        const M3 clavWorld = pose.rot[arm-1] * lean * swing;
        SetPullRotation(5+i, pose.frames[arm-1], clavWorld * (parentWorld[i] * lean).T());
        pose.rot[arm-1] = clavWorld;
        for (int k = arm; k <= arm+2; ++k) {
            pose.org[k] = clav + swing.Apply(pivot + lean.Apply(pose.org[k] - pivot) - clav);
            pose.rot[k] = pose.rot[k] * lean * swing;
        }
        const V3 shoulder = pose.org[arm], elbow = pose.org[arm+1], wrist = pose.org[arm+2];
        const float upper = Len(elbow - shoulder), fore = Len(wrist - elbow);
        if (std::min(upper, fore) < .01f) continue;
        const V3 axis = Norm(goal[i] - shoulder, Norm(wrist - shoulder));
        const float length = std::clamp(Len(goal[i] - shoulder), std::abs(upper - fore) + .001f, upper + fore - .001f);
        const float along = (upper * upper - fore * fore + length * length) / (2.f * length);
        // Carry the animated bend plane through the shortest swing of the
        // shoulder-to-wrist axis. Projecting the upper arm onto the new axis
        // flips the elbow when the target crosses the upper-arm direction.
        const auto plane = BasisFrom(shoulder, wrist, elbow);
        const V3 turn = Cross(plane.r[0], axis);
        const float denom = 1.f + Dot(plane.r[0], axis);
        const V3 bend = denom > 1e-5f ? Norm(plane.r[1] + Cross(turn, plane.r[1]) +
                                       Cross(turn, Cross(turn, plane.r[1])) * (1.f / denom)) : plane.r[1];
        const V3 solved = shoulder + axis * along + bend * std::sqrt(std::max(0.f, upper * upper - along * along));
        const V3 end = shoulder + axis * length;
        const int elbowPoint = i ? Ragdoll::RELB : Ragdoll::LELB;
        offset = std::max({offset,Len(end-pose.pts[elbowPoint+1]),Len(solved-pose.pts[elbowPoint])});
        // Keep the native twist of both links relative to the elbow bend plane.
        const M3 oldUpper = BasisFrom(shoulder, elbow, wrist), newUpper = BasisFrom(shoulder, solved, end);
        const M3 oldFore = BasisFrom(elbow, wrist, shoulder), newFore = BasisFrom(solved, end, shoulder);
        const M3 upperWorld = pose.rot[arm] * oldUpper.T() * newUpper;
        const M3 foreWorld = pose.rot[arm+1] * oldFore.T() * newFore;
        SetPullRotation(2*i, pose.frames[arm], upperWorld * pose.rot[arm-1].T());
        SetPullRotation(2*i+1, pose.frames[arm+1], foreWorld * upperWorld.T());
    }
    // A smoothstep's peak slope is 1.5. Allocate recovery time from the
    // actual displacement at contact, at ordinary arm-reaching speed.
    g_lastAttached = {driver->serial, g_pullArms, driver->age, std::max(.12f,1.5f*offset/1.35f)};
}

void Render() {
    if (!g_cfg.enabled) return;
    Purge();
    PruneOccupants();
    auto arms = [] { RestorePullArms(); };
    Guarded(arms);
    for (auto &o : g_occupants) {
        auto write = [&] { if (o.rail) ApplyRailPose(o); else o.body->Apply(); };
        Guarded(write);
    }
    TracePedestrians(TrajectoryStage::RenderBegin);
    for (auto &r : g_ragdolls) {
        // the game brought him back since our tick: the game's (the tick drops the ragdoll)
        if (SafeAliveAgain(r.get())) continue;
        if (r->carExtraction || g_lastAttached.serial == r->serial) {
            auto reach = [&] { PosePullArms(r.get()); };
            Guarded(reach);
        }
        // the engine played the model's animation (a death animation) since our write: the muscles' next target
        float ow = SafeOverwritten(r.get());
        if (ow > 0.5f && jolt_ragdoll::Has(r.get()) && SafeSnapshotNow(r.get())) r->animSerial++;
        if (!AtRest(r.get(), ow)) SafeApply(r.get(), true);
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

void RenderEnd() {
    for (auto &o:g_occupants) if (o.rail) {
        auto restore=[&] { RestoreRailPose(o); };
        Guarded(restore);
    }
}

// Stage 1 is after the world's actual drawing, unlike RENDER_BEGIN's pose preparation.
void Rendered() {
    if (!g_cfg.enabled) return;
    TracePedestrians(TrajectoryStage::WorldDrawn);
    for (auto &o : g_occupants) {
        if (!o.sampleDraw) continue;
        o.sampleDraw = false;
        auto read = [&] {
            if (g_freed.count(o.model) || o.body->UsesAny(g_freed)) return;
            M3 rotations[19];
            V3 origins[19];
            o.body->PhysicalPose(rotations, origins);
            const float *w = game::WorldMatrix(o.body->Frames()[9]);
            o.drawnNeckError = Len(origins[9] - V3{w[12], w[13], w[14]});
        };
        Guarded(read);
    }
    auto read = [&] {
        if (g_pull.body && !g_pull.rows.empty() && !g_pull.rows.back().drawn) {
            Ragdoll::PoseSnap pose{};
            if (g_pull.body->SnapshotNow(&pose)) {
                auto &row = g_pull.rows.back();
                row.drawnHead = pose.pts[Ragdoll::HEAD];
                row.drawnNose = pose.pts[Ragdoll::NOSE];
                if (game::Player() && SafeSnapshot(game::Player(), &pose)) {
                    row.drawnArms.Read(pose);
                    if (!g_pullArms.model) row.nativeArms = row.drawnArms;
                    row.armsRecorded = true;
                    if (!row.releaseWeight)
                        row.ikWeight = jolt_ragdoll::PullContacts(g_pull.body, row.contact, &row.gripPart);
                    row.drawnWrist[0] = pose.pts[Ragdoll::LHAND];
                    row.drawnWrist[1] = pose.pts[Ragdoll::RHAND];
                }
                row.drawn = true;
            }
        }
        for (const auto &kv : g_poses) {
            void *actor = kv.first;
            if (g_tick - kv.second.seen > 1 || !IsPed(actor) || Find(actor) || IsDead(actor)) continue;
            Ragdoll::PoseSnap pose{};
            if (!SafeSnapshot(actor, &pose)) continue;
            g_lastDrawn[actor] = {pose, ModelOf(actor), g_tick, GetTickCount64()};
        }
        for (auto &entry : g_handovers) {
            Ragdoll *r = entry.body;
            if (!r || Find(entry.actor) != r || r->Model() != entry.model || r->age > 3.5f ||
                (entry.worldSampled && entry.lastWorldTick == g_tick) ||
                (r->age > .2f && r->age < entry.nextWorldAge)) continue;
            entry.worldSampled = true;
            entry.lastWorldTick = g_tick;
            entry.nextWorldAge = r->age + .10f;
            const unsigned before = entry.poseCount;
            SampleHandoverPose(&entry, r, 5);
            if (entry.poseCount > before)
                entry.maxWorldError = std::max(entry.maxWorldError, entry.poses[entry.poseCount - 1].error);
        }
    };
    Guarded(read);   // observation only: no pose, velocity or collision policy changes
    auto restore = [] { RestorePullArms(); };
    Guarded(restore);
}

void OnHit(const MmlEvent &ev) {
    if (g_transportShot) g_transportShot->hitPerson = true;
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
    if (car && ev.type == MML_EVENT_PEDESTRIAN_HIT) {
        if (CarHandover *entry = PendingHandover(human)) {
            entry->nativeEvent = true;
            entry->car = attacker;
            entry->damage = std::isfinite(dmg) ? dmg : 0;
            entry->hitVelocity = dir;
            auto readCar = [&] {
                void *frame = attacker ? game::at<void *>(attacker, game::kActorFrame) : nullptr;
                const float *matrix = frame ? game::WorldMatrix(frame) : nullptr;
                if (!matrix) return;
                entry->carRight = {matrix[0], matrix[1], matrix[2]};
                entry->carForward = {matrix[8], matrix[9], matrix[10]};
                entry->carOrigin = {matrix[12], matrix[13], matrix[14]};
                entry->carBasis = true;
            };
            Guarded(readCar);
            TracePedestrian(*entry, TrajectoryStage::NativeHit, GetTickCount64());
        }
    }
    if (!g_cfg.enabled) return;
    ListPeople();   // (a hit between ticks: the pedestrians as they are)
    bool hasPos = Len(pos) > 1e-3f;
    bool bullet = type == 0 && !car;
    const HitStrength &strength = bullet ? g_cfg.shot : kOtherHit;
    Ragdoll *rd = Find(human);
    if (!rd) {
        bool died = wasAlive && !alive;
        // Native promotion marks the already dead driver as dead on the new
        // human. This event transfers ownership and carries no second impact.
        if (died && !IsPed(human) && TakeClonedDriver(human)) return;
        if (died && !IsPed(human) && BeingPulledFromCar(human)) return;
        // (someone alive in a car's seat is the game's: a crash's hit on its driver, type 5, made him a ragdoll that
        // got up on the street under his own car, still seated, and died there)
        // A gentle car touch is a native stagger, not a physical fall. In particular, do not re-knock a person
        // getting up beside a stopped car because it reported a 0.2 m/s contact (both builds' native thresholds).
        bool hardCarHit = car && (IsPed(human) ? dmg > 16.0f : Len(dir) > 3.0f);
        // Living NPCs keep their native blast reaction and position for subsequent hits and mission checks.
        const bool playerBlast = blast && type == 2 && wasAlive && alive && g_cfg.blastKnockdown &&
                                 human == H->human_player(M) && NativeBlastFall(human, dmg);
        bool knocked = alive && ((hardCarHit && g_cfg.carKnockdown) || body || playerBlast) &&
                       (IsPed(human) || !SeatCarOf(human));
        if (!died && !knocked) {
            if (!blast && !car && (!bullet || g_cfg.bullets)) Remember(human, dir, hasPos ? &pos : nullptr, dmg, strength);
            return;
        }
        std::string err;
        rd = Start(human, knocked, &err, car ? attacker : nullptr);
        if (!rd) return;
        rd->knockdown = knocked;
        LOG("hit %p type %d damage %.1f alive %d->%d%s: a ragdoll", human, type, dmg, wasAlive, alive,
            car ? Fmt(", a car at %.1f m/s", Len(carVel)).c_str() : body ? ", a body's blow" : "");
        if (died) TakeRemembered(human,rd);
    }
    if (car) {
        rd->vehicleImpact = true;
        rd->impactVehicle = attacker;
        return;   // vehicle/body contact in Jolt supplies the entire car impact
    }
    if (body) return;   // the blow is the caller's (KnockByBodies): the body's momentum where it struck
    V3 at = hasPos ? pos : rd->Center();
    if (!hasPos && !car && attacker && Len(dir) > 0.5f) {
        // no place known: the part nearest the line from the attacker's chest
        V3 from = game::at<V3>(attacker, game::kActorPos) + V3{0, 1.4f, 0};
        at = rd->ClosestToLine(from, from + Norm(dir) * 200.0f);
    }
    if (blast) return;   // an explosion: its blast throws every body near it (OnExplosion, the event after the hits)
    if (bullet && !g_cfg.bullets) return;
    if (bullet || dmg > 0) Kick(rd, dir, at, dmg, -1, strength);
}

// NewExplosion has applied damage. The bodies already in physics respond to its pressure wave.
// every body within its radius and a half thrown away from it. The speed at its centre grows with the root of its
// force (a grenade's 400: 12 m/s, a car's larger one more), at most 25 m/s; a third of it upwards.
void OnExplosion(const MmlExplosion &x) {
    if (!g_cfg.enabled) return;
    V3 c{x.pos.x, x.pos.y, x.pos.z};
    if (x.source && std::any_of(g_occupants.begin(), g_occupants.end(), [&](const auto &o) {
        return !o.rail && !o.wreckModel && o.carrier == x.source;
    })) g_wreckSearches.insert_or_assign(x.source, WreckSearch{c, WrecksNear(c)});

    if (g_cfg.blast <= 0) return;
    float reach = std::max(x.radius, 1.0f) * 1.5f;
    float speed = std::min(12.0f * std::sqrt(std::max(x.damage, 0.0f) / 400.0f), 25.0f) * g_cfg.blast;
    unsigned thrown = dogs::Blast(c,reach,speed,.35f);
    auto apply = [&](Ragdoll &body) {
        // Its own car carries the seated body through the blast via the cabin.
        // Free-field pressure here would launch limbs into the replacement shell.
        const void *seat = jolt_ragdoll::SeatedCar(&body);
        if (seat && seat == x.source) return;
        if (body.Blast(c, reach, speed, 0.35f)) ++thrown;
    };
    for (auto &r : g_ragdolls) apply(*r);
    for (auto &o : g_occupants) if (!o.rail) apply(*o.body);
    LOG("explosion at %.1f %.1f %.1f (radius %.1f, force %.0f): %u bodies thrown, %.1f m/s at its centre", c.x, c.y, c.z,
        x.radius, x.damage, thrown, speed);
}

void *FindNewWreck(void *car) {
    auto entry = g_wreckSearches.extract(car);
    if (entry.empty()) return nullptr;
    auto &search = entry.mapped();
    auto candidates = WrecksNear(search.origin);
    std::erase_if(candidates, [&](void *candidate) {
        return candidate == car || std::find(search.existing.begin(), search.existing.end(), candidate) != search.existing.end();
    });
    if (candidates.size() != 1) return nullptr;
    for (auto &[source, other] : g_wreckSearches) other.existing.push_back(candidates.front());
    return candidates.front();
}

bool CharTrafficOccupant(Occupant &o, void *wreck) {
    void *parent = wreck ? H->actor_frame(M, wreck) : nullptr;
    if (!parent) wreck = nullptr;
    const V3 at = parent ? V3{} : o.body->BodyPos(Ragdoll::B_PELVIS);
    std::unique_ptr<void, RemoveOccupantProp> model(H->prop_create(M, "ohorelec01.i3d", parent, &at.x, nullptr, 1.f));
    if (!model) return false;
    Ragdoll::PoseSnap live;
    std::string err;
    if (!Ragdoll::Snapshot(model.get(), nullptr, &live, &err)) return false;
    if (!jolt_ragdoll::TransferSeat(o.body.get(), wreck)) {
        wreck = nullptr;
        if (!H->prop_attach(M, model.get(), nullptr, 1)) return false;
        jolt_ragdoll::TransferSeat(o.body.get(), nullptr);
    }
    if (!o.body->RebindModel(model.get(), live, true)) return false;
    o.record = nullptr;
    o.carrier = wreck;
    o.model = model.get();
    o.body->actor = o.model;
    o.wreckModel = std::move(model);
    M3 rotation[19];
    V3 origins[19], points[Ragdoll::COUNT];
    o.body->PhysicalPose(rotation, origins);
    Ragdoll::PointsFromPose(rotation, origins, points);
    RememberWreckInterior(wreck, o.model, points);
    LOG("traffic ragdoll body %u: charred model %p in wreck %p", o.body->serial, o.model, wreck);
    return true;
}

void OnCarExploded(void *car) {
    if (!car || !g_cfg.enabled) return;
    std::vector<Ragdoll *> rebound;
    for (auto &body : g_ragdolls) {
        if (body->pedestrian || jolt_ragdoll::SeatedCar(body.get()) != car) continue;
        auto restore = [&] {
            if (!IsDead(body->actor)) return;
            void *wreck = SeatCarOf(body->actor);
            if (!wreck) return;
            Ragdoll::PoseSnap live;
            if (!SafeSnapshot(body->actor, &live)) return;
            if (wreck == car && std::equal(std::begin(live.frames), std::end(live.frames), body->Frames())) return;
            if (!jolt_ragdoll::TransferSeat(body.get(), wreck) || !body->RebindModel(ModelOf(body->actor), live)) return;
            RememberWreckInterior(wreck, body->Model(), live.pts);
            body->carExtraction = false;
            rebound.push_back(body.get());
            LOG("ragdoll %p body %u: rebound after car %p exploded, cabin %p", body->actor, body->serial, car, wreck);
        };
        Guarded(restore);
    }
    if (g_wreckSearches.contains(car)) {
        void *wreck = FindNewWreck(car);
        for (auto &o : g_occupants) {
            if (o.rail || o.wreckModel || o.carrier != car) continue;
            auto restore = [&] { if (CharTrafficOccupant(o, wreck)) rebound.push_back(o.body.get()); };
            Guarded(restore);
        }
    }
    // A new bone can reuse an address freed during this explosion. Only the bodies whose entire
    // skeleton was resolved again can survive those pending notifications.
    Purge(rebound);
    for (Ragdoll *body : rebound) WatchRagdoll(body);
}

void OnActorDestroyed(void *actor) {
    dogs::Forget(actor);
    jolt_ragdoll::ForgetCarrier(actor);
    std::erase_if(g_occupants, [actor](auto &o) {
        if (o.carrier != actor) return false;
        if (!o.wreckModel) return true;
        o.carrier = nullptr;
        return false;
    });
    if (g_pull.car == actor) g_pull.car = nullptr;
    for (auto &body : g_ragdolls) if (body->impactVehicle == actor) body->impactVehicle = nullptr;
    if (Find(actor)) Drop(actor);
    knock::Forget(actor);
    g_kept.erase(std::remove_if(g_kept.begin(), g_kept.end(), [&](const Kept &k) { return k.actor == actor; }), g_kept.end());
    std::erase_if(g_railPassengers, [actor](const auto &p) { return p.first == actor || p.second.tram == actor; });
    g_poses.erase(actor);
    g_lastDrawn.erase(actor);
    g_isHuman.erase(actor);
    g_failed.erase(actor);
    vehicles::Forget(actor);
    g_wreckInteriors.erase(actor);
    g_carTracks.erase(actor);
}

// ---- the water the bodies float in (jolt_world's buoyancy): the loader's world_water from just over the body, as the
// game asks it under a person, and only open water (a street, a pier, a bridge's deck between: none; the city's river
// is one plane under the whole city, under every street). Kept in cells of 2 m across and 1 m high for the mission
// (the game's water does not move, and every part of every body near water asks each tick). The flow: none.
std::unordered_map<uint64_t, float> g_waterCells;   // the surface's height; NaN: no open water there
bool WaterAt(float x, float y, float z, float *surfaceY, V3 *normal, V3 *flow) {
    int32_t cx = static_cast<int32_t>(std::floor(x * 0.5f)), cz = static_cast<int32_t>(std::floor(z * 0.5f));
    int32_t cy = static_cast<int32_t>(std::floor(y));
    uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(cx) & 0x1FFFFF) << 43) |
                   (static_cast<uint64_t>(static_cast<uint32_t>(cy) & 0x3FFFFF) << 21) | (static_cast<uint32_t>(cz) & 0x1FFFFF);
    auto it = g_waterCells.find(key);
    if (it == g_waterCells.end()) {
        if (g_waterCells.size() > 200000) g_waterCells.clear();
        MmlWater w{};
        w.size = sizeof w;
        // from 3 m over the cell (the host looks from 2 m over `at`), then, for a body sunk deeper, from 12 m: water
        // found with something of the static world between is not open
        float at[3] = {cx * 2.0f + 1.0f, cy + 1.0f, cz * 2.0f + 1.0f};
        bool found = H->world_water(M, at, &w) != 0;
        if (!found) {
            at[1] = cy + 10.0f;
            found = H->world_water(M, at, &w) != 0 && w.height > cy;
        }
        bool open = found && !w.covered;
        it = g_waterCells.emplace(key, open ? w.height : std::numeric_limits<float>::quiet_NaN()).first;
    }
    if (std::isnan(it->second)) return false;
    *surfaceY = it->second;
    *normal = {0, 1, 0};
    *flow = {0, 0, 0};
    return true;
}

void OnWorldReset(const char *reason) {
    headwear::Reset();
    dogs::Clear();
    g_lastAttached = {};
    g_pullArms = {};
    g_occupants.clear();
    g_railTrace.clear();
    g_carriageMeshes.clear();
    EndPullTrace("world reset");
    for (auto &entry : g_handovers) { entry.traceEnded = true; PersistTrajectory(entry); }
    ragcam::Stop();
    g_playerRisingUntil = 0;
    if (!g_ragdolls.empty()) LOG("%s: dropping %u ragdolls", reason, (unsigned)g_ragdolls.size());
    for (auto &r : g_ragdolls) Unshape(r->actor);
    g_ragdolls.clear();
    jolt_ragdoll::Clear();
    jolt_world::Stop();   // (the next mission's world is made around the player)
    scenery::Clear();
    g_waterCells.clear();   // (the next mission's water)
    vehicles::Clear();
    g_wreckInteriors.clear();
    g_wreckSearches.clear();
    knock::Clear();
    g_railPassengers.clear();
    g_poses.clear();
    g_lastDrawn.clear();
    g_isHuman.clear();
    g_failed.clear();
    g_carTracks.clear();
    for (auto &entry : g_handovers) entry = {};
    g_handoverCount = 0;
    g_lastSavedTrajectory = 0;
    g_kept.clear();
    g_peds.clear();
    g_pedList.clear();
    g_humanList.clear();
    g_freed.clear();
    // A same-mission load can keep frames alive. FrameGone still removes their watches when they die.
}

void Settings() {
    auto num = [](const char *k, float def) { return static_cast<float>(H->config_number(M, k, def)); };
    auto flag = [](const char *k, bool def) { return H->config_number(M, k, def ? 1 : 0) != 0; };
    Config d;
    const bool wasEnabled = g_cfg.enabled;
    g_cfg.enabled = flag("enabled", d.enabled);
    if (wasEnabled && !g_cfg.enabled) {
        auto restore = [] { RestorePullArms(); };
        Guarded(restore);
        Purge();
        const int state = H->mission_state();
        if (state == MML_MISSION_RUNNING || state == MML_MISSION_PAUSED) {
            for (const auto &r : g_ragdolls)
                if ((r->puppet || r->knockdown) && !IsDead(r->actor)) GetUp(r->actor, r.get());
        }
        OnWorldReset("disabled");
        for (void *frame : g_watched) H->frame_watch(M, frame, nullptr, nullptr);
        g_watched.clear();
    }
    g_cfg.carKnockdown = flag("car_knockdown", d.carKnockdown);
    g_cfg.carFeel = flag("car_feel", d.carFeel);
    g_cfg.trackRadius = num("track_radius", d.trackRadius);
    const bool hadBullets = g_cfg.bullets;
    g_cfg.bullets = flag("bullets", d.bullets);
    if (hadBullets && !g_cfg.bullets) {
        g_kept.clear();
        for (auto &r : g_ragdolls) { Unshape(r->actor); r->shapeAtRest = false; }
    }
    g_cfg.shot.perDamage = num("bullet_kick", d.shot.perDamage);
    g_cfg.whole = num("bullet_whole", d.whole);
    g_cfg.radius = num("bullet_radius", d.radius);
    g_cfg.shot.maxPart = num("bullet_max_part", d.shot.maxPart);
    g_cfg.shot.maxImpulse = num("bullet_most", d.shot.maxImpulse);
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

uint32_t VehicleSupport(void *car, void *) {
    if (!g_cfg.enabled || !g_cfg.carFeel || !g_carTracks.count(car)) return 0;
    return jolt_world::WheelSupport(car);
}

void Start() {
    headwear::Prepare();
    dogs::Start();
    if (MML_HAS(H, vehicle_support_hook) && H->vehicle_support_hook) {
        if (!H->vehicle_support_hook(M, VehicleSupport, nullptr))
            LOG("vehicle wheel support: native usability integration unavailable");
    }
    jolt_world::SetWaterQuery(WaterAt);
    jolt_ragdoll::Init(mod::Log);
    knock::Init(mod::Log);
    Settings();
    const bool v10 = game::g_exe == &game::kExe10;
    const bool traffic = H->hook_mid(M, v10 ? 0x506C00 : 0x4BADD0, TrafficDeath, nullptr);
    if (!H->hook_mid(M, v10 ? 0x4A8829 : 0x5882B9, TrafficDriverCloned, nullptr))
        LOG("traffic driver model handover unavailable");
    const bool rail = H->hook_mid(M, v10 ? 0x4D5160 : 0x484230, RailDeath, nullptr);
    const bool carHit = H->hook(M, v10 ? 0x46A670 : 0x423600, reinterpret_cast<void *>(&CarHit), reinterpret_cast<void **>(&originalCarHit));
    const bool railHit = H->hook(M, v10 ? 0x4E1D90 : 0x490B40, reinterpret_cast<void *>(&RailwayHit), reinterpret_cast<void **>(&originalRailHit));
    LOG("occupant death pose capture: traffic %d, railway %d; shot impulses: traffic %d, railway %d", traffic, rail, carHit, railHit);
    const auto bulletRay=game::Exe().bulletRay;
    if (!H->hook_mid(M,bulletRay,PhysicsBulletStretch,nullptr))
        LOG("physics bullet stretch unavailable");
    else {
        // 5A9336 / 5E4C86: the instruction after TickShoot sets the shooter's
        // exclusions, before the moving ray and its unmodified return address.
        constexpr uint8_t prep[]={0x8d,0x8c,0x24,0xf4,0,0,0,0x6a,0x01};
        const auto prepAt=bulletRay-0x26;
        if (memcmp(reinterpret_cast<const void *>(prepAt),prep,sizeof prep)!=0 ||
            !H->hook_mid(M,prepAt,BeforePhysicsBulletStretch,nullptr))
            LOG("native dog shot-volume exclusion unavailable");
    }
    Hooks();
    H->hit_filter(M, HitFilter, nullptr);
    const int mission = H->mission_state();
    if (g_cfg.enabled && (mission == MML_MISSION_RUNNING || mission == MML_MISSION_PAUSED))
        SafeWatchTramGetUps();
}

void Stop() {
    headwear::Reset();
    dogs::Clear();
    auto restore = [] { RestorePullArms(); };
    Guarded(restore);
    g_occupants.clear();
    g_carriageMeshes.clear();
    EndPullTrace("unload");
    ragcam::Stop();
    for (auto &r : g_ragdolls) Unshape(r->actor);
    g_ragdolls.clear();
    jolt_ragdoll::Clear();
    jolt_world::Stop();
}

std::string CarHandoverState() {
    std::string out = "[";
    const unsigned count = std::min<unsigned>(g_handoverCount, g_handovers.size());
    for (unsigned i = 0; i < count; ++i) {
        const auto &entry = g_handovers[(g_handoverCount - count + i) % g_handovers.size()];
        const V3 &before = entry.beforePose.pts[Ragdoll::PELVIS];
        out += Fmt("%s{\"actor\":\"0x%08X\",\"serial\":%u,\"tick\":%u,\"captured\":%s,\"started\":%s,"
                   "\"history\":%s,\"pre_hit_pose\":%s,\"sweep_committed\":%s,\"pose_gap_m\":%.4f,"
                   "\"element_before\":[%.4f,%.4f,%.4f],\"model_before\":[%.4f,%.4f,%.4f],"
                   "\"pelvis_before\":[%.4f,%.4f,%.4f],\"pelvis_initial\":[%.4f,%.4f,%.4f],"
                   "\"element_after_hit\":[%.4f,%.4f,%.4f],\"stepped\":%s,\"first_step_m\":%.4f,"
                   "\"pose_samples\":%u,\"max_draw_error_m\":%.4f,\"max_world_error_m\":%.4f,"
                   "\"last_drawn_captured\":%s,\"last_drawn_tick\":%u,\"last_drawn_gap_m\":%.4f,"
                   "\"last_drawn_pelvis\":[%.4f,%.4f,%.4f],\"last_drawn_root\":[%.4f,%.4f,%.4f]}",
                   i ? "," : "", (unsigned)(uintptr_t)entry.actor, entry.serial, entry.tick, entry.captured ? "true" : "false",
                   entry.started ? "true" : "false", entry.history ? "true" : "false", entry.preHitPose ? "true" : "false",
                   entry.committed ? "true" : "false",
                   entry.poseGap, entry.elementBefore.x, entry.elementBefore.y, entry.elementBefore.z,
                   entry.modelBefore.x, entry.modelBefore.y, entry.modelBefore.z, before.x, before.y, before.z,
                   entry.initial.x, entry.initial.y, entry.initial.z, entry.elementAfter.x, entry.elementAfter.y,
                   entry.elementAfter.z, entry.stepped ? "true" : "false",
                   entry.stepped ? Len(entry.firstStep - entry.bodyStart) : 0.0f, entry.poseCount, entry.maxDrawError,
                   entry.maxWorldError, entry.lastDrawnCaptured ? "true" : "false", entry.lastDrawnTick, entry.lastDrawnGap,
                   entry.lastDrawnPelvis.x,entry.lastDrawnPelvis.y,entry.lastDrawnPelvis.z,
                   entry.lastDrawnRoot.x,entry.lastDrawnRoot.y,entry.lastDrawnRoot.z);
    }
    return out + "]";
}

std::string State() {
    std::string list;
    for (auto &r : g_ragdolls) {
        V3 p = r->BodyPos(Ragdoll::B_PELVIS);
        float fastest = 0;
        for (int b = 0; b < Ragdoll::BodyCount(); b++) fastest = std::max(fastest, Len(r->BodyVel(b)));
        V3 ch = r->BodyPos(Ragdoll::B_CHEST), hd = r->BodyPos(Ragdoll::B_HEAD);
        list += Fmt("%s{\"actor\":\"0x%08X\",\"body_serial\":%u,\"pedestrian\":%s,\"knockdown\":%s,\"puppet\":%s,\"sleeping\":%s,\"vehicle_impact\":%s,\"age\":%.2f,"
                    "\"pelvis\":[%.3f,%.3f,%.3f],\"chest\":[%.3f,%.3f,%.3f],\"head\":[%.3f,%.3f,%.3f],\"fastest_part\":%.4f,\"quiet_s\":%.3f,\"grip_distance\":%.4f}",
                    list.empty() ? "" : ",", (unsigned)(uintptr_t)r->actor, r->serial, r->pedestrian ? "true" : "false",
                    r->knockdown ? "true" : "false",
                    r->puppet ? "true" : "false", r->sleeping ? "true" : "false", r->vehicleImpact ? "true" : "false", r->age, p.x, p.y, p.z, ch.x, ch.y, ch.z,
                    hd.x, hd.y, hd.z, fastest, r->sleepTimer, jolt_ragdoll::GripDistance(r.get()));
    }
    std::string occupants;
    for (auto &o : g_occupants) {
        o.sampleDraw = true;
        const auto p = o.body->BodyPos(Ragdoll::B_PELVIS), h = o.body->BodyPos(Ragdoll::B_HEAD);
        const auto v=o.body->BodyVel(Ragdoll::B_PELVIS);
        occupants += Fmt("%s{\"body_serial\":%u,\"model\":\"0x%08X\",\"carrier\":\"0x%08X\",\"kind\":\"%s_%s\",\"carriage\":%d,\"in_cabin\":%s,\"drawn_neck_error\":%.4f,\"age\":%.3f,\"pelvis\":[%.3f,%.3f,%.3f],\"head\":[%.3f,%.3f,%.3f],\"pelvis_velocity\":[%.4f,%.4f,%.4f]}",
            occupants.empty() ? "" : ",", o.body->serial, (unsigned)(uintptr_t)o.model, (unsigned)(uintptr_t)o.carrier,
            o.rail ? "rail" : o.wreckModel ? "wreck" : "traffic",
            o.driver ? "driver" : "passenger", o.carriage, jolt_ragdoll::SeatedCar(o.body.get()) ? "true" : "false", o.drawnNeckError,
            o.body->age,p.x,p.y,p.z,h.x,h.y,h.z,v.x,v.y,v.z);
    }
    std::string supports;
    for (const auto &[car, track] : g_carTracks) {
        const uint32_t mask = VehicleSupport(car, nullptr);
        if (mask)
            supports += Fmt("%s{\"car\":\"0x%08X\",\"wheels\":%u}", supports.empty() ? "" : ",",
                            (unsigned)(uintptr_t)car, mask);
    }
    return Fmt("{\"occupants\":[%s],\"car_handovers\":%s,\"wheel_support\":[%s],\"enabled\":%s,\"ragdolls\":[%s],\"tracked\":%u,\"pedestrians\":%u,\"world\":%s,\"corpse_hits\":%u,\"dodge_fixes\":%u,\"dodge_step_sweeps\":%u,"
               "\"scenery\":\"%s\",\"person_push\":%s,\"last_bullet\":%s}",
               occupants.c_str(), CarHandoverState().c_str(), supports.c_str(), g_cfg.enabled ? "true" : "false", list.c_str(), (unsigned)g_poses.size(), (unsigned)g_pedList.size(),
               jolt_world::Running() ? "true" : "false", g_corpseHits, g_dodgeFixes, g_dodgeStepSweeps, scenery::Describe().c_str(),
               jolt_world::PersonPushState().c_str(), g_lastBullet.actor ?
               Fmt("{\"actor\":\"0x%08X\",\"at\":[%.4f,%.4f,%.4f],\"shape\":%d,\"part\":%d,\"local_dv\":%.4f}",
                   (unsigned)(uintptr_t)g_lastBullet.actor, g_lastBullet.at.x, g_lastBullet.at.y, g_lastBullet.at.z,
                   g_lastBullet.shape, g_lastBullet.part, g_lastBullet.localDv).c_str() : "null");
}

std::string Exec(const std::string &cmd, const std::string &args, bool *ok) {
    *ok = true;
    if (cmd == "railtrace") return RailTrace(args);
    if (cmd == "dodge") {
        bool native = args.find("\"native\"") != std::string::npos;
        bool fixed = args.find("\"fixed\"") != std::string::npos;
        if (native == fixed) { *ok = false; return "dodge: mode must be native or fixed"; }
        const game::ExeBuild &x = game::Exe();
        for (uintptr_t at : {x.pedCarSweep, x.pedCarHit, PedTickAddress(), x.sphereSweep}) if (at) H->unhook(M, at);
        if (fixed) Hooks();
        return native ? "{\"mode\":\"native\"}" : "{\"mode\":\"fixed\"}";
    }
    if (cmd == "start") {   // a ragdoll on someone alive (a puppet: no death; `drop` gives him back)
        void *who = reinterpret_cast<void *>(static_cast<uintptr_t>(strtoul(args.c_str(), nullptr, 16)));
        ListPeople();
        if (!g_cfg.enabled) { *ok = false; return "start: ragdolls disabled"; }
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
        if (getup) GetUp(who, Find(who));
        Drop(who);
        return "{\"dropped\":true}";
    }
    if (cmd == "trajectory") {
        void *who = reinterpret_cast<void *>(static_cast<uintptr_t>(strtoul(args.c_str(), nullptr, 16)));
        for (unsigned i = 0; i < std::min<unsigned>(g_handoverCount, g_handovers.size()); ++i) {
            const auto &entry = g_handovers[(g_handoverCount - 1 - i) % g_handovers.size()];
            if (!who || entry.actor == who) return TrajectoryJson(entry);
        }
        *ok = false;
        return "trajectory: no retained car contact";
    }
    if (cmd == "handover") {
        void *who = reinterpret_cast<void *>(static_cast<uintptr_t>(strtoul(args.c_str(), nullptr, 16)));
        for (unsigned i = 0; i < std::min<unsigned>(g_handoverCount, g_handovers.size()); ++i) {
            const auto &entry = g_handovers[(g_handoverCount - 1 - i) % g_handovers.size()];
            if (entry.actor != who) continue;
            std::string out = Fmt("{\"serial\":%u,\"samples\":[", entry.serial);
            for (unsigned j = 0; j < entry.poseCount; ++j) {
                const auto &p = entry.poses[j];
                out += Fmt("%s{\"tick\":%u,\"stage\":%u,\"age\":%.4f,\"error_m\":%.4f,\"fastest_m_s\":%.3f,"
                           "\"physical\":[%.4f,%.4f,%.4f],\"drawn\":[%.4f,%.4f,%.4f],\"element\":[%.4f,%.4f,%.4f]}",
                           j ? "," : "", p.tick, p.stage, p.age, p.error, p.fastest,
                           p.physical.x,p.physical.y,p.physical.z,p.drawn.x,p.drawn.y,p.drawn.z,
                           p.element.x,p.element.y,p.element.z);
            }
            out += Fmt("],\"contacts_dropped\":%u,\"world_contacts_dropped\":%u,\"contacts\":[",
                       entry.contactsDropped, entry.worldContactsDropped);
            for (unsigned j = 0; j < entry.contactCount; ++j) {
                const auto &p = entry.contacts[j];
                const auto &c = p.contact;
                out += Fmt("%s{\"tick\":%u,\"age\":%.4f,\"car\":\"0x%08X\",\"part\":%d,\"body\":%u,\"sensor\":%s,\"depth_m\":%.4f,"
                           "\"at\":[%.4f,%.4f,%.4f],\"normal\":[%.4f,%.4f,%.4f],\"car_origin\":[%.4f,%.4f,%.4f],"
                           "\"car_right\":[%.4f,%.4f,%.4f],\"car_forward\":[%.4f,%.4f,%.4f],\"velocity_before\":[%.4f,%.4f,%.4f]}",
                           j ? "," : "", p.tick,p.age,(unsigned)(uintptr_t)c.car,c.part,c.body,c.sensor ? "true" : "false",c.depth,
                           c.at.x,c.at.y,c.at.z,c.n.x,c.n.y,c.n.z,c.carOrigin.x,c.carOrigin.y,c.carOrigin.z,
                           c.carRight.x,c.carRight.y,c.carRight.z,c.carForward.x,c.carForward.y,c.carForward.z,
                           c.velocity.x,c.velocity.y,c.velocity.z);
            }
            return out + "]}";
        }
        *ok = false;
        return "handover: no retained car contact on that person";
    }
    if (cmd == "inspect") {
        void *who = reinterpret_cast<void *>(static_cast<uintptr_t>(strtoul(args.c_str(), nullptr, 16)));
        Ragdoll *rd = Find(who);
        if (!rd) { *ok = false; return "inspect: no ragdoll on that person"; }
        std::string report;
        jolt_ragdoll::Exec("jolt", Fmt("body %08X", (unsigned)(uintptr_t)who), &report);
        return rd->Diagnose() + "\nrender errors [cm, degrees]: " + rd->BoneCheck() + "\n" + report;
    }
    if (cmd == "vehicle") {
        const void *car = reinterpret_cast<void *>(static_cast<uintptr_t>(strtoul(args.c_str(), nullptr, 16)));
        return jolt_world::DescribeCar(car ? car : game::PlayerCar());
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
