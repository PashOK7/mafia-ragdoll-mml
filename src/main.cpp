// The mod's entry: the loader's host table kept, the ragdolls started and stopped, the events and commands wired.
//
// Events: MISSION_TICK (the ragdolls' tick, after the game's world moved), RENDER_BEGIN (the bones written over the
// game's animation), HUMAN_HIT and PEDESTRIAN_HIT (a death, a knock-down, a bullet's push), ACTOR_DESTROYED,
// MISSION_CLOSE, CONFIG_CHANGED (the settings, from config.json or the game's Options > Mods).
// Commands (the bridge's `ragdoll.<name>`): state, start {actor | pedestrian}, drop {actor | pedestrian}, jolt, knock.
#include <windows.h>
#include <cstdarg>
#include <cstdio>
#include <string>
#include "core.h"
#include "game_build.h"
#include "mml/mml.h"
#include "mod.h"

namespace mod {
const MmlHost *H;
MmlMod *M;

void Log(const char *fmt, ...) {
    char line[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    H->log(M, MML_LOG_INFO, "%s", line);
}
void Warn(const char *fmt, ...) {
    char line[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    H->log(M, MML_LOG_WARN, "%s", line);
}
void Reply(MmlReply *r, const std::string &text) { H->reply_append(r, text.c_str(), text.size()); }
} // namespace mod

using namespace mod;

namespace {

void OnEvent(const MmlEvent *ev, void *) {
    switch (ev->type) {
    case MML_EVENT_MISSION_TICK: core::Tick(ev->dt); break;
    case MML_EVENT_RENDER_BEGIN: if (ev->world) core::Render(); break;
    case MML_EVENT_HUMAN_HIT:
    case MML_EVENT_PEDESTRIAN_HIT: core::OnHit(*ev); break;
    case MML_EVENT_ACTOR_DESTROYED: core::OnActorDestroyed(ev->actor); break;
    case MML_EVENT_MISSION_CLOSE: core::OnMissionClose(); break;
    case MML_EVENT_CONFIG_CHANGED: core::Settings(); break;
    default: break;
    }
}

// the first "0x..." of a command's json: the actor or pedestrian it names
std::string Address(const char *args) {
    const char *p = args ? strstr(args, "0x") : nullptr;
    if (!p) return "";
    const char *e = p + 2;
    while ((*e >= '0' && *e <= '9') || (*e >= 'a' && *e <= 'f') || (*e >= 'A' && *e <= 'F')) e++;
    return std::string(p, e);
}

int Command(const char *args, MmlReply *r, void *user) {
    const char *name = static_cast<const char *>(user);
    if (!strcmp(name, "state")) { Reply(r, core::State()); return 0; }
    bool ok = false;
    std::string out = core::Exec(name, !strcmp(name, "start") || !strcmp(name, "drop") ? Address(args) : std::string(args ? args : ""), &ok);
    if (!ok) { H->reply_error(r, out.c_str()); return 1; }
    Reply(r, out);
    return 0;
}

} // namespace

MML_EXPORT uint32_t mml_mod_api(void) { return MML_API_VERSION; }

MML_EXPORT int mml_mod_load(const MmlHost *host, MmlMod *m) {
    H = host;
    M = m;
    if (!MML_HAS(H, scene_ready)) {
        H->log(M, MML_LOG_ERROR, "this loader is too old for the ragdolls (no scene_ready: MafiaModLoader of 2026-10-22 or newer)");
        return 1;
    }
    // the build's addresses (game_build.h: this module's own copy of the tables)
    uintptr_t ls3d = H->module_base("LS3DF.dll");
    if (!ls3d || !game::Detect(ls3d)) {
        H->log(M, MML_LOG_ERROR, "not a Mafia build the ragdolls know (Game.exe %s, LS3DF.dll %s)", game::ExeName(), game::Ls3dName());
        return 1;
    }
    for (MmlEventType t : {MML_EVENT_MISSION_TICK, MML_EVENT_RENDER_BEGIN, MML_EVENT_HUMAN_HIT, MML_EVENT_PEDESTRIAN_HIT,
                           MML_EVENT_ACTOR_DESTROYED, MML_EVENT_MISSION_CLOSE, MML_EVENT_CONFIG_CHANGED})
        H->subscribe(M, t, OnEvent, nullptr);
    static const char *const kCommands[][2] = {
        {"state", "the ragdolls: each one's actor, kind, age, pelvis; the people tracked, the world, the counts"},
        {"start", "a ragdoll on someone alive (a puppet), by his address: {\"actor\": \"0x...\"} or {\"pedestrian\": \"0x...\"}"},
        {"drop", "the ragdoll taken off someone (a living one gets up): {\"actor\": \"0x...\"}"},
        {"jolt", "the rigid-body world and the ragdolls' bodies, described"},
        {"knock", "the knock-downs by bodies: the last decisions"},
    };
    for (auto &c : kCommands) H->command(M, c[0], c[1], Command, const_cast<char *>(c[0]));
    core::Start();
    Log("ragdoll: loaded (game %s)", H->game_build());
    return 0;
}

MML_EXPORT void mml_mod_unload(MmlMod *) { core::Stop(); }
