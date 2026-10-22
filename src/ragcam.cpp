// The ragdoll camera (ragcam.h).
#include "ragcam.h"
#include <windows.h>
#include <algorithm>
#include <cmath>
#include "fmt.h"
#include "mod.h"

using namespace mod;

namespace ragcam {
namespace {

constexpr float kRad = 0.01745329f;
constexpr float kNotch = 0.87f;          // the distance times this per wheel notch forward (nearer)
constexpr float kCountsPerAim = 400.0f;  // an "Aim X" of 1.0 for a frame is 400 of the mouse's counts
constexpr float kCountsPerNotch = 120;   // a wheel notch: 120 counts on the mouse's z axis

struct Config {
    bool on = true;
    float look = 0.12f;    // degrees the camera turns per count of the mouse
    float nearest = 1.2f;  // m: the nearest the wheel brings it
    float furthest = 12;   // m: the furthest
    float follow = 0.06f;  // s: how softly it follows the body (0: rigidly)
    float blend = 0.6f;    // s: back into the game's camera once he is up (0: at once)
    float start = 3;       // wheel notches further than the game's camera it starts at (negative: nearer)
} g_cfg;

int g_slotX = -1, g_slotY = -1, g_slotUp = -1, g_slotDown = -1;
bool g_want;               // a target was given this tick
V3 g_target;               // the point circled (world)
bool g_overriding;         // camera_override is ours
bool g_have;               // the orbit is set up (from the game's camera's place)
bool g_captured;           // the mouse's actions kept from the game
V3 g_smooth;               // the point followed, softened
float g_yaw, g_pitch, g_dist;
V3 g_lastEye, g_lastFwd;
float g_blendLeft;
LARGE_INTEGER g_last, g_freq;
unsigned g_frames, g_walls;
float g_wheelSeen;         // the last wheel's value read (the state shows it)

void Capture(bool on) {
    if (on == g_captured) return;
    for (int s : {g_slotX, g_slotY, g_slotUp, g_slotDown})
        if (s >= 0) H->action_capture(M, s, on ? 1 : 0);
    g_captured = on;
}

V3 Forward() {
    float cp = std::cos(g_pitch);
    return {std::sin(g_yaw) * cp, std::sin(g_pitch), std::cos(g_yaw) * cp};
}

void Begin(const MmlViewCamera *c) {
    V3 eye{c->world[12], c->world[13], c->world[14]}, d = g_target - eye;
    float l = Len(d);
    if (l < 1e-3f) { d = {c->world[8], c->world[9], c->world[10]}; l = 1; }
    g_dist = std::clamp(l * std::pow(kNotch, -g_cfg.start), g_cfg.nearest, g_cfg.furthest);
    g_yaw = std::atan2(d.x, d.z);
    g_pitch = std::clamp(std::asin(std::clamp(d.y / l, -1.0f, 1.0f)), -80.0f * kRad, 60.0f * kRad);
    g_smooth = g_target;
    g_have = true;
}

// The orbit's pose this frame: turned by the mouse, the wheel's distance, kept out of walls.
void Orbit(float dt, V3 *eye, V3 *fwd) {
    float mx = g_slotX >= 0 ? H->action_held(g_slotX) * kCountsPerAim : 0;
    float my = g_slotY >= 0 ? H->action_held(g_slotY) * kCountsPerAim : 0;
    float up = g_slotUp >= 0 ? H->action_held(g_slotUp) : 0, down = g_slotDown >= 0 ? H->action_held(g_slotDown) : 0;
    float wheel = (up - down) * kCountsPerAim / kCountsPerNotch;   // notches, up (away) positive
    if (wheel != 0) g_wheelSeen = up - down;
    g_yaw += mx * g_cfg.look * kRad;
    g_pitch = std::clamp(g_pitch - my * g_cfg.look * kRad, -80.0f * kRad, 60.0f * kRad);
    if (wheel != 0) g_dist = std::clamp(g_dist * std::pow(kNotch, wheel), g_cfg.nearest, g_cfg.furthest);
    float k = g_cfg.follow > 1e-3f ? 1.0f - std::exp(-dt / g_cfg.follow) : 1.0f;
    g_smooth = g_smooth + (g_target - g_smooth) * k;
    V3 f = Forward();
    V3 e = g_smooth - f * g_dist;
    // out of walls and cars: the game's collision (the static world and the moving objects but people) from just off
    // the body towards the camera; stopped short of what it meets (a body behind a car was looked at through it)
    V3 from = g_smooth - f * 0.4f;
    MmlHit hit{};
    hit.size = sizeof hit;
    const float a[3] = {from.x, from.y, from.z}, b[3] = {e.x, e.y, e.z};
    if (g_dist > 0.6f && H->world_ray(M, a, b, MML_WORLD_ALL | MML_WORLD_NO_PEOPLE, H->human_player(M), &hit)) {
        V3 h{hit.pos[0], hit.pos[1], hit.pos[2]};
        e = g_smooth - f * std::max(0.6f, Len(h - g_smooth) - 0.25f);
        g_walls++;
    }
    *eye = e;
    *fwd = f;
}

int View(MmlViewCamera *c, void *) {
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    float dt = std::clamp(static_cast<float>(now.QuadPart - g_last.QuadPart) / static_cast<float>(g_freq.QuadPart), 0.0f, 0.1f);
    g_last = now;
    V3 eye, fwd;
    if (g_want) {
        if (!g_have) Begin(c);
        Orbit(dt, &eye, &fwd);
        g_lastEye = eye;
        g_lastFwd = fwd;
    } else if (g_have && g_blendLeft > 0) {
        // from where the orbit left off into the game's camera, eased
        float s = 1.0f - g_blendLeft / std::max(g_cfg.blend, 1e-3f);
        s = s * s * (3 - 2 * s);
        V3 ge{c->world[12], c->world[13], c->world[14]}, gf{c->world[8], c->world[9], c->world[10]};
        eye = g_lastEye + (ge - g_lastEye) * s;
        fwd = Norm(g_lastFwd + (gf - g_lastFwd) * s, gf);
        g_blendLeft -= dt;
    } else {
        g_have = false;
        return 0;
    }
    // forward and a world up: the host makes right and up orthonormal from them
    c->world[8] = fwd.x; c->world[9] = fwd.y; c->world[10] = fwd.z;
    c->world[12] = eye.x; c->world[13] = eye.y; c->world[14] = eye.z;
    c->world[4] = 0; c->world[5] = 1; c->world[6] = 0;
    c->flags |= MML_VIEW_OWN_SECTOR;   // (the room looked up at the new eye)
    g_frames++;
    return 1;
}

void Give(bool on) {
    if (on == g_overriding) return;
    H->camera_override(M, on ? View : nullptr, nullptr);
    g_overriding = on;
}

} // namespace

void Settings() {
    auto num = [](const char *k, float def) { return static_cast<float>(H->config_number(M, k, def)); };
    Config d;
    g_cfg.on = H->config_number(M, "ragcam.on", d.on ? 1 : 0) != 0;
    g_cfg.look = num("ragcam.look", d.look);
    g_cfg.nearest = num("ragcam.near", d.nearest);
    g_cfg.furthest = std::max(num("ragcam.far", d.furthest), g_cfg.nearest);
    g_cfg.follow = num("ragcam.follow", d.follow);
    g_cfg.blend = num("ragcam.blend", d.blend);
    g_cfg.start = num("ragcam.start", d.start);
    if (g_slotX < 0) {
        g_slotX = H->action_slot("Aim X");
        g_slotY = H->action_slot("Aim Y");
        g_slotUp = H->action_slot("Cycle Up");
        g_slotDown = H->action_slot("Cycle Down");
        QueryPerformanceFrequency(&g_freq);
        QueryPerformanceCounter(&g_last);
    }
}

void Follow(const V3 *target) {
    bool want = target && g_cfg.on;
    if (want) {
        if (!g_want) { g_blendLeft = 0; Capture(true); Give(true); }
        g_target = *target;
    } else if (g_want) {
        Capture(false);
        g_blendLeft = g_have ? g_cfg.blend : 0;
    }
    g_want = want;
    // given back once the blend is over (another mod's camera, a free camera, gets it again when it asks)
    if (!g_want && g_overriding && g_blendLeft <= 0) { Give(false); g_have = false; }
}

void Stop() {
    Capture(false);
    Give(false);
    g_want = g_have = false;
    g_blendLeft = 0;
}

std::string State() {
    return Fmt("{\"on\":%s,\"following\":%s,\"blending\":%s,\"overriding\":%s,\"frames\":%u,\"walls\":%u,\"yaw\":%.1f,"
               "\"pitch\":%.1f,\"distance\":%.2f,\"wheel_seen\":%.3f}",
               g_cfg.on ? "true" : "false", g_want ? "true" : "false", !g_want && g_blendLeft > 0 ? "true" : "false",
               g_overriding ? "true" : "false", g_frames, g_walls, g_yaw / kRad, g_pitch / kRad, g_dist, g_wheelSeen);
}

} // namespace ragcam
