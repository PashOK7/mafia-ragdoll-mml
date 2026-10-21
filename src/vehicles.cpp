// The vehicles as the physics sees them (vehicles.h).
//
// A car's body is read once a model (the game swaps a car's model: a traffic car becomes the player's when he gets
// in): every visual of it the loader gives (actor_meshes: its body, doors, wheels, each in its frame's pose now), the
// triangles in the car's frame, one hull a visual (a single hull of a whole car fills the space under its bumpers).
// Parts smaller than 8 cm (lamps, badges) and flat ones (panes, plates: no volume for a hull) are left out; a part
// whose frame is the model's WHL<n> or WHR<n> is a wheel (mafia-ragdoll's vehicles.cpp, from the cars' .4ds files
// then; the meshes the game draws now).
#include "vehicles.h"
#include <windows.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>
#include "game.h"
#include "mod.h"

using namespace mod;

namespace vehicles {
namespace {

constexpr float kMinPart = 0.08f;
constexpr int kCar = 4, kTram = 8;

struct CarBody {
    std::vector<V3> all;
    std::vector<std::vector<V3>> parts;
    std::vector<char> wheel;
    void *root = nullptr;
    unsigned version = 0;
    unsigned long retryAt = 0;
};
std::unordered_map<void *, CarBody> g_bodies;
std::vector<MmlMeshPiece> g_pieces;

template <class F> bool Guarded(F &f) {
    __try { f(); return true; } __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}

bool CarPose(void *car, V3 *t, V3 ax[3]) {
    void *frame = game::at<void *>(car, game::kActorFrame);
    if (!frame) return false;
    const float *m = game::WorldMatrix(frame);
    if (!m) return false;
    M3 r = M3::FromWorld(m);
    r.Orthonormalize();
    for (int i = 0; i < 3; i++) ax[i] = r.r[i];
    *t = {m[12], m[13], m[14]};
    return true;
}

bool IsWheelFrame(void *frame) {
    const char *n = nullptr;
    auto f = [&] { n = game::at<char *>(frame, game::kFrameName); };
    if (!Guarded(f) || !n) return false;
    const char *dot = strrchr(n, '.');
    const char *s = dot ? dot + 1 : n;
    return (s[0] == 'W' || s[0] == 'w') && (s[1] == 'H' || s[1] == 'h') && (s[2] == 'L' || s[2] == 'l' || s[2] == 'R' || s[2] == 'r') &&
           s[3] >= '0' && s[3] <= '9';
}

bool Unfit(const std::vector<V3> &p) {
    if (p.size() < 4) return true;
    V3 lo = p[0], hi = p[0];
    for (const V3 &q : p) {
        lo = {std::fmin(lo.x, q.x), std::fmin(lo.y, q.y), std::fmin(lo.z, q.z)};
        hi = {std::fmax(hi.x, q.x), std::fmax(hi.y, q.y), std::fmax(hi.z, q.z)};
    }
    if (std::fmax(hi.x - lo.x, std::fmax(hi.y - lo.y, hi.z - lo.z)) < kMinPart) return true;
    V3 n{0, 0, 0};
    for (size_t t = 0; t + 2 < p.size() && Dot(n, n) < 1e-8f; t += 3) n = Cross(p[t + 1] - p[t], p[t + 2] - p[t]);
    n = Norm(n, {0, 1, 0});
    float thick = 0;
    for (const V3 &q : p) thick = std::fmax(thick, std::fabs(Dot(q - p[0], n)));
    return thick < 0.005f;
}

const CarBody *Body(void *car) {
    void *root = nullptr;
    auto r = [&] { root = game::at<void *>(car, game::kActorFrame); };
    if (!Guarded(r) || !root) return nullptr;
    auto it = g_bodies.find(car);
    if (it != g_bodies.end() && it->second.root == root && (!it->second.all.empty() || GetTickCount() < it->second.retryAt))
        return it->second.all.empty() ? nullptr : &it->second;
    CarBody &body = g_bodies[car];
    unsigned version = body.version + 1;
    body = CarBody{};
    body.root = root;
    body.version = version;
    body.retryAt = GetTickCount() + 1000;
    V3 t, ax[3];
    bool posed = false;
    auto p = [&] { posed = CarPose(car, &t, ax); };
    if (!Guarded(p) || !posed) return nullptr;
    int n = H->actor_meshes(M, car, nullptr, 0);
    if (n <= 0) return nullptr;
    g_pieces.resize(n);
    n = H->actor_meshes(M, car, g_pieces.data(), n);
    for (int i = 0; i < n && i < (int)g_pieces.size(); i++) {
        const MmlMeshPiece &mp = g_pieces[i];
        std::vector<V3> part;
        part.reserve(mp.triangles * 3);
        const float *w = mp.world;
        for (int f = 0; f < mp.triangles; f++)
            for (int k = 0; k < 3; k++) {
                uint32_t v = mp.index[f * 3 + k];
                if (v >= (uint32_t)mp.vertices) continue;
                const float *q = mp.pos + v * 3;
                V3 wp{q[0] * w[0] + q[1] * w[4] + q[2] * w[8] + w[12], q[0] * w[1] + q[1] * w[5] + q[2] * w[9] + w[13],
                      q[0] * w[2] + q[1] * w[6] + q[2] * w[10] + w[14]};
                V3 d = wp - t;
                V3 l{Dot(d, ax[0]), Dot(d, ax[1]), Dot(d, ax[2])};
                body.all.push_back(l);
                part.push_back(l);
            }
        if (Unfit(part)) continue;
        body.parts.push_back(std::move(part));
        body.wheel.push_back(IsWheelFrame(mp.visual));
    }
    if (body.all.empty()) return nullptr;
    Log("vehicles: car %p body from %u model triangles, %u parts", car, (unsigned)body.all.size() / 3, (unsigned)body.parts.size());
    return &body;
}

int TypeOf(void *a) {
    int t = -1;
    auto f = [&] { t = game::at<int>(a, game::kActorType); };
    Guarded(f);
    return t;
}

} // namespace

// The vehicles that may be near, asked of the loader four times a second (world_near walks and sorts every actor:
// asked twice a tick for cars and trams, it was a tenth of the mod's time): a car at 50 m/s goes 12 m between two
// askings, inside the 20 m the candidates reach past the radius, and each list is made from where they are now. A
// destroyed one goes at once (Forget: the loader tells every actor's destruction).
namespace {
std::vector<void *> g_candidates;
V3 g_candAt{};
float g_candRadius = -1;
unsigned long g_candTick;
void Candidates(const V3 &at, float radius) {
    unsigned long now = GetTickCount();
    if (g_candRadius >= radius + Len(at - g_candAt) && now - g_candTick < 250) return;
    g_candidates.clear();
    g_candAt = at;
    g_candRadius = std::max(radius, 80.0f) + 20.0f;
    g_candTick = now;
    void *found[256];
    const float p[3] = {at.x, at.y, at.z};
    for (int type : {kCar, kTram}) {
        int n = H->world_near(M, p, g_candRadius, type, found, 256);
        for (int i = 0; i < n && i < 256; i++) g_candidates.push_back(found[i]);
    }
}
} // namespace

void GatherVehicles(const V3 &lo, const V3 &hi, std::vector<smoke::Mover> &out) {
    out.clear();
    V3 mid = (lo + hi) * 0.5f;
    static std::vector<void *> cars;
    ListVehicles(mid, Len(hi - mid) + 5.0f, cars);
    for (void *car : cars) {
        const CarBody *body = Body(car);
        if (!body) continue;
        V3 t, ax[3];
        if (!Pose(car, &t, ax)) continue;
        if (t.x < lo.x || t.y < lo.y || t.z < lo.z || t.x > hi.x || t.y > hi.y || t.z > hi.z) continue;
        smoke::Mover m;
        m.key = car;
        m.kind = smoke::Mover::HULL;
        m.c = t;
        for (int i = 0; i < 3; i++) m.ax[i] = ax[i];
        m.hull = &body->all;
        m.hullParts = &body->parts;
        for (size_t i = 0; i < body->wheel.size() && i < 64; i++) if (body->wheel[i]) m.wheelParts |= 1ull << i;
        m.shapeVersion = body->version;
        m.hullIsTriangles = true;
        // a car's mass is C_car's field; a tram (type 8) is another class: kinematic, with its exact mesh
        float mass = 0;
        if (TypeOf(car) == kCar) {
            auto f = [&] { mass = game::at<float>(static_cast<uint8_t *>(car) + game::Exe().vehicle, game::Exe().vehMass); };
            Guarded(f);
        }
        m.mass = mass > 300.0f && mass < 20000.0f ? mass : 0.0f;
        m.hullAsMesh = m.mass == 0.0f;
        out.push_back(m);
    }
}

void ListVehicles(const V3 &at, float radius, std::vector<void *> &out) {
    out.clear();
    Candidates(at, radius);
    for (void *car : g_candidates) {
        V3 t, ax[3];
        if (Pose(car, &t, ax) && Len(t - at) <= radius) out.push_back(car);
    }
}

void Clear() { g_bodies.clear(); g_candidates.clear(); g_candRadius = -1; }
void Forget(void *car) {
    g_bodies.erase(car);
    g_candidates.erase(std::remove(g_candidates.begin(), g_candidates.end(), car), g_candidates.end());
}

bool Pose(void *car, V3 *origin, V3 axes[3]) {
    bool ok = false;
    auto f = [&] { ok = CarPose(car, origin, axes); };
    return Guarded(f) && ok;
}

bool ModelTriangles(void *car, const std::vector<V3> **tris) {
    const CarBody *b = Body(car);
    if (!b) return false;
    *tris = &b->all;
    return true;
}

bool HullParts(void *car, const std::vector<std::vector<V3>> **parts, unsigned *version) {
    const CarBody *b = Body(car);
    if (!b || b->parts.empty()) return false;
    *parts = &b->parts;
    *version = b->version;
    return true;
}

} // namespace vehicles
