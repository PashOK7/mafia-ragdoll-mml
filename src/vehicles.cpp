// The vehicles as the physics sees them (vehicles.h).
//
// Sample the visible parts in the car's own frame. Changed doors and detached parts update the
// compound while unchanged parts keep their cooked hulls. The loader supplies the posed meshes; the
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

struct PiecePose {
    void *visual{};
    const float *mesh{};
    int vertices{}, triangles{};
    V3 origin{}, axes[3]{};
    bool wheel{};
    bool door{};
    uint32_t nativeWheel{};
    size_t first{}, count{};
    int part = -1;
};
uint64_t g_nextHull;
struct CarBody {
    std::vector<PiecePose> poses;
    std::vector<uint64_t> revisions;
    std::vector<V3> all;
    std::vector<std::vector<V3>> parts;
    std::vector<char> wheel;
    std::vector<uint32_t> nativeWheels;
    void *root = nullptr;
    unsigned version = 0;
    ULONGLONG refreshAt = 0;
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

bool IsDoorFrame(void *frame) {
    const char *name = game::at<char *>(frame, game::kFrameName);
    if (!name) return false;
    const char *dot = strrchr(name, '.');
    const char *part = dot ? dot + 1 : name;
    return _strnicmp(part, "door", 4) == 0 && part[4] >= '0' && part[4] <= '9';
}

bool DoorPoseChanged(const CarBody &body, V3 origin, const V3 *axes) {
    auto local = [&](V3 v) { return V3{Dot(v, axes[0]), Dot(v, axes[1]), Dot(v, axes[2])}; };
    bool changed = false;
    auto read = [&] {
        for (const auto &piece : body.poses) {
            if (!piece.door) continue;
            const float *w = game::WorldMatrix(piece.visual);
            changed = !w || Len(local(V3{w[12], w[13], w[14]} - origin) - piece.origin) > .005f;
            for (int j = 0; w && j < 3 && !changed; ++j)
                changed = Len(local({w[j * 4], w[j * 4 + 1], w[j * 4 + 2]}) - piece.axes[j]) > .005f;
            if (changed) break;
        }
    };
    return !Guarded(read) || changed;
}

uint32_t NativeWheel(void *car, void *visual) {
    if (game::at<int>(car, game::kActorType) != kCar) return 0;
    game::Car native(car);
    for (int i = 0; i < native.WheelCount() && i < 32; ++i) {
        void *wheel = native.Wheel(i);
        if (wheel && game::at<void *>(wheel, game::kWheelFrame) == visual)
            return static_cast<uint32_t>(i + 1);
    }
    return 0;
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
    CarBody &cached = g_bodies[car];
    const auto now = GetTickCount64();
    V3 t, ax[3];
    if (!Pose(car, &t, ax)) return nullptr;
    // Check the hinges on every sample. The ordinary mesh refresh interval is
    // longer than several physics steps, enough for a door to pass a limb.
    if (cached.root == root && now < cached.refreshAt && !DoorPoseChanged(cached, t, ax))
        return cached.parts.empty() ? nullptr : &cached;
    const bool extended = MML_HAS(H, actor_meshes_ex) && H->actor_meshes_ex;
    auto meshes = [&] {
        return extended ? H->actor_meshes_ex(M, car, MML_ACTOR_MESH_HIDDEN_ROOT, g_pieces.data(), (int)g_pieces.size())
                        : H->actor_meshes(M, car, g_pieces.data(), (int)g_pieces.size());
    };
    if (g_pieces.empty()) g_pieces.resize(32);
    int n = meshes();
    if (n > (int)g_pieces.size()) { g_pieces.resize(n); n = meshes(); }
    if (n < 0 || n > (int)g_pieces.size()) return nullptr;
    auto local = [&](V3 v) { return V3{Dot(v, ax[0]), Dot(v, ax[1]), Dot(v, ax[2])}; };
    std::vector<PiecePose> poses(n);
    std::vector<const PiecePose *> reused(n);
    bool changed = cached.root != root || poses.size() != cached.poses.size();
    for (int i = 0; i < n; ++i) {
        const MmlMeshPiece &mp = g_pieces[i];
        PiecePose &p = poses[i];
        p.visual = mp.visual; p.mesh = mp.pos;
        p.vertices = mp.vertices; p.triangles = mp.triangles;
        p.wheel = IsWheelFrame(p.visual);
        p.door = IsDoorFrame(p.visual);
        if (p.wheel) p.nativeWheel = NativeWheel(car, p.visual);
        const float *w = mp.world;
        p.origin = local(V3{w[12], w[13], w[14]} - t);
        for (int j = 0; j < 3; ++j) p.axes[j] = local({w[4*j], w[4*j+1], w[4*j+2]});
        if (cached.root == root) {
            for (const auto &old : cached.poses) {
                if (old.visual != p.visual || old.mesh != p.mesh || old.vertices != p.vertices ||
                    old.triangles != p.triangles || old.wheel != p.wheel || old.nativeWheel != p.nativeWheel) continue;
                if (Len(p.origin - old.origin) > (p.wheel ? .04f : .005f)) break;
                // Tyre spin is supplied by WheelContact. Keep its rolling hull stable.
                bool aligned = true;
                if (!p.wheel) for (int j = 0; j < 3; ++j) aligned &= Len(p.axes[j] - old.axes[j]) <= .005f;
                if (aligned) reused[i] = &old;
                break;
            }
        }
        if (!reused[i]) changed = true;
    }
    if (!changed) {
        cached.refreshAt = now + (n ? 80 : 500);
        return cached.parts.empty() ? nullptr : &cached;
    }
    CarBody body;
    body.root = root;
    body.version = cached.version + 1;
    body.refreshAt = now + (n ? 80 : 500);
    for (int i = 0; i < n; ++i) {
        PiecePose &p = poses[i];
        const auto *old = reused[i];
        const MmlMeshPiece &mp = g_pieces[i];
        if (old) p = *old; // Keep the reference pose for the movement threshold.
        p.first = body.all.size();
        if (old) {
            body.all.insert(body.all.end(), cached.all.begin() + old->first, cached.all.begin() + old->first + old->count);
        } else {
            for (int j = 0; j < mp.triangles * 3; ++j) {
                const auto v = mp.index[j];
                if (v >= (uint32_t)mp.vertices) continue;
                const float *q = mp.pos + 3*v;
                body.all.push_back(p.origin + p.axes[0]*q[0] + p.axes[1]*q[1] + p.axes[2]*q[2]);
            }
        }
        p.count = body.all.size() - p.first;
        p.part = -1;
        if (old && old->part < 0) continue;
        std::vector<V3> points(body.all.begin() + p.first, body.all.end());
        if (!old && Unfit(points)) continue;
        p.part = (int)body.parts.size();
        body.parts.push_back(std::move(points));
        body.wheel.push_back(p.wheel);
        body.nativeWheels.push_back(p.nativeWheel);
        body.revisions.push_back(old ? cached.revisions[old->part] : ++g_nextHull);
    }
    body.poses = std::move(poses);
    cached = std::move(body);
    return cached.parts.empty() ? nullptr : &cached;
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
        V3 t, ax[3];
        if (!Pose(car, &t, ax)) continue;
        if (t.x < lo.x || t.y < lo.y || t.z < lo.z || t.x > hi.x || t.y > hi.y || t.z > hi.z) continue;
        smoke::Mover m;
        if (MoverOf(car, &m)) out.push_back(m);
    }
}

bool MoverOf(void *car, smoke::Mover *out) {
    const CarBody *body = Body(car);
    if (!body) return false;
    V3 t, ax[3];
    if (!Pose(car, &t, ax)) return false;
    smoke::Mover m;
    m.key = car;
    m.kind = smoke::Mover::HULL;
    m.c = t;
    for (int i = 0; i < 3; i++) m.ax[i] = ax[i];
    m.hull = &body->all;
    m.hullParts = &body->parts;
    m.nativeWheels = &body->nativeWheels;
    m.hullRevisions = &body->revisions;
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
    *out = m;
    return true;
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

std::string TriangleVisual(void *car, unsigned version, unsigned triangle) {
    const auto it = g_bodies.find(car);
    if (it == g_bodies.end() || version != it->second.version) return {};
    const size_t vertex = size_t(triangle) * 3;
    for (const auto &piece : it->second.poses) {
        if (vertex < piece.first || vertex >= piece.first + piece.count) continue;
        std::string name;
        auto read = [&] {
            const char *s = game::at<char *>(piece.visual, game::kFrameName);
            if (s) name.assign(s, strnlen(s, 160));
        };
        Guarded(read);
        return name;
    }
    return {};
}

bool ModelTriangles(void *car, const std::vector<V3> **tris) {
    const CarBody *b = Body(car);
    if (!b) return false;
    *tris = &b->all;
    return true;
}

namespace {
float SurfaceBelow(const std::vector<V3> &triangles, V3 point, float minY, float maxY) {
    float result = minY;
    for (size_t i = 0; i + 2 < triangles.size(); i += 3) {
        const V3 &a = triangles[i];
        const V3 e = triangles[i + 1] - a, f = triangles[i + 2] - a, d = point - a;
        const float det = e.x * f.z - e.z * f.x;
        if (std::abs(det) < 1e-6f) continue;
        const float u = (d.x * f.z - d.z * f.x) / det;
        const float v = (e.x * d.z - e.z * d.x) / det;
        if (u < 0 || v < 0 || u + v > 1) continue;
        const float y = a.y + e.y * u + f.y * v;
        if (y <= maxY) result = std::max(result, y);
    }
    return result;
}
}

int OpenSeat(void *car, const V3 &localPelvis, float *support) {
    int n = 0, result = -1;
    const auto *slots = game::CarSeats(car, &n);
    float closest = .30f * .30f;
    for (int i = 0; i < n; ++i) {
        if (!slots[i].usable || slots[i].owner) continue;
        const V3 d = localPelvis - slots[i].origin;
        const float distance = d.x * d.x + d.z * d.z;
        if (distance >= closest || d.y < .25f || d.y > 2.4f) continue;
        closest = distance;
        result = i;
    }
    if (result < 0) return -1;
    const std::vector<V3> *mesh;
    if (!ModelTriangles(car, &mesh)) return -1;
    const V3 base = slots[result].origin;
    const float floor = SurfaceBelow(*mesh, localPelvis, base.y + .25f, base.y + .97f);
    if (floor <= base.y + .25f ||
        SurfaceBelow(*mesh, localPelvis, floor + .62f, floor + 2.f) > floor + .62f) return -1;
    *support = floor;
    return result;
}

bool HullParts(void *car, const std::vector<std::vector<V3>> **parts, unsigned *version) {
    const CarBody *b = Body(car);
    if (!b || b->parts.empty()) return false;
    *parts = &b->parts;
    *version = b->version;
    return true;
}

} // namespace vehicles
