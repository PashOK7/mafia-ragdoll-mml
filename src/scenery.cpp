// The static world from the loader's scene pieces (scenery.h).
//
// A piece (MmlMeshPiece) is one visual's mesh in its vertex space with a matrix whose rows are the world axes with
// scale. Each becomes a SceneryPiece: its pose a proper rotation (the rows made orthonormal, right-handed) and its
// position, its shape the mesh under the rest of the matrix (scale and any mirroring), cooked once a mesh and a scale
// (the meshes are shared by every instance of a model). As mafia-ragdoll's scenery made its shapes: a triangle faces
// the side its vertex normals point to, a two-sided material gives both windings, cut-out (colour key) and additive
// materials give none, and the water's faces none (water is not ground: a body sinks in it). The loader gives the
// triangles' material flags, not their materials: a water face is a level triangle at the height of the game's water
// under it (world_water: the river's 54 triangles at y -9.99, the harbour, the sea), asked once a shape.
#include "scenery.h"
#include <windows.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <unordered_map>
#include "mod.h"

using namespace mod;

namespace scenery {
namespace {

constexpr uint32_t kDoubleSided = 0x10000000, kColorKey = 0x20000000, kAdditive = 0x80000000;
constexpr unsigned kKeepSets = 120;   // a shape no piece used for 120 sets (30 s) goes

struct Shape {
    std::vector<V3> pos;
    std::vector<uint32_t> tris;
    unsigned used = 0;
};
std::map<std::pair<const float *, std::array<int, 9>>, std::unique_ptr<Shape>> g_shapes;

struct Focus { bool on = false; V3 lo, hi; };
Focus g_focus[kConsumers];
std::vector<smoke::SceneryPiece> g_pieces;
std::vector<MmlMeshPiece> g_raw;
unsigned g_sets, g_epoch = 1, g_full;
bool g_first = true, g_sentFocus;
uint64_t g_lastSignature;
unsigned g_waterFaces;

// whether a level triangle lies on the game's water (its centre at the water's height there)
bool OnWater(const V3 &a, const V3 &b, const V3 &c) {
    V3 n = Cross(b - a, c - a);
    float l = Len(n);
    if (l < 1e-9f || std::fabs(n.y) < 0.995f * l) return false;
    V3 m = (a + b + c) * (1.0f / 3.0f);
    MmlWater w{};
    w.size = sizeof w;
    const float at[3] = {m.x, m.y + 0.5f, m.z};
    if (!H->world_water(M, at, &w)) return false;
    return std::fabs(w.height - m.y) < 0.05f;
}

// the pose: rows made a proper rotation; k: what stays of the matrix (scale, mirroring) applied to the vertices
void Split(const float *w, M3 *r, M3 *k) {
    M3 m{{{w[0], w[1], w[2]}, {w[4], w[5], w[6]}, {w[8], w[9], w[10]}}};
    V3 r0 = Norm(m.r[0], {1, 0, 0});
    V3 r1 = Norm(m.r[1] - r0 * Dot(m.r[1], r0), {0, 1, 0});
    V3 r2 = Cross(r0, r1);
    *r = M3{{r0, r1, r2}};
    *k = m * r->T();
}

Shape *ShapeFor(const MmlMeshPiece &p, const M3 &k, const M3 &r, const V3 &t) {
    std::array<int, 9> q;
    for (int i = 0; i < 3; i++) {
        q[i * 3] = (int)std::lround(k.r[i].x * 1e4f);
        q[i * 3 + 1] = (int)std::lround(k.r[i].y * 1e4f);
        q[i * 3 + 2] = (int)std::lround(k.r[i].z * 1e4f);
    }
    auto key = std::make_pair(p.pos, q);
    auto it = g_shapes.find(key);
    if (it != g_shapes.end()) { it->second->used = g_sets; return it->second.get(); }
    auto sh = std::make_unique<Shape>();
    sh->used = g_sets;
    sh->pos.resize(p.vertices);
    for (int i = 0; i < p.vertices; i++) sh->pos[i] = k.Apply({p.pos[i * 3], p.pos[i * 3 + 1], p.pos[i * 3 + 2]});
    bool mirrored = Dot(k.r[0], Cross(k.r[1], k.r[2])) < 0;
    auto world = [&](uint32_t i) { const V3 &v = sh->pos[i]; return t + r.r[0] * v.x + r.r[1] * v.y + r.r[2] * v.z; };
    for (int f = 0; f < p.triangles; f++) {
        uint32_t flags = p.material ? p.material[f] : 0;
        if (flags & (kColorKey | kAdditive)) continue;
        uint32_t a = p.index[f * 3], b = p.index[f * 3 + 1], c = p.index[f * 3 + 2];
        if (a >= (uint32_t)p.vertices || b >= (uint32_t)p.vertices || c >= (uint32_t)p.vertices) continue;
        V3 n = Cross(sh->pos[b] - sh->pos[a], sh->pos[c] - sh->pos[a]);
        if (Dot(n, n) < 1e-12f) continue;
        if (OnWater(world(a), world(b), world(c))) { g_waterFaces++; continue; }
        bool both = (flags & kDoubleSided) || !p.normal;
        if (p.normal) {
            V3 vn{p.normal[a * 3] + p.normal[b * 3] + p.normal[c * 3], p.normal[a * 3 + 1] + p.normal[b * 3 + 1] + p.normal[c * 3 + 1],
                  p.normal[a * 3 + 2] + p.normal[b * 3 + 2] + p.normal[c * 3 + 2]};
            // the vertex normals are in the mesh's space: the face's own normal there against them
            V3 raw = Cross(V3{p.pos[b * 3] - p.pos[a * 3], p.pos[b * 3 + 1] - p.pos[a * 3 + 1], p.pos[b * 3 + 2] - p.pos[a * 3 + 2]},
                           V3{p.pos[c * 3] - p.pos[a * 3], p.pos[c * 3 + 1] - p.pos[a * 3 + 1], p.pos[c * 3 + 2] - p.pos[a * 3 + 2]});
            if (Dot(raw, vn) < 0) std::swap(b, c);   // the front is the side it is seen from
        }
        if (mirrored) std::swap(b, c);
        sh->tris.insert(sh->tris.end(), {a, b, c});
        if (both) sh->tris.insert(sh->tris.end(), {a, c, b});
    }
    Shape *out = sh.get();
    g_shapes.emplace(key, std::move(sh));
    return out;
}

void SendFocus() {
    Focus u;
    for (const Focus &f : g_focus) {
        if (!f.on) continue;
        if (!u.on) { u = f; continue; }
        u.lo = {std::min(u.lo.x, f.lo.x), std::min(u.lo.y, f.lo.y), std::min(u.lo.z, f.lo.z)};
        u.hi = {std::max(u.hi.x, f.hi.x), std::max(u.hi.y, f.hi.y), std::max(u.hi.z, f.hi.z)};
    }
    if (u.on) {
        const float lo[3] = {u.lo.x, u.lo.y, u.lo.z}, hi[3] = {u.hi.x, u.hi.y, u.hi.z};
        H->scene_focus(M, lo, hi);
    } else {
        H->scene_focus(M, nullptr, nullptr);
    }
    g_sentFocus = u.on;
}

} // namespace

void SetFocus(Consumer c, const V3 &lo, const V3 &hi) {
    g_focus[c] = {true, lo, hi};
    SendFocus();
}
void ClearFocus(Consumer c) {
    g_focus[c].on = false;
    SendFocus();
}

void Tick() {
    if (!g_sentFocus) return;
    int n = H->scene_pieces(M, nullptr, 0);
    if (n < 0) return;
    g_raw.resize(n);
    if (n) n = H->scene_pieces(M, g_raw.data(), n);
    g_raw.resize(std::max(n, 0));
    // a new set only when the host's pieces changed (their visuals and versions)
    // (the host gives them in no fixed order: a sum of each one's mixed key, not a chain)
    uint64_t sig = g_raw.size() * 0x9E3779B97F4A7C15ull;
    for (const MmlMeshPiece &p : g_raw) {
        uint64_t k = (static_cast<uint64_t>(reinterpret_cast<uintptr_t>(p.visual)) << 32) ^ p.version ^ (p.moving ? 0x8000000000000000ull : 0);
        k ^= k >> 33;
        k *= 0xFF51AFD7ED558CCDull;
        k ^= k >> 33;
        sig += k;
    }
    if (sig == g_lastSignature && !g_pieces.empty()) {
        // the box covered whole now, with the same pieces: this set is a full one
        for (const Focus &f : g_focus)
            if (f.on && Covers(f.lo, f.hi) && g_full != g_sets) g_full = g_sets;
        return;
    }
    g_lastSignature = sig;
    g_sets++;
    g_pieces.clear();
    g_pieces.reserve(g_raw.size());
    for (const MmlMeshPiece &p : g_raw) {
        if (p.moving || p.triangles <= 0 || p.vertices <= 0) continue;   // (a door, a gate moving: not ground now)
        M3 r, k;
        Split(p.world, &r, &k);
        V3 t{p.world[12], p.world[13], p.world[14]};
        Shape *sh = ShapeFor(p, k, r, t);
        if (sh->tris.empty()) continue;
        smoke::SceneryPiece sp;
        sp.key = p.visual;
        sp.version = p.version;
        sp.shapeKey = sh;
        sp.t = t;
        sp.ax[0] = r.r[0];
        sp.ax[1] = r.r[1];
        sp.ax[2] = r.r[2];
        sp.pos = &sh->pos;
        sp.tris = &sh->tris;
        g_pieces.push_back(sp);
    }
    bool covered = false;
    for (const Focus &f : g_focus) covered = covered || (f.on && Covers(f.lo, f.hi));
    if (covered) g_full = g_sets;
    if (g_sets % 40 == 0)   // the shapes no piece used for a while go (the region moves on)
        for (auto it = g_shapes.begin(); it != g_shapes.end();) it = it->second->used + kKeepSets < g_sets ? g_shapes.erase(it) : std::next(it);
    if (g_first && !g_pieces.empty()) {
        Log("scenery: %zu pieces in the first set (%u water faces left out)", g_pieces.size(), g_waterFaces);
        g_first = false;
    }
}

const std::vector<smoke::SceneryPiece> &Pieces() { return g_pieces; }
unsigned Scans() { return g_sets; }
bool FirstScan() { return g_sets <= 1; }
unsigned Epoch() { return g_epoch; }
unsigned FullScan() { return g_full; }

bool Covers(const V3 &lo, const V3 &hi) {
    const float l[3] = {lo.x, lo.y, lo.z}, h[3] = {hi.x, hi.y, hi.z};
    return H->scene_ready(M, l, h) != 0;
}

std::string CoverReport(const V3 &lo, const V3 &hi) {
    char b[256];
    snprintf(b, sizeof b, "; scenery: set %u (full %u), %zu pieces, the loader says the box is %s", g_sets, g_full, g_pieces.size(),
             Covers(lo, hi) ? "ready" : "not ready yet");
    return b;
}

void Clear() {
    g_pieces.clear();
    g_raw.clear();
    g_shapes.clear();
    g_epoch++;
    g_sets = 0;
    g_full = 0;
    g_first = true;
    g_lastSignature = 0;
}

std::string Describe() {
    char b[256];
    snprintf(b, sizeof b, "scenery: %zu pieces (set %u, full %u, epoch %u), %zu shapes, %u water faces left out", g_pieces.size(), g_sets, g_full,
             g_epoch, g_shapes.size(), g_waterFaces);
    return b;
}

} // namespace scenery
