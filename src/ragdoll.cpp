#include "ragdoll.h"
#include "game.h"
#include "collision.h"
#include "mod.h"
#include <windows.h>
#include <cstdio>
#include <algorithm>

// The ragdoll's binding to the game: the skeleton read from the model's frames, the 15 body parts made from it
// (their masses, inertia and capsules), the blows that change their velocities, and the bones written back from the
// parts' poses. The parts move in Jolt (jolt_ragdoll). Math convention inside this file: column vectors,
// world = R(q) * local. The engine uses row vectors (rows = local axes in world); conversions are explicit.

namespace {

constexpr float kDeg = 0.0174533f;

using QuatMakeFn = void(__stdcall *)(void *quat, const float *matrix);
QuatMakeFn QuatMake() {
    static QuatMakeFn fn = reinterpret_cast<QuatMakeFn>(
        GetProcAddress(GetModuleHandleA("LS3DF.dll"), "?Make@S_quat@@QAGXABUS_matrix@@@Z"));
    return fn;
}

V3 Origin(void *frame) { const float *m = game::WorldMatrix(frame); return {m[12], m[13], m[14]}; }

// Row of a frame's world rotation that best matches `want` (sign-corrected), made perpendicular to `axis`.
V3 BestRow(void *frame, const V3 &want, const V3 &axis) {
    M3 r = M3::FromWorld(game::WorldMatrix(frame));
    V3 best = r.r[0]; float bestDot = 0;
    for (auto &row : r.r) {
        V3 v = Norm(row - axis * Dot(row, axis));
        float d = Dot(v, want);
        if (std::fabs(d) > std::fabs(bestDot)) { bestDot = d; best = v; }
    }
    return bestDot < 0 ? -best : best;
}

float Clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

} // namespace


// ------------------------------------------------------------------ quaternion helpers

V3 Ragdoll::Rotate(const Quat &q, const V3 &v) {
    V3 u{q.x, q.y, q.z};
    V3 t = Cross(u, v) * 2.0f;
    return v + t * q.w + Cross(u, t);
}

V3 Ragdoll::InvRotate(const Quat &q, const V3 &v) {
    Quat c{q.w, -q.x, -q.y, -q.z};
    return Rotate(c, v);
}

Ragdoll::Quat Ragdoll::Mul(const Quat &a, const Quat &b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

void Ragdoll::Normalize(Quat &q) {
    float l = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (l < 1e-9f) { q = Quat{}; return; }
    q.w /= l; q.x /= l; q.y /= l; q.z /= l;
}

void Ragdoll::AddRotation(Quat &q, const V3 &w) {
    Quat d = Mul(Quat{0, w.x, w.y, w.z}, q);
    q.w += 0.5f * d.w; q.x += 0.5f * d.x; q.y += 0.5f * d.y; q.z += 0.5f * d.z;
    Normalize(q);
}

Ragdoll::Quat Ragdoll::FromAxes(const V3 &c0, const V3 &c1, const V3 &c2) {
    float m00 = c0.x, m01 = c1.x, m02 = c2.x;
    float m10 = c0.y, m11 = c1.y, m12 = c2.y;
    float m20 = c0.z, m21 = c1.z, m22 = c2.z;
    Quat q;
    float tr = m00 + m11 + m22;
    if (tr > 0) {
        float s = 0.5f / std::sqrt(tr + 1.0f);
        q = {0.25f / s, (m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s};
    } else if (m00 > m11 && m00 > m22) {
        float s = 2.0f * std::sqrt(1.0f + m00 - m11 - m22);
        q = {(m21 - m12) / s, 0.25f * s, (m01 + m10) / s, (m02 + m20) / s};
    } else if (m11 > m22) {
        float s = 2.0f * std::sqrt(1.0f + m11 - m00 - m22);
        q = {(m02 - m20) / s, (m01 + m10) / s, 0.25f * s, (m12 + m21) / s};
    } else {
        float s = 2.0f * std::sqrt(1.0f + m22 - m00 - m11);
        q = {(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25f * s};
    }
    Normalize(q);
    return q;
}

// ------------------------------------------------------------------ inertia (the blows' spin)

V3 Ragdoll::InvInertiaWorld(const Body &bd, const V3 &v) const {
    V3 l = InvRotate(bd.q, v);
    l = {l.x * bd.invInertia.x, l.y * bd.invInertia.y, l.z * bd.invInertia.z};
    return Rotate(bd.q, l);
}

// ------------------------------------------------------------------ construction

void Ragdoll::MakeBody(int id, const V3 &a, const V3 &b, const V3 &side, float radius, float mass) {
    Body &bd = body[id];
    V3 x = Norm(b - a, {1, 0, 0});
    V3 z = side - x * Dot(side, x);
    if (Len(z) < 1e-3f) z = Cross(x, std::fabs(x.y) < 0.9f ? V3{0, 1, 0} : V3{1, 0, 0});
    z = Norm(z);
    V3 y = Cross(z, x);
    bd.q = FromAxes(x, y, z);
    bd.qPrev = bd.q;
    bd.x = (a + b) * 0.5f;
    bd.xPrev = bd.x;
    bd.v = bd.w = {0, 0, 0};
    bd.a = InvRotate(bd.q, a - bd.x);
    bd.b = InvRotate(bd.q, b - bd.x);
    bd.radius = radius;
    bd.invMass = 1.0f / mass;
    float L = Len(b - a);
    float ip = mass * (3 * radius * radius + L * L) / 12.0f;
    // Thin limbs have almost no inertia about their own axis; any small torque would spin them up
    // to absurd rates. Keep the axial inertia within 2.5x of the transverse one (common ragdoll practice).
    float ia = std::max(0.5f * mass * radius * radius, 0.4f * ip);
    bd.invInertia = {1.0f / std::max(ia, 1e-4f), 1.0f / std::max(ip, 1e-4f), 1.0f / std::max(ip, 1e-4f)};
}

void Ragdoll::SetCapsule(int id, const V3 &a, const V3 &b, float radius) {
    Body &bd = body[id];
    bd.a = InvRotate(bd.q, a - bd.x);
    bd.b = InvRotate(bd.q, b - bd.x);
    bd.radius = radius;
    // Inertia of the new capsule about the body axes (its direction in the body frame weights the terms).
    float m = 1.0f / bd.invMass, L = Len(b - a);
    float ip = m * (3 * radius * radius + L * L) / 12.0f, ia = std::max(0.5f * m * radius * radius, 0.4f * ip);
    V3 d = Norm(bd.b - bd.a, {1, 0, 0});
    V3 I{ip + (ia - ip) * d.x * d.x, ip + (ia - ip) * d.y * d.y, ip + (ia - ip) * d.z * d.z};
    bd.invInertia = {1.0f / I.x, 1.0f / I.y, 1.0f / I.z};
}

void Ragdoll::Setup(const V3 *worldPts, const V3 &fwd) {
    // Simulate around a local origin: world coordinates in Lost Heaven reach ~2000 m, where a float
    // step is ~0.1 mm and velocities derived as dx/h at 600 Hz would carry ~0.1 m/s of rounding noise.
    origin = worldPts[PELVIS];
    V3 lp[COUNT];
    for (int i = 0; i < COUNT; i++) { initPos[i] = worldPts[i]; lp[i] = worldPts[i] - origin; }
    const V3 *pts = lp;
    initFwd = fwd;

    // Anatomical frames (x right, y up, z forward).
    V3 pr = Norm(pts[RHIP] - pts[LHIP], {1, 0, 0});
    V3 pf = Norm(Cross(pr, pts[BACK1] - pts[PELVIS]), fwd);
    if (Dot(pf, fwd) < 0) pf = -pf;
    V3 cr = Norm(pts[RCLAV] - pts[LCLAV], pr);
    V3 cf = Norm(Cross(cr, pts[NECK] - pts[BACK3]), fwd);
    if (Dot(cf, fwd) < 0) cf = -cf;

    MakeBody(B_PELVIS, pts[LHIP], pts[RHIP], pf, 0.12f, 11.0f);
    MakeBody(B_ABDOMEN, pts[BACK1], pts[BACK2], pf, 0.12f, 7.0f);
    MakeBody(B_CHESTLOW, pts[BACK2], pts[BACK3], cf, 0.13f, 7.0f);
    MakeBody(B_CHEST, pts[BACK3], pts[NECK], cf, 0.15f, 14.0f);
    {   // the ribcage is wide: collide with a capsule from shoulder to shoulder at chest height
        V3 mid = pts[BACK3] * 0.4f + pts[NECK] * 0.6f;
        SetCapsule(B_CHEST, mid - cr * 0.10f, mid + cr * 0.10f, 0.14f);
    }
    MakeBody(B_HEAD, pts[NECK], pts[HEAD], Norm(pts[NOSE] - pts[NECK]), 0.10f, 5.0f);
    MakeBody(B_RUARM, pts[RARM], pts[RELB], cf, 0.055f, 2.2f);
    MakeBody(B_RFARM, pts[RELB], pts[RTIP], pts[RTHUMB] - pts[RHAND], 0.045f, 1.8f);
    MakeBody(B_LUARM, pts[LARM], pts[LELB], cf, 0.055f, 2.2f);
    MakeBody(B_LFARM, pts[LELB], pts[LTIP], pts[LTHUMB] - pts[LHAND], 0.045f, 1.8f);
    MakeBody(B_RTHIGH, pts[RHIP], pts[RKNEE], pf, 0.075f, 8.0f);
    MakeBody(B_RSHIN, pts[RKNEE], pts[RANKLE], pf, 0.06f, 3.5f);
    MakeBody(B_RFOOT, pts[RANKLE], pts[RTOE], pts[RKNEE] - pts[RANKLE], 0.045f, 1.2f);
    MakeBody(B_LTHIGH, pts[LHIP], pts[LKNEE], pf, 0.075f, 8.0f);
    MakeBody(B_LSHIN, pts[LKNEE], pts[LANKLE], pf, 0.06f, 3.5f);
    MakeBody(B_LFOOT, pts[LANKLE], pts[LTOE], pts[LKNEE] - pts[LANKLE], 0.045f, 1.2f);
}

bool Ragdoll::Capture(void *model, const V3 *dirIn, V3 *pts, V3 *fwdOut, void **frames, std::string *err) {
    if (!model) { *err = "no model frame"; return false; }
    static const char *names[] = {"r_thigh", "r_shin", "r_foot", "l_thigh", "l_shin", "l_foot", "back1", "back2", "back3",
                                  "neck", "r_shoulder", "r_arm", "r_elbow", "r_hand", "l_shoulder", "l_arm", "l_elbow", "l_hand"};
    void **f = frames;
    for (int i = 0; i < 18; i++) {
        f[i] = game::FindChild(model, names[i]);
        if (!f[i]) { *err = std::string("joint not found: ") + names[i]; return false; }
    }
    enum { RTH, RSH, RFT, LTH, LSH, LFT, B1, B2, B3, NK, RCL, RAR, REL, RHA, LCL, LAR, LEL, LHA };
    void *pelvis = game::at<void *>(f[RTH], game::kFrameParent);
    if (!pelvis) { *err = "pelvis frame missing"; return false; }
    void *root = game::at<void *>(pelvis, game::kFrameParent);
    if (!root) { *err = "root frame missing"; return false; }
    f[18] = pelvis;
    f[19] = root;

    V3 dir;
    if (dirIn) dir = *dirIn;
    else { const float *m = game::WorldMatrix(model); dir = {m[8], m[9], m[10]}; }
    dir.y = 0;
    V3 fwd = Norm(dir, {0, 0, 1});
    *fwdOut = fwd;
    PointsFromFrames(frames, fwd, pts);
    return true;
}

void Ragdoll::PointsFromFrames(void *const *f, const V3 &fwd, V3 *pts) {
    enum { RTH, RSH, RFT, LTH, LSH, LFT, B1, B2, B3, NK, RCL, RAR, REL, RHA, LCL, LAR, LEL, LHA };
    void *pelvis = f[18];
    pts[PELVIS] = Origin(pelvis);
    pts[BACK1] = Origin(f[B1]);
    pts[BACK2] = Origin(f[B2]);
    pts[BACK3] = Origin(f[B3]);
    pts[NECK] = Origin(f[NK]);
    V3 headDir = Norm(pts[NECK] - pts[BACK3]);
    pts[HEAD] = pts[NECK] + headDir * 0.20f;
    pts[NOSE] = pts[NECK] + headDir * 0.10f + fwd * 0.10f;
    auto arm = [&](int clav, int armP, int elb, int hand, int tip, int thumb, void *fc, void *fa, void *fe, void *fh) {
        pts[clav] = Origin(fc); pts[armP] = Origin(fa); pts[elb] = Origin(fe); pts[hand] = Origin(fh);
        V3 fore = Norm(pts[hand] - pts[elb]);
        pts[tip] = pts[hand] + fore * 0.10f;
        pts[thumb] = pts[hand] + BestRow(fh, fwd, fore) * 0.05f;
    };
    arm(RCLAV, RARM, RELB, RHAND, RTIP, RTHUMB, f[RCL], f[RAR], f[REL], f[RHA]);
    arm(LCLAV, LARM, LELB, LHAND, LTIP, LTHUMB, f[LCL], f[LAR], f[LEL], f[LHA]);
    auto leg = [&](int hip, int knee, int ankle, int toe, void *ft, void *fs, void *ff) {
        pts[hip] = Origin(ft); pts[knee] = Origin(fs); pts[ankle] = Origin(ff);
        V3 shin = Norm(pts[ankle] - pts[knee]);
        pts[toe] = pts[ankle] + BestRow(ff, fwd, shin) * 0.14f + V3{0, -0.04f, 0};
    };
    leg(RHIP, RKNEE, RANKLE, RTOE, f[RTH], f[RSH], f[RFT]);
    leg(LHIP, LKNEE, LANKLE, LTOE, f[LTH], f[LSH], f[LFT]);
}

bool Ragdoll::SnapshotNow(PoseSnap *out) const {
    if (!frames[18] || !frames[19]) return false;
    // The facing: the physical pelvis's forward, level (the helper points and the body frames' signs need it).
    V3 fwd = Rotate(body[B_PELVIS].q, {0, 0, 1});
    fwd.y = 0;
    fwd = Norm(fwd, initFwd);
    for (int i = 0; i < 20; i++) out->frames[i] = frames[i];
    out->fwd = fwd;
    PointsFromFrames(frames, fwd, out->pts);
    for (int i = 0; i < 20; i++) {
        out->rot[i] = M3::FromWorld(game::WorldMatrix(frames[i]));
        out->org[i] = Origin(frames[i]);
    }
    return true;
}

// Which skeleton points span each body's capsule (same as in Setup).
static const int kBodyPoints[Ragdoll::BCOUNT][2] = {
    {Ragdoll::LHIP, Ragdoll::RHIP}, {Ragdoll::BACK1, Ragdoll::BACK2}, {Ragdoll::BACK2, Ragdoll::BACK3}, {Ragdoll::BACK3, Ragdoll::NECK},
    {Ragdoll::NECK, Ragdoll::HEAD}, {Ragdoll::RARM, Ragdoll::RELB}, {Ragdoll::RELB, Ragdoll::RTIP}, {Ragdoll::LARM, Ragdoll::LELB},
    {Ragdoll::LELB, Ragdoll::LTIP}, {Ragdoll::RHIP, Ragdoll::RKNEE}, {Ragdoll::RKNEE, Ragdoll::RANKLE}, {Ragdoll::RANKLE, Ragdoll::RTOE},
    {Ragdoll::LHIP, Ragdoll::LKNEE}, {Ragdoll::LKNEE, Ragdoll::LANKLE}, {Ragdoll::LANKLE, Ragdoll::LTOE}};

// Gives every body the motion the animation had: linear velocity from the move of its capsule centre and
// angular velocity from the turn of its axis, between an earlier pose and the start pose.
void Ragdoll::SetMotionFrom(const V3 *prevWorld, float dt) {
    if (!prevWorld || dt <= 1e-4f) return;
    for (int b = 0; b < BCOUNT; b++) {
        int i = kBodyPoints[b][0], j = kBodyPoints[b][1];
        V3 c0 = (prevWorld[i] + prevWorld[j]) * 0.5f, c1 = (initPos[i] + initPos[j]) * 0.5f;
        V3 v = (c1 - c0) * (1.0f / dt);
        V3 a0 = Norm(prevWorld[j] - prevWorld[i]), a1 = Norm(initPos[j] - initPos[i]);
        V3 axis = Cross(a0, a1);
        float s = Len(axis), c = Dot(a0, a1);
        V3 w = s > 1e-6f ? axis * (std::atan2(s, c) / s / dt) : V3{0, 0, 0};
        // A pose snap (animation cut, teleport) must not fling the body.
        float vl = Len(v), wl = Len(w);
        if (vl > 12.0f) v = v * (12.0f / vl);
        if (wl > 20.0f) w = w * (20.0f / wl);
        body[b].v = v;
        body[b].w = w;
    }
}

bool Ragdoll::Snapshot(void *model, const V3 *dir, PoseSnap *out, std::string *err) {
    if (!Capture(model, dir, out->pts, &out->fwd, out->frames, err)) return false;
    for (int i = 0; i < 20; i++) {
        out->rot[i] = M3::FromWorld(game::WorldMatrix(out->frames[i]));
        out->org[i] = Origin(out->frames[i]);
    }
    return true;
}

V3 Ragdoll::ClosestToLine(const V3 &a, const V3 &b) const {
    float best = 1e30f; V3 at = origin + body[B_CHEST].x;
    for (int i = 0; i < BCOUNT; i++) {
        float s, t; V3 c1, c2;
        float d = ClosestSegSeg(AxisPoint(i, 0) + origin, AxisPoint(i, 1) + origin, a, b, &s, &t, &c1, &c2);
        if (d < best) { best = d; at = c1; }
    }
    return at;
}

bool Ragdoll::Init(void *key, void *mdl, const V3 *dir, std::string *err, const V3 *prevPts, float prevDt, const PoseSnap *start) {
    actor = key;
    model = mdl;
    if (!QuatMake()) { *err = "S_quat::Make not found in LS3DF.dll"; return false; }
    PoseSnap now;
    if (!start) {
        if (!Snapshot(model, dir, &now, err)) return false;
        start = &now;
    }
    const PoseSnap &ps = *start;
    void *const *f = ps.frames;
    enum { RTH, RSH, RFT, LTH, LSH, LFT, B1, B2, B3, NK, RCL, RAR, REL, RHA, LCL, LAR, LEL, LHA, PEL, ROOT };
    pelvisFrame = f[PEL];
    rootFrame = f[ROOT];
    for (int i = 0; i < 20; i++) frames[i] = f[i];
    Setup(ps.pts, ps.fwd);
    SetMotionFrom(prevPts, prevDt);

    // Bone links, parent-first. Clavicles ride on the chest, hands on the forearms. The binding of each bone
    // to its body comes from the same snapshot as the bodies themselves.
    struct { int fi; int b; const char *name; } links[] = {
        {PEL, B_PELVIS, "pelvis"}, {B1, B_ABDOMEN, "back1"}, {B2, B_CHESTLOW, "back2"}, {B3, B_CHEST, "back3"},
        {NK, B_HEAD, "neck"}, {RCL, B_CHEST, "r_shoulder"}, {RAR, B_RUARM, "r_arm"}, {REL, B_RFARM, "r_elbow"},
        {RHA, B_RFARM, "r_hand"}, {LCL, B_CHEST, "l_shoulder"}, {LAR, B_LUARM, "l_arm"}, {LEL, B_LFARM, "l_elbow"},
        {LHA, B_LFARM, "l_hand"}, {RTH, B_RTHIGH, "r_thigh"}, {RSH, B_RSHIN, "r_shin"}, {RFT, B_RFOOT, "r_foot"},
        {LTH, B_LTHIGH, "l_thigh"}, {LSH, B_LSHIN, "l_shin"}, {LFT, B_LFOOT, "l_foot"}};
    bones.clear();
    for (auto &l : links) {
        const Body &bd = body[l.b];
        M3 rows{{Rotate(bd.q, {1, 0, 0}), Rotate(bd.q, {0, 1, 0}), Rotate(bd.q, {0, 0, 1})}};
        V3 originLocal = InvRotate(bd.q, (ps.org[l.fi] - origin) - bd.x);
        bones.push_back({f[l.fi], l.b, ps.rot[l.fi] * rows.T(), originLocal, l.name});
    }
    pelvisOffset = InvRotate(body[B_PELVIS].q, (ps.pts[PELVIS] - origin) - body[B_PELVIS].x);
    return true;
}

// ------------------------------------------------------------------ simulation

V3 Ragdoll::AxisPoint(int b, float t) const {
    const Body &bd = body[b];
    return bd.x + Rotate(bd.q, bd.a + (bd.b - bd.a) * t);
}

V3 Ragdoll::Center() const { return body[B_CHEST].x + origin; }

void Ragdoll::Impulse(const V3 &dir, const V3 &at, float speed, float radius, float wholeBody) {
    V3 d = Norm(dir);
    for (auto &bd : body) {
        float w = wholeBody;
        float r = Len(bd.x + origin - at);
        if (radius > 0 && r < radius) w += 1.0f - r / radius;
        bd.v += d * (speed * w);
    }
    sleeping = false; sleepTimer = 0;
    velocityEdited = true;
}

void Ragdoll::Kick(const V3 &J, const V3 &at, float whole, float radius, float maxPart) {
    float M = 0;
    for (const Body &bd : body) if (bd.invMass > 0) M += 1.0f / bd.invMass;
    if (M <= 0) return;
    float w[BCOUNT] = {}, wsum = 0, nearestD = 1e30f;
    int nearest = 0;
    for (int b = 0; b < BCOUNT; b++) {
        if (body[b].invMass <= 0) continue;
        V3 a, e;
        float r;
        BodyCapsule(b, &a, &e, &r);
        V3 ae = e - a;
        float l2 = Dot(ae, ae);
        float t = l2 > 1e-8f ? std::min(1.0f, std::max(0.0f, Dot(at - a, ae) / l2)) : 0.0f;
        float d = std::max(0.0f, Len(a + ae * t - at) - r);
        if (d < nearestD) { nearestD = d; nearest = b; }
        if (radius > 0 && d < radius) { w[b] = (1.0f - d / radius) / body[b].invMass; wsum += w[b]; }
    }
    if (wsum <= 0) { w[nearest] = 1; wsum = 1; }
    if (kickAge != age) {   // the first blow of this tick
        for (int b = 0; b < BCOUNT; b++) kickV[b] = kickW[b] = V3{0, 0, 0};
        kickAge = age;
    }
    V3 local = J * (1.0f - whole), spill{0, 0, 0};
    const float maxSpin = 2.0f * maxPart;
    for (int b = 0; b < BCOUNT; b++) {
        if (w[b] <= 0) continue;
        Body &bd = body[b];
        V3 Jb = local * (w[b] / wsum);
        // held to what is left of this tick's allowance for the part
        V3 total = kickV[b] + Jb * bd.invMass;
        float s = Len(total);
        if (s > maxPart) {
            V3 allowed = total * (maxPart / s);
            V3 dvKept = allowed - kickV[b];
            spill = spill + (Jb - dvKept * (1.0f / bd.invMass));
            Jb = dvKept * (1.0f / bd.invMass);
        }
        V3 dv = Jb * bd.invMass;
        kickV[b] = kickV[b] + dv;
        bd.v = bd.v + dv;
        // the spin of an off-centre blow, held the same way (a film's blow on a 2 kg forearm would spin it at
        // hundreds of rad/s); what is cut is spin the joints could not have passed on
        V3 spin = kickW[b] + InvInertiaWorld(bd, Cross(at - (origin + bd.x), Jb));
        float sw = Len(spin);
        if (sw > maxSpin) spin = spin * (maxSpin / sw);
        bd.w = bd.w + (spin - kickW[b]);
        kickW[b] = spin;
    }
    V3 dvAll = (J * whole + spill) * (1.0f / M);
    for (Body &bd : body) bd.v = bd.v + dvAll;
    sleeping = false; sleepTimer = 0;
    velocityEdited = true;
}


// ------------------------------------------------------------------ output

void Ragdoll::Apply() {
    QuatMakeFn make = QuatMake();
    // The model root follows the body (under the pelvis, on the lowest point of the body): the game hangs the
    // shadow, visibility culling and its own idea of where the body is on the root, not on the bones.
    {
        V3 pel = body[B_PELVIS].x;
        float low = pel.y;
        for (int b = 0; b < BCOUNT; b++) low = std::min(low, body[b].x.y - body[b].radius);
        V3 want = origin + V3{pel.x, low, pel.z};
        void *parent = game::at<void *>(rootFrame, game::kFrameParent);
        V3 local = want;
        if (parent) {
            const float *pw = game::WorldMatrix(parent);
            local = M3::Raw(pw).Inverse().Apply(want - V3{pw[12], pw[13], pw[14]});
        }
        const float lp[3] = {local.x, local.y, local.z};
        mod::H->bone_set(mod::M, rootFrame, nullptr, lp);
        rootPos = want;
    }
    const float *rootW = game::WorldMatrix(rootFrame);
    M3 rootRot = M3::FromWorld(rootW);
    std::vector<std::pair<void *, M3>> world;
    world.reserve(bones.size());
    for (auto &l : bones) {
        const Body &bd = body[l.bodyIdx];
        M3 rows{{Rotate(bd.q, {1, 0, 0}), Rotate(bd.q, {0, 1, 0}), Rotate(bd.q, {0, 0, 1})}};
        M3 w = l.offset * rows;
        w.Orthonormalize();
        void *parent = game::at<void *>(l.frame, game::kFrameParent);
        M3 parentW = rootRot;
        for (auto &e : world) if (e.first == parent) { parentW = e.second; break; }
        M3 local = w * parentW.T();
        float m[16] = {local.r[0].x, local.r[0].y, local.r[0].z, 0, local.r[1].x, local.r[1].y, local.r[1].z, 0,
                       local.r[2].x, local.r[2].y, local.r[2].z, 0, 0, 0, 0, 1};
        Q q;
        make(&q, m);
        const float lq[4] = {q.w, q.x, q.y, q.z};
        mod::H->bone_set(mod::M, l.frame, lq, nullptr);
        const float *wq = &game::at<float>(l.frame, game::kFrameRot);
        for (int k = 0; k < 4; k++) const_cast<BoneLink &>(l).written[k] = wq[k];
        const_cast<BoneLink &>(l).wrote = true;
        world.push_back({l.frame, w});
        if (l.frame == pelvisFrame) {
            // Subtract the two large numbers first so the small local offsets keep their precision.
            V3 rel = (origin - V3{rootW[12], rootW[13], rootW[14]}) + (bd.x + Rotate(bd.q, pelvisOffset));
            V3 lp = M3::Raw(rootW).Inverse().Apply(rel);
            const float pp[3] = {lp.x, lp.y, lp.z};
            mod::H->bone_set(mod::M, l.frame, nullptr, pp);
        }
    }
    // (the bones are written through the loader's bone_set: it updates the model once after RENDER_BEGIN's mods, as
    // LS3D's Update re-skins the mesh from its joints, and keeps the pose drawn for human_get_up)
}

void Ragdoll::PhysicalPose(M3 *rot, V3 *org) const {
    for (int i = 0; i < 19; i++) { rot[i] = M3::Identity(); org[i] = origin; }
    for (const BoneLink &l : bones) {
        int fi = -1;
        for (int i = 0; i < 19; i++) if (frames[i] == l.frame) { fi = i; break; }
        if (fi < 0) continue;
        const Body &bd = body[l.bodyIdx];
        M3 rows{{Rotate(bd.q, {1, 0, 0}), Rotate(bd.q, {0, 1, 0}), Rotate(bd.q, {0, 0, 1})}};
        rot[fi] = l.offset * rows;
        rot[fi].Orthonormalize();
        org[fi] = origin + (bd.x + Rotate(bd.q, l.originLocal));
    }
}

void Ragdoll::WritePose(void *model, void *const *f, const M3 *rot, const V3 &pelvis) {
    QuatMakeFn make = QuatMake();
    void *root = f[19];
    const float *rootW = game::WorldMatrix(root);
    M3 rootRot = M3::FromWorld(rootW);
    for (int i = 0; i < 19; i++) {
        void *parent = game::at<void *>(f[i], game::kFrameParent);
        M3 parentW = rootRot;
        bool known = parent == root;
        for (int j = 0; j < 19 && !known; j++) if (f[j] == parent) { parentW = rot[j]; known = true; }
        if (!known && parent) parentW = M3::FromWorld(game::WorldMatrix(parent));
        M3 local = rot[i] * parentW.T();
        float m[16] = {local.r[0].x, local.r[0].y, local.r[0].z, 0, local.r[1].x, local.r[1].y, local.r[1].z, 0,
                       local.r[2].x, local.r[2].y, local.r[2].z, 0, 0, 0, 0, 1};
        Q q;
        make(&q, m);
        game::SetLocalRotation(f[i], q);
    }
    game::SetLocalPosition(f[18], M3::Raw(rootW).Inverse().Apply(pelvis - V3{rootW[12], rootW[13], rootW[14]}));
    using UpdateFn = void(__stdcall *)(void *);
    reinterpret_cast<UpdateFn>((*reinterpret_cast<void ***>(model))[0x18 / 4])(model);
}

void Ragdoll::PlaceFrame(void *frame, const M3 &rot, const V3 &pos) {
    M3 local = rot;
    V3 p = pos;
    if (void *parent = game::at<void *>(frame, game::kFrameParent)) {
        const float *pw = game::WorldMatrix(parent);
        local = rot * M3::FromWorld(pw).T();
        p = M3::Raw(pw).Inverse().Apply(pos - V3{pw[12], pw[13], pw[14]});
    }
    float m[16] = {local.r[0].x, local.r[0].y, local.r[0].z, 0, local.r[1].x, local.r[1].y, local.r[1].z, 0,
                   local.r[2].x, local.r[2].y, local.r[2].z, 0, 0, 0, 0, 1};
    Q q;
    QuatMake()(&q, m);
    game::SetLocalRotation(frame, q);
    game::SetLocalPosition(frame, p);
}

std::string Ragdoll::Describe() const {
    char buf[256];
    std::string s;
    snprintf(buf, sizeof buf, "{\"sleeping\":%s,\"age\":%.2f,\"puppet\":%s,\"bodies\":[", sleeping ? "true" : "false", age,
             puppet ? "true" : "false");
    s += buf;
    for (int i = 0; i < BCOUNT; i++) {
        snprintf(buf, sizeof buf, "%s[%.3f,%.3f,%.3f]", i ? "," : "", body[i].x.x + origin.x, body[i].x.y + origin.y, body[i].x.z + origin.z);
        s += buf;
    }
    s += "]}";
    return s;
}

std::string Ragdoll::Diagnose() const {
    char buf[256];
    snprintf(buf, sizeof buf, "{\"fwd\":[%.3f,%.3f,%.3f],\"init\":[", initFwd.x, initFwd.y, initFwd.z);
    std::string s = buf;
    for (int i = 0; i < COUNT; i++) {
        snprintf(buf, sizeof buf, "%s[%.4f,%.4f,%.4f]", i ? "," : "", initPos[i].x, initPos[i].y, initPos[i].z);
        s += buf;
    }
    return s + "]}";
}

// Compares every driven frame as the engine sees it with the pose the physics asked for.
std::string Ragdoll::BoneCheck() const {
    std::string s = "{";
    char buf[256];
    for (size_t i = 0; i < bones.size(); i++) {
        const BoneLink &l = bones[i];
        const Body &bd = body[l.bodyIdx];
        const float *m = game::WorldMatrix(l.frame);
        V3 actual{m[12], m[13], m[14]};
        V3 expect = bd.x + Rotate(bd.q, l.originLocal) + origin;
        M3 rows{{Rotate(bd.q, {1, 0, 0}), Rotate(bd.q, {0, 1, 0}), Rotate(bd.q, {0, 0, 1})}};
        M3 want = l.offset * rows;
        M3 got = M3::FromWorld(m);
        float worst = 0;
        for (int k = 0; k < 3; k++) worst = std::max(worst, std::acos(Clamp(Dot(Norm(want.r[k]), got.r[k]), -1, 1)) / kDeg);
        snprintf(buf, sizeof buf, "%s\"%s\":[%.1f,%.1f]", i ? "," : "", l.name, Len(actual - expect) * 100.0f, worst);
        s += buf;
    }
    return s + "}";  // name: [position error cm, rotation error deg]
}


float Ragdoll::Overwritten(const char **bone) const {
    float worst = 0;
    *bone = "";
    for (auto &l : bones) {
        if (!l.wrote) continue;
        const float *q = &game::at<float>(l.frame, game::kFrameRot);
        float d = std::fabs(q[0] * l.written[0] + q[1] * l.written[1] + q[2] * l.written[2] + q[3] * l.written[3]);
        float deg = 2.0f * std::acos(Clamp(d, 0, 1)) / kDeg;
        if (deg > worst) { worst = deg; *bone = l.name; }
    }
    return worst;
}

void Ragdoll::Probe(V3 *phys, V3 *drawn, V3 *root) const {
    *phys = origin + body[B_PELVIS].x;
    const float *m = game::WorldMatrix(pelvisFrame);
    *drawn = {m[12], m[13], m[14]};
    const float *r = game::WorldMatrix(rootFrame);
    *root = {r[12], r[13], r[14]};
}
