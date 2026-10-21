#include "collision.h"
#include <algorithm>
#include <cmath>

namespace {

// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection, 5.1.5).
V3 ClosestOnTriangle(const V3 &p, const V3 &a, const V3 &b, const V3 &c) {
    V3 ab = b - a, ac = c - a, ap = p - a;
    float d1 = Dot(ab, ap), d2 = Dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    V3 bp = p - b;
    float d3 = Dot(ab, bp), d4 = Dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
    V3 cp = p - c;
    float d5 = Dot(ab, cp), d6 = Dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
    float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    float denom = 1.0f / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

V3 ClosestOnSegment(const V3 &p, const V3 &a, const V3 &b) {
    V3 ab = b - a;
    float l2 = Dot(ab, ab);
    float t = l2 > 1e-12f ? std::min(1.0f, std::max(0.0f, Dot(p - a, ab) / l2)) : 0.0f;
    return a + ab * t;
}

} // namespace

void CollShape::ComputeBounds() {
    switch (kind) {
    case TRI:
        lo = hi = p[0];
        for (int i = 1; i < 3; i++) {
            lo = {std::min(lo.x, p[i].x), std::min(lo.y, p[i].y), std::min(lo.z, p[i].z)};
            hi = {std::max(hi.x, p[i].x), std::max(hi.y, p[i].y), std::max(hi.z, p[i].z)};
        }
        break;
    case BOX: {
        V3 e{0, 0, 0};
        for (int i = 0; i < 3; i++) {
            V3 a = ax[i] * (i == 0 ? h.x : i == 1 ? h.y : h.z);
            e += V3{std::fabs(a.x), std::fabs(a.y), std::fabs(a.z)};
        }
        lo = c - e; hi = c + e;
        break;
    }
    case SPHERE: lo = c - V3{r, r, r}; hi = c + V3{r, r, r}; break;
    case VCYL: lo = {c.x - r, -1e9f, c.z - r}; hi = {c.x + r, 1e9f, c.z + r}; break;
    case CAPSULE:
        lo = {std::min(p[0].x, p[1].x) - r, std::min(p[0].y, p[1].y) - r, std::min(p[0].z, p[1].z) - r};
        hi = {std::max(p[0].x, p[1].x) + r, std::max(p[0].y, p[1].y) + r, std::max(p[0].z, p[1].z) + r};
        break;
    }
}

void CollShape::Translate(const V3 &d) {
    for (auto &v : p) v += d;
    c += d; pivot += d; lo += d; hi += d;
}

float SignedDistance(const CollShape &s, const V3 &x, V3 *normal) {
    switch (s.kind) {
    case CollShape::TRI: {
        V3 cp = ClosestOnTriangle(x, s.p[0], s.p[1], s.p[2]);
        V3 d = x - cp;
        float plane = Dot(x - s.p[0], s.n);
        float len = Len(d);
        // Behind the face and inside its outline: penetrating, measured along the face normal.
        if (plane < 0 && plane > -s.backDepth && len < 1e-4f - plane) {
            V3 proj = x - s.n * plane;
            if (Len(ClosestOnTriangle(proj, s.p[0], s.p[1], s.p[2]) - proj) < 1e-4f) { *normal = s.n; return plane; }
        }
        if (len > 1e-6f) { *normal = d * (1.0f / len); return len; }
        *normal = s.n;
        return 0;
    }
    case CollShape::BOX: {
        V3 rel = x - s.c;
        V3 q{Dot(rel, s.ax[0]), Dot(rel, s.ax[1]), Dot(rel, s.ax[2])};
        V3 d{std::fabs(q.x) - s.h.x, std::fabs(q.y) - s.h.y, std::fabs(q.z) - s.h.z};
        V3 out{std::max(d.x, 0.0f), std::max(d.y, 0.0f), std::max(d.z, 0.0f)};
        float outside = Len(out);
        if (outside > 0) {
            V3 cl{std::min(std::max(q.x, -s.h.x), s.h.x), std::min(std::max(q.y, -s.h.y), s.h.y), std::min(std::max(q.z, -s.h.z), s.h.z)};
            V3 nl = (q - cl) * (1.0f / outside);
            *normal = s.ax[0] * nl.x + s.ax[1] * nl.y + s.ax[2] * nl.z;
            return outside;
        }
        int k = d.x > d.y ? (d.x > d.z ? 0 : 2) : (d.y > d.z ? 1 : 2);
        float qk = k == 0 ? q.x : k == 1 ? q.y : q.z;
        *normal = s.ax[k] * (qk >= 0 ? 1.0f : -1.0f);
        return k == 0 ? d.x : k == 1 ? d.y : d.z;
    }
    case CollShape::SPHERE: {
        V3 d = x - s.c;
        float l = Len(d);
        *normal = l > 1e-6f ? d * (1.0f / l) : V3{0, 1, 0};
        return l - s.r;
    }
    case CollShape::VCYL: {
        V3 d{x.x - s.c.x, 0, x.z - s.c.z};
        float l = Len(d);
        *normal = l > 1e-6f ? d * (1.0f / l) : V3{1, 0, 0};
        return l - s.r;
    }
    case CollShape::CAPSULE: {
        V3 d = x - ClosestOnSegment(x, s.p[0], s.p[1]);
        float l = Len(d);
        *normal = l > 1e-6f ? d * (1.0f / l) : V3{0, 1, 0};
        return l - s.r;
    }
    }
    *normal = {0, 1, 0};
    return 1e9f;
}

// Closest points between a segment and a triangle (exact): the segment crossing the face, its ends against
// the face, or the segment against one of the three edges.
static float SegmentTriangle(const V3 &a, const V3 &b, const V3 &p0, const V3 &p1, const V3 &p2, const V3 &n, float *tOut) {
    float da = Dot(a - p0, n), db = Dot(b - p0, n);
    if ((da <= 0 && db >= 0) || (da >= 0 && db <= 0)) {
        float den = da - db;
        float t = std::fabs(den) > 1e-12f ? da / den : 0.0f;
        V3 x = a + (b - a) * t;
        if (Len(ClosestOnTriangle(x, p0, p1, p2) - x) < 1e-5f) { *tOut = t; return 0; }
    }
    float best = Len(ClosestOnTriangle(a, p0, p1, p2) - a), bt = 0;
    float d1 = Len(ClosestOnTriangle(b, p0, p1, p2) - b);
    if (d1 < best) { best = d1; bt = 1; }
    const V3 *e[3][2] = {{&p0, &p1}, {&p1, &p2}, {&p2, &p0}};
    for (auto &edge : e) {
        float ss, tt; V3 c1, c2;
        float d = std::sqrt(std::max(0.0f, ClosestSegSeg(a, b, *edge[0], *edge[1], &ss, &tt, &c1, &c2)));
        if (d < best) { best = d; bt = ss; }
    }
    *tOut = bt;
    return best;
}

float DeepestOnSegment(const CollShape &s, const V3 &a, const V3 &b, float *tOut) {
    // A one-sided-free triangle (backDepth 0, car meshes): its distance is unsigned, the closest points are exact.
    if (s.kind == CollShape::TRI && s.backDepth <= 0) return SegmentTriangle(a, b, s.p[0], s.p[1], s.p[2], s.n, tOut);
    // The signed distance of a point moving along a segment to a convex shape is convex in t,
    // so a ternary search finds the deepest point.
    V3 n;
    float lo = 0, hi = 1;
    for (int it = 0; it < 22; it++) {
        float m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
        float f1 = SignedDistance(s, a + (b - a) * m1, &n), f2 = SignedDistance(s, a + (b - a) * m2, &n);
        if (f1 < f2) hi = m2; else lo = m1;
    }
    float t = 0.5f * (lo + hi);
    float d = SignedDistance(s, a + (b - a) * t, &n);
    // The ends are candidates too (the search interval never quite reaches them).
    float d0 = SignedDistance(s, a, &n), d1 = SignedDistance(s, b, &n);
    if (d0 < d) { d = d0; t = 0; }
    if (d1 < d) { d = d1; t = 1; }
    *tOut = t;
    return d;
}

float ClosestSegSeg(const V3 &p1, const V3 &q1, const V3 &p2, const V3 &q2, float *s, float *t, V3 *c1, V3 *c2) {
    V3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    float a = Dot(d1, d1), e = Dot(d2, d2), f = Dot(d2, r);
    const float eps = 1e-9f;
    float ss, tt;
    if (a <= eps && e <= eps) { ss = tt = 0; }
    else if (a <= eps) { ss = 0; tt = std::min(1.0f, std::max(0.0f, f / e)); }
    else {
        float c = Dot(d1, r);
        if (e <= eps) { tt = 0; ss = std::min(1.0f, std::max(0.0f, -c / a)); }
        else {
            float b = Dot(d1, d2), denom = a * e - b * b;
            ss = denom > eps ? std::min(1.0f, std::max(0.0f, (b * f - c * e) / denom)) : 0.0f;
            tt = (b * ss + f) / e;
            if (tt < 0) { tt = 0; ss = std::min(1.0f, std::max(0.0f, -c / a)); }
            else if (tt > 1) { tt = 1; ss = std::min(1.0f, std::max(0.0f, (b - c) / a)); }
        }
    }
    *s = ss; *t = tt;
    *c1 = p1 + d1 * ss;
    *c2 = p2 + d2 * tt;
    V3 d = *c1 - *c2;
    return Dot(d, d);
}
