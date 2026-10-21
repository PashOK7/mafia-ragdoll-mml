// Collision shapes and queries for the ragdoll. The world (static grid and moving objects) is reduced to a
// few convex primitives; the ragdoll asks for everything inside a box and tests its capsules against it.
#pragma once
#include <vector>
#include "mathx.h"

struct CollShape {
    enum Kind : unsigned char { TRI, BOX, SPHERE, VCYL, CAPSULE };
    Kind kind = TRI;
    // TRI: p[0..2] vertices, n face normal (points to the free side).
    // BOX: c center, ax[0..2] unit axes in world, half extents h.
    // SPHERE: c center, r radius.  VCYL: c (x, z used) axis, r radius, infinite in y.
    // CAPSULE: p[0], p[1] segment ends, r radius.
    V3 p[3]{}, n{}, c{}, ax[3]{}, h{};
    float r = 0;
    // TRI only: how far behind its face a point still counts as penetrating it. Large for the ground and
    // walls (thick solids behind), small for model shells such as car bodies (thin, detailed).
    float backDepth = 0.4f;
    // Motion of whatever carries the shape (zero for the static world): velocity of a point x is
    // vel + Cross(angVel, x - pivot).
    V3 vel{}, angVel{}, pivot{};
    const void *id = nullptr;   // source primitive, for debugging and de-duplication
    void *owner = nullptr;      // actor that owns a moving shape, if any
    V3 lo{}, hi{};              // world bounds

    V3 VelocityAt(const V3 &x) const { return vel + Cross(angVel, x - pivot); }
    void ComputeBounds();
    void Translate(const V3 &d);
};

// Signed distance from a point to the shape (negative inside) and the outward unit normal there.
float SignedDistance(const CollShape &s, const V3 &x, V3 *normal);

// Deepest point of segment a-b against the shape: returns the segment parameter t in [0,1] where the
// signed distance is smallest, and that distance.
float DeepestOnSegment(const CollShape &s, const V3 &a, const V3 &b, float *tOut);

// Closest points between segments p1-q1 and p2-q2 (Ericson 5.1.9); returns the squared distance.
float ClosestSegSeg(const V3 &p1, const V3 &q1, const V3 &p2, const V3 &q2, float *s, float *t, V3 *c1, V3 *c2);

// Something that can list collision shapes in an axis-aligned box. `lo`/`hi` and the returned shapes are
// relative to `origin` (the caller's local frame), so callers far from the world origin keep their precision.
// Shapes owned by `excludeOwner` (the ragdoll's own actor) are left out.
struct CollisionSource {
    virtual ~CollisionSource() = default;
    virtual void Gather(const V3 &origin, const V3 &lo, const V3 &hi, void *excludeOwner, std::vector<CollShape> &out) = 0;
    // Only the fixed world (no owner, no motion), same frame convention as Gather.
    virtual void GatherStatic(const V3 &origin, const V3 &lo, const V3 &hi, std::vector<CollShape> &out) {
        std::vector<CollShape> list;
        Gather(origin, lo, hi, nullptr, list);
        for (auto &s : list)
            if (!s.owner && Dot(s.vel, s.vel) == 0 && Dot(s.angVel, s.angVel) == 0) out.push_back(s);
    }
    // Is anything moving (a car, a walking human) inside the box? Lets a sleeping body wake up when touched.
    virtual bool MovingNear(const V3 &origin, const V3 &lo, const V3 &hi, void *excludeOwner) {
        std::vector<CollShape> list;
        Gather(origin, lo, hi, excludeOwner, list);
        for (auto &s : list)
            if (Dot(s.vel, s.vel) > 0.04f || Dot(s.angVel, s.angVel) > 0.04f) return true;
        return false;
    }
};
