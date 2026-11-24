#include "mod.h"
#include "ragdoll.h"
#include <cstdio>

namespace mod {
const MmlHost *H;
MmlMod *M;
}

int main(int, char **) {
    Ragdoll r;
    V3 previous[Ragdoll::COUNT]{};
    constexpr float dt = .016f;
    for (int i = 0; i < Ragdoll::COUNT; ++i) {
        previous[i] = {0, .05f * i, 0};
        r.initPos[i] = previous[i] + V3{.20f, 0, 0};
    }
    const V3 still{};
    r.SetMotionFrom(still, previous, dt);
    for (int i = 0; i < Ragdoll::BCOUNT; ++i)
        if (Len(r.BodyVel(i)) > .001f) return 2;
    // Real native motion carries through. A swinging limb stays animated but adds no net translation.
    const V3 running{0, 0, -3.0f};
    r.initPos[Ragdoll::RTIP].x += .50f;
    r.SetMotionFrom(running, previous, dt);
    V3 momentum{};
    for (int i = 0; i < Ragdoll::BCOUNT; ++i)
        momentum += (r.BodyVel(i) - running) * Ragdoll::SegmentMass[i];
    if (Len(momentum) > .005f || r.BodyVel(Ragdoll::B_RFARM).x < 1.0f) return 3;
    r.SetMotionFrom(still);
    for (int i = 0; i < Ragdoll::BCOUNT; ++i)
        if (Len(r.BodyVel(i)) > .001f) return 4;
    // The pose captured at the hit may be far from either motion sample.
    // Its geometry must not enter the velocity calculation.
    V3 sampled[Ragdoll::COUNT];
    for (int i = 0; i < Ragdoll::COUNT; ++i) {
        sampled[i] = previous[i] + V3{0, 0, -.048f};
        r.initPos[i] = previous[i] + V3{.35f, -.2f, 0};
    }
    sampled[Ragdoll::RTIP].x += .05f;
    r.SetMotionFrom(running, previous, dt, sampled);
    const V3 hand = r.BodyVel(Ragdoll::B_RFARM);
    const V3 pelvis = r.BodyVel(Ragdoll::B_PELVIS);
    if (hand.x <= 0 || pelvis.x >= 0 || std::abs(pelvis.y) > .001f || std::abs(pelvis.z + 3.f) > .001f) return 5;
    std::puts("PASS: native translation and compatible limb motion; starting pose independent of the velocity sample");
}
