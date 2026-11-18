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
    r.SetMotionFrom(previous, dt);
    if (r.BodyVel(Ragdoll::B_PELVIS).x < 11.9f) return 1;
    const V3 still{};
    r.SetMotionFrom(previous, dt, &still);
    for (int i = 0; i < Ragdoll::BCOUNT; ++i)
        if (Len(r.BodyVel(i)) > .001f) return 2;
    // Real native motion carries through. A swinging limb stays animated but adds no net translation.
    const V3 running{0, 0, -3.0f};
    r.initPos[Ragdoll::RTIP].x += .50f;
    r.SetMotionFrom(previous, dt, &running);
    V3 momentum{};
    for (int i = 0; i < Ragdoll::BCOUNT; ++i)
        momentum += (r.BodyVel(i) - running) * Ragdoll::SegmentMass[i];
    if (Len(momentum) > .005f || r.BodyVel(Ragdoll::B_RFARM).x < 1.0f) return 3;
    r.SetMotionFrom(nullptr, 0, &still);
    for (int i = 0; i < Ragdoll::BCOUNT; ++i)
        if (Len(r.BodyVel(i)) > .001f) return 4;
    std::puts("PASS: a 20 cm pose jump adds no bulk speed; native running and local limb motion carry through");
}
