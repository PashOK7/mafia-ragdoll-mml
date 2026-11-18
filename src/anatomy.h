#pragma once
#include "ragdoll.h"

// The landmark order already identifies the front of each trunk segment (right cross spine). The model root
// keeps its standing heading while a death animation lies flat: using that heading to sign-correct each segment
// independently can turn one frame by 180 degrees when the back bends through horizontal.
struct TrunkAxes {
    V3 pelvisRight, pelvisFront, chestRight, chestFront;
    static TrunkAxes From(const V3 *p, const V3 &fallback) {
        using P = Ragdoll;
        TrunkAxes a;
        a.pelvisRight = Norm(p[P::RHIP] - p[P::LHIP], {1,0,0});
        a.pelvisFront = Norm(Cross(a.pelvisRight, p[P::BACK1] - p[P::PELVIS]), fallback);
        a.chestRight = Norm(p[P::RCLAV] - p[P::LCLAV], a.pelvisRight);
        a.chestFront = Norm(Cross(a.chestRight, p[P::NECK] - p[P::BACK3]), a.pelvisFront);
        return a;
    }
};
