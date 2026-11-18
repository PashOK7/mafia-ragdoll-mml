#include "anatomy.h"
#include <cstdio>
#include <cmath>

int main(int, char **) {
    // A curved back lying face-down. Its two front vectors have opposite tiny projections onto the unchanged
    // standing root heading: the old independent sign correction made them oppose by almost 180 degrees.
    V3 p[Ragdoll::COUNT]{};
    p[Ragdoll::RHIP] = {.1f,0,0}; p[Ragdoll::LHIP] = {-.1f,0,0};
    p[Ragdoll::BACK1] = {0,.01f,.2f}; p[Ragdoll::BACK3] = {0,.02f,.5f};
    p[Ragdoll::NECK] = {0,.01f,.7f};
    p[Ragdoll::RCLAV] = {.1f,.02f,.5f}; p[Ragdoll::LCLAV] = {-.1f,.02f,.5f};
    for (float side : {-1.f,1.f}) {
        V3 posed[Ragdoll::COUNT];
        for (int i=0;i<Ragdoll::COUNT;i++) { posed[i]=p[i]; posed[i].z*=side; }
        for (V3 heading : {V3{0,0,1},V3{0,0,-1},V3{1,0,0}}) {
            auto a = TrunkAxes::From(posed, heading);
            if (Dot(a.pelvisFront,a.chestFront) < .99f) { std::puts("FAIL: lying back frame flips"); return 1; }
            auto b = TrunkAxes::From(posed,-heading);
            if (Dot(a.pelvisFront,b.pelvisFront) < .999f || Dot(a.chestFront,b.chestFront) < .999f) {
                std::puts("FAIL: root heading changed posed anatomy"); return 1;
            }
        }
    }
    std::puts("PASS: lying curved trunk frames are continuous and independent of standing root heading");
}
