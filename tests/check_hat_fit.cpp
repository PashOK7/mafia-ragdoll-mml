// Compile with the SDK game headers on the include path.
#include <cassert>
#include "../src/hat_fit.h"

int main() {
    for (float fps : {20.f,30.f,60.f,144.f}) {
        const float dt=1/fps;
        const V3 up{0,1,0};
        HatFit upright,tilted,falling,inverted,impact;
        float inversionTime=0;
        bool released=false;
        for (int i=0;i<int(4*fps);++i) {
            assert(!upright.slips(up,{8,0,0},dt));
            assert(!tilted.slips({.5f,.8660254f,0},{},dt));
            V3 freeFall{0,-9.81f*i*dt,0};
            assert(!falling.slips({1,0,0},freeFall,dt));
            if (!released) {
                released=inverted.slips({0,-1,0},{},dt);
                inversionTime=i*dt;
            }
        }
        assert(released && inversionTime>.12f && inversionTime<.40f);
        // With an upright head, a strong lateral acceleration can tip the
        // band during the shove. Mild acceleration leaves it seated.
        HatFit mild;
        released=false;
        for (int i=0;i<int(fps*.3f);++i) {
            float t=i*dt;
            assert(!mild.slips(up,{0,0,4*t},dt));
            if (impact.slips(up,{0,0,120*t},dt)) {released=true;break;}
        }
        assert(released);
        HatFit carried;
        for (int i=0;i<=int(fps);++i) {
            V3 centre{0,1,18*i*dt};
            carried.track(centre+V3{.05f,.22f,0},centre,dt);
        }
        assert(std::abs(carried.hatMotion.z-18)<.02f);
        assert(std::abs(carried.torsoMotion.z-18)<.02f);
        carried.track({300,2,100},{300,1,100},dt);
        assert(Len(carried.hatMotion)==0 && Len(carried.torsoMotion)==0);
    }
}
