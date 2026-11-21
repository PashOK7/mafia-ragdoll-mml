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
            assert(!upright.slips(up,{8,0,0},dt,{8,0,0}));
            assert(!tilted.slips({.5f,.8660254f,0},{},dt,{}));
            V3 freeFall{0,-9.81f*i*dt,0};
            assert(!falling.slips({1,0,0},freeFall,dt,freeFall));
            if (!released) {
                released=inverted.slips({0,-1,0},{},dt,{});
                inversionTime=i*dt;
            }
        }
        assert(released && inversionTime>.20f && inversionTime<.40f);
        // A strong crosswise impulse accelerates the whole seated hat. The
        // original fit stripped it here and cancelled the launch velocity.
        assert(!impact.slips(up,{},dt,{}));
        for (int i=0;i<int(fps/2);++i) assert(!impact.slips(up,{0,0,18},dt,{0,0,18}));
        for (int i=0;i<int(fps/2);++i) assert(!impact.slips({0,-1,0},{0,0,5},dt,{0,0,18}));
        released=false;
        for (int i=0;i<int(fps);++i) {
            if (impact.slips({0,-1,0},{0,0,18},dt,{0,0,18})) {released=true;break;}
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
