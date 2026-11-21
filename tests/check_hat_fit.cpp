// Compile with the SDK game headers on the include path.
#include <cassert>
#include "../src/hat_fit.h"

int main() {
    for (float fps : {20.f,30.f,60.f,144.f}) {
        const float dt=1/fps;
        const V3 up{0,1,0};
        HatFit upright,tilted,falling,inverted,jolted;
        float inversionTime=0;
        bool released=false;
        for (int i=0;i<int(4*fps);++i) {
            assert(!upright.slips(up,{8,0,0},dt));
            assert(!tilted.slips({.5f,.8660254f,0},{},dt));
            assert(!falling.slips({1,0,0},{0,-9.81f*i*dt,0},dt));
            if (!released) {
                released=inverted.slips({0,-1,0},{},dt);
                inversionTime=i*dt;
            }
        }
        assert(released && inversionTime>.06f && inversionTime<.20f);
        assert(!jolted.slips(up,{},dt));
        released=false;
        for (int i=0;i<int(fps);++i) {
            if (jolted.slips(up,{4,0,0},dt)) {released=true;break;}
        }
        assert(released && jolted.velocity.x<0);
    }
}
