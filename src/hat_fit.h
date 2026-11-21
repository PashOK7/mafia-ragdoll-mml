#pragma once
#include <algorithm>
#include "mathx.h"

// Displacement of a hat relative to its seat on the head, in world metres.
// Gravity and the change in velocity of the crown act on the loose hat.
struct HatFit {
    V3 previousVelocity{}, offset{}, velocity{};
    bool sampled=false;

    bool slips(V3 up,V3 crownVelocity,float dt) {
        if (!(dt>0)) return false;
        if (!sampled || dt>.15f) {
            previousVelocity=crownVelocity;
            offset=velocity={};
            sampled=true;
            return false;
        }
        V3 load=V3{0,-9.81f,0}-(crownVelocity-previousVelocity)*(1.f/dt);
        previousVelocity=crownVelocity;
        float axial=Dot(load,up);
        // The head supports inward loads. That pressure increases lateral grip.
        const V3 tangent=load-up*axial;
        const float grip=5.f+.7f*std::max(0.f,-axial);
        V3 drive=up*std::max(0.f,axial-2.f);
        float side=Len(tangent);
        if (side>grip) drive+=tangent*((side-grip)/side);
        int steps=int(std::ceil(dt*120.f));
        float step=dt/steps;
        for (int i=0;i<steps;++i) {
            velocity+=(drive-offset*42.f-velocity*8.f)*step;
            offset+=velocity*step;
            float inward=Dot(offset,up);
            if (inward<0) {
                offset-=up*inward;
                velocity-=up*std::min(0.f,Dot(velocity,up));
            }
            if (Dot(offset,offset)>=.04f*.04f) return true;
        }
        return false;
    }
};
