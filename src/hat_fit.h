#pragma once
#include <algorithm>
#include "mathx.h"

// The band bears lateral loads while seated. To release, it must clear the
// crown along the head axis. The fit state describes engagement, not a second
// moving body: the worn visual travels rigidly with the physical head.
struct HatFit {
    V3 mount{},lastHat{},lastTorso{},hatMotion{},torsoMotion{};
    bool tracking=false;
    V3 previousVelocity{};
    float lift=0, liftRate=0;
    bool sampled=false;

    // Follow the actual mounted point. A contact solver can move a body by
    // positional correction while reporting almost zero end-of-step velocity.
    void track(V3 hat,V3 torso,float dt) {
        if (tracking && dt>0 && dt<=.15f && Len(hat-lastHat)<80.f*dt) {
            float blend=-std::expm1(-dt/.055f);
            hatMotion+=((hat-lastHat)*(1.f/dt)-hatMotion)*blend;
            torsoMotion+=((torso-lastTorso)*(1.f/dt)-torsoMotion)*blend;
        } else {
            hatMotion=torsoMotion={};
            sampled=false;
        }
        lastHat=hat;
        lastTorso=torso;
        tracking=true;
    }

    bool slips(V3 up,V3 crownVelocity,float dt,V3 torsoVelocity) {
        if (!(dt>0)) return false;
        if (!sampled || dt>.15f) {
            previousVelocity=crownVelocity;
            lift=liftRate=0;
            sampled=true;
            return false;
        }
        V3 load=V3{0,-9.81f,0}-(crownVelocity-previousVelocity)*(1.f/dt);
        previousVelocity=crownVelocity;
        float axial=Dot(load,up);
        // Sideways loading can peel a tipped band over the crown. An upright
        // band transfers that same load to the hat as the head accelerates.
        float tipped=std::clamp(1.f-std::max(0.f,up.y),0.f,1.f);
        float shear=Len(load-up*axial);
        float peel=.65f*tipped*std::max(0.f,shear-6.f);
        // Limit how rapidly a yielding band can open during a contact spike.
        float pull=std::clamp(axial+peel-2.8f,-18.f,18.f);
        int steps=int(std::ceil(dt*120.f));
        float step=dt/steps;
        for (int i=0;i<steps;++i) {
            liftRate+=(pull-30.f*lift-8.f*liftRate)*step;
            liftRate=std::min(.42f,liftRate);
            lift+=liftRate*step;
            if (lift<0) {
                lift=0;
                liftRate=0;
            }
            if (lift>=.09f) {
                lift=.09f;
                // During impact the trunk can already be travelling while the
                // neck is still catching up. Keep carrying the hat in that phase.
                float speed=Len(torsoVelocity);
                if (Dot(crownVelocity,torsoVelocity)>=speed*(speed*.88f-.65f)) return true;
            }
        }
        return false;
    }
};
