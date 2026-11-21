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
    float depth=.09f, leverage=.45f;
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

    bool slips(V3 up,V3 crownVelocity,float dt) {
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
        float shear=Len(load-up*axial);
        // Lateral force applied above the band tips it about the rim.
        float peel=leverage*shear;
        float pull=axial+peel-2.8f;
        int steps=int(std::ceil(dt*120.f));
        float step=dt/steps;
        for (int i=0;i<steps;++i) {
            liftRate+=(pull-30.f*lift-8.f*liftRate)*step;
            lift+=liftRate*step;
            if (lift<0) {
                lift=0;
                liftRate=0;
            }
            if (lift>=depth) { lift=depth; return true; }
        }
        return false;
    }
};
