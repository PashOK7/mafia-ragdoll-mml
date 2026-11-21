#pragma once
#include <algorithm>
#include "mathx.h"

// The band bears lateral loads while seated. To release, it must clear the
// crown along the head axis. Its integrated lift drives the mounted visual
// until the same pose can be handed to the free rigid body.
struct HatFit {
    V3 mount{},lastHat{},lastTorso{},firstTorso{},hatMotion{},torsoMotion{};
    bool tracking=false;
    V3 previousVelocity{};
    float lift=0, liftRate=0;
    float depth=.09f, travel=.16f, leverage=.45f;
    static constexpr float slideSpeed=1.65f;
    bool sampled=false;

    V3 carryAxis() const {
        return Norm({torsoMotion.x,0,torsoMotion.z},{});
    }

    // Follow the actual mounted point. A contact solver can move a body by
    // positional correction while reporting almost zero end-of-step velocity.
    void track(V3 hat,V3 torso,float dt) {
        if (tracking && dt>0 && dt<=.15f && Len(hat-lastHat)<80.f*dt) {
            float blend=-std::expm1(-dt/.055f);
            hatMotion+=((hat-lastHat)*(1.f/dt)-hatMotion)*blend;
            torsoMotion+=((torso-lastTorso)*(1.f/dt)-torsoMotion)*blend;
        } else {
            hatMotion=torsoMotion={};
            firstTorso=torso;
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
        const float shear=Len(load-up*axial);
        // Load pressing the hat into the crown increases the band friction.
        // Only the remaining sideways load can lever the rim up.
        constexpr float friction=.45f;
        constexpr float initialGrip=4.8f;
        constexpr float stiffness=40.f;
        constexpr float damping=10.f;
        const float peel=leverage*std::max(0.f,shear-friction*std::max(0.f,-axial));
        const float pull=axial+peel-initialGrip;
        // Keep the band travel bounded even when a delayed frame batches time.
        const float slideTime=std::min(dt,.04f);
        const int steps=int(std::ceil(slideTime*120.f));
        const float step=slideTime/steps;
        for (int i=0;i<steps;++i) {
            liftRate=std::clamp(liftRate+(pull-stiffness*lift-damping*liftRate)*step,
                                -slideSpeed,slideSpeed);
            const float next=lift+liftRate*step;
            lift=std::clamp(next,0.f,travel);
            if (lift!=next) liftRate=0;
        }
        return lift>=depth;
    }
};
