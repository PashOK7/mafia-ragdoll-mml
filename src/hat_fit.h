#pragma once
#include <algorithm>
#include "mathx.h"

// The band bears lateral loads while seated. To release, it must clear the
// crown along the head axis. The fit state describes engagement, not a second
// moving body: the worn visual travels rigidly with the physical head.
struct HatFit {
    V3 mount{},lastHat{},lastTorso{},firstTorso{},hatMotion{},torsoMotion{};
    bool tracking=false;
    V3 previousVelocity{};
    V3 shotMotion{};
    V3 pendingShot{};
    float shotAge=0;
    float lift=0, liftRate=0;
    float depth=.09f, leverage=.45f;
    bool sampled=false;
    bool ridingShot=false;
    float engaged=0.f;

    void shot(V3 delta,float age) {
        shotMotion=shotMotion*std::exp(-std::max(0.f,age-shotAge)/.18f)+delta;
        pendingShot+=delta;
        shotAge=age;
    }

    V3 releaseVelocity(V3 headVelocity,float age) const {
        float speed=Len(shotMotion);
        if (speed<1e-4f) return hatMotion;
        V3 axis=shotMotion*(1.f/speed);
        float residue=std::exp(-std::max(0.f,age-shotAge)/.18f);
        float inherited=Dot(hatMotion,axis);
        // A shot changes the head velocity before the position-tracking filter
        // catches up. Keep the actual forward speed and give the loosened
        // band a small departure speed so it clears the following skull.
        float forward=std::max(inherited,Dot(headVelocity,axis))+
                      std::min(1.8f,.40f*speed)*residue;
        return hatMotion+axis*((forward-inherited)*residue);
    }

    bool clearsShotGuide(float span,float dt) {
        if (!ridingShot) return true;
        V3 forward{shotMotion.x,0,shotMotion.z};
        const V3 direction=Norm(forward,{});
        const float translation=std::max(0.f,Dot(lastTorso-firstTorso,direction));
        // While the band is riding over the crown, its motion remains coupled
        // to the wearer. It clears over the cap's own span as the trunk moves.
        // If only the head rotates (e.g. a body backed against a wall), let the
        // measured axial opening carry it free instead.
        const float forwardSpeed=Dot(torsoMotion,direction);
        engaged+=std::max(0.f,liftRate)*dt*(1.f-std::clamp(forwardSpeed/.85f,0.f,1.f));
        if (translation+engaged<span) return false;
        return true;
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
        // Sideways loading can peel a tipped band over the crown. An upright
        // band transfers that same load to the hat as the head accelerates.
        float shear=Len(load-up*axial);
        // Lateral force applied above the band tips it about the rim.
        float peel=leverage*shear;
        float pull=axial+peel-2.8f;
        // The shot drives the head before the finite-difference tracker sees
        // that motion. Apply its sideways impulse to the band once.
        const V3 crosswise=pendingShot-up*Dot(pendingShot,up);
        if (Len(crosswise)>.1f && !ridingShot) {
            ridingShot=true;
            engaged=0.f;
        }
        liftRate+=std::min(2.5f,Len(crosswise)*leverage*2.6f);
        pendingShot={};
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
