#pragma once
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include "jolt_world.h"

namespace articulation {
inline JPH::Vec3 Vector(V3 v) { return {v.x,v.y,v.z}; }
inline constexpr float radians=0.01745329252f;

struct Axes { V3 axis, normal; };

inline JPH::Ref<JPH::SwingTwistConstraintSettings> Ball(V3 anchor,Axes parent,Axes child,
        float flex,float side,float twistLo,float twistHi,float frequency) {
    JPH::Ref<JPH::SwingTwistConstraintSettings> joint=new JPH::SwingTwistConstraintSettings;
    joint->mPosition1=joint->mPosition2=Vector(anchor);
    joint->mTwistAxis1=Vector(parent.axis);
    joint->mPlaneAxis1=Vector(parent.normal);
    joint->mTwistAxis2=Vector(child.axis);
    joint->mPlaneAxis2=Vector(child.normal);
    joint->mNormalHalfConeAngle=flex*radians;
    joint->mPlaneHalfConeAngle=side*radians;
    joint->mTwistMinAngle=twistLo*radians;
    joint->mTwistMaxAngle=twistHi*radians;
    joint->mSwingMotorSettings=joint->mTwistMotorSettings=
        JPH::MotorSettings(JPH::ESpringMode::FrequencyAndDamping,frequency,1.f);
    return joint;
}

inline JPH::Ref<JPH::HingeConstraintSettings> Hinge(V3 anchor,Axes parent,Axes child,
                                                   float minimum,float maximum,float frequency) {
    JPH::Ref<JPH::HingeConstraintSettings> joint=new JPH::HingeConstraintSettings;
    joint->mPoint1=joint->mPoint2=Vector(anchor);
    joint->mHingeAxis1=Vector(parent.axis);
    joint->mHingeAxis2=Vector(child.axis);
    joint->mNormalAxis1=Vector(parent.normal);
    joint->mNormalAxis2=Vector(child.normal);
    joint->mLimitsMin=minimum*radians;
    joint->mLimitsMax=maximum*radians;
    joint->mMotorSettings=JPH::MotorSettings(JPH::ESpringMode::FrequencyAndDamping,frequency,1.f);
    return joint;
}

inline void Prepare(JPH::RagdollSettings &rig,const JPH::Mat44 *pose,float tolerance) {
    rig.Stabilize();
    rig.DisableParentChildCollisions(pose,tolerance);
    rig.CalculateConstraintPriorities();
    rig.CalculateBodyIndexToConstraintIndex();
}

inline int ShotPart(const JPH::Ragdoll &rig,V3 from,V3 delta,V3 *point) {
    const JPH::RRayCast ray{Vector(jolt_world::ToJolt(from)),Vector(delta)};
    const auto &bodies=jolt_world::Bodies();
    JPH::RayCastResult hit;
    hit.mFraction=1.f;
    int part=-1;
    for (int i=0;i<(int)rig.GetBodyCount();++i) {
        if (bodies.GetTransformedShape(rig.GetBodyID(i)).CastRay(ray,hit)) part=i;
    }
    if (part>=0) *point=from+delta*hit.mFraction;
    return part;
}
}
