#pragma once
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <algorithm>
#include <cmath>

namespace jolt_ragdoll {
struct Resistance {
    float friction; // N m
    float damping;  // N m s/rad
};
inline constexpr Resistance neckResistance{.02f, 1.5f};

// The solver applies equal and opposite torques to the joined bodies.
// Use only relative rotation, with a finite torque even at impact speeds.
inline void SetPassiveResistance(JPH::TwoBodyConstraint &joint, Resistance resistance, float scale) {
    const auto *parent = joint.GetBody1();
    const auto relative = joint.GetBody2()->GetAngularVelocity() - parent->GetAngularVelocity();
    auto torque = [&](float speed) {
        return scale * (resistance.friction + resistance.damping * std::min(speed, 6.f));
    };
    if (joint.GetSubType() == JPH::EConstraintSubType::Hinge) {
        auto &hinge = static_cast<JPH::HingeConstraint &>(joint);
        const auto axis = parent->GetRotation() * hinge.GetLocalSpaceHingeAxis1();
        hinge.SetMaxFrictionTorque(torque(std::abs(relative.Dot(axis))));
    } else {
        auto &ball = static_cast<JPH::SwingTwistConstraint &>(joint);
        ball.SetMaxFrictionTorque(torque(relative.Length()));
    }
}
}
