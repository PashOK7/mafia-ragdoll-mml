#pragma once
#include "ragdoll.h"
#include <algorithm>

// The same spine, clavicles and arm lengths determine what the hands can reach
// in the physics tick and where they are placed in the render pose.
struct PullReach {
    V3 pivot, goal[2], clav[2], shoulder[2];
    float reach[2], gap = 0;
    M3 lean, swing[2];

    static M3 Swing(V3 pivot, V3 shoulder, V3 goal, float reach, float limit) {
        const V3 delta = goal - shoulder;
        if (Len(delta) <= reach) return M3::Identity();
        const float radius = Len(shoulder - pivot), distance = Len(goal - pivot);
        if (radius * distance < 1e-6f) return M3::Identity();
        const V3 from = Norm(shoulder - pivot), to = Norm(goal - pivot);
        const V3 normal = Cross(from, to);
        const float cone = std::acos(std::clamp((radius*radius + distance*distance - reach*reach) /
                                                (2*radius*distance), -1.f, 1.f));
        const float half = .5f * std::clamp(std::atan2(Len(normal), Dot(from, to)) - cone, 0.f, limit);
        // LS3D applies row vectors: a positive quaternion spins X towards -Y about Z.
        const V3 v = Norm(normal) * -std::sin(half);
        float m[16];
        QuatMatrix({std::cos(half), v.x, v.y, v.z}, m);
        return M3::Raw(m);
    }

    PullReach(const V3 *pose, const V3 (&contact)[2], float weight) {
        using R = Ragdoll;
        const V3 upper[] = {pose[R::LARM], pose[R::RARM]};
        const V3 elbow[] = {pose[R::LELB], pose[R::RELB]};
        const V3 wrist[] = {pose[R::LHAND], pose[R::RHAND]};
        float excess[2];
        for (int i = 0; i < 2; ++i) {
            goal[i] = wrist[i] + (contact[i] - wrist[i]) * weight;
            reach[i] = Len(wrist[i]-elbow[i]) + Len(elbow[i]-upper[i]) - .004f;
            excess[i] = Len(goal[i]-upper[i]) - reach[i];
        }
        const int limitingArm = excess[0] > excess[1] ? 0 : 1;
        pivot = pose[R::BACK1];
        lean = Swing(pivot, upper[limitingArm], goal[limitingArm], reach[limitingArm], .90f);
        for (int i = 0; i < 2; ++i) {
            clav[i] = pivot + lean.Apply(pose[i ? R::RCLAV : R::LCLAV] - pivot);
            const V3 arm = pivot + lean.Apply(upper[i] - pivot);
            swing[i] = Swing(clav[i], arm, goal[i], reach[i], .65f);
            shoulder[i] = clav[i] + swing[i].Apply(arm - clav[i]);
            gap = std::max(gap, Len(goal[i]-shoulder[i]) - reach[i]);
        }
    }
};
