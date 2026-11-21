#pragma once
#include <string>

struct MmlBulletHit;
struct V3;

namespace dogs {
void PrepareStep(float dt, const V3 &player);
bool Near(const V3 &point);
void Bullet(const MmlBulletHit &hit);
void DrawPose();
void RestorePose();
void Forget(void *actor);
void Clear();
std::string State();
}
