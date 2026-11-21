#pragma once
#include <cstddef>
#include <string>
#include "mathx.h"

struct MmlBulletHit;

namespace dogs {
struct ShotHit { void *actor=nullptr; int part=-1; V3 point{}; };

void PrepareStep(float dt, const V3 &player);
bool Near(const V3 &point);
void Bullet(const MmlBulletHit &hit);
std::size_t AppendShotExclusions(void **objects,std::size_t count,std::size_t capacity);
bool TraceShot(V3 from,V3 axis,float limit,ShotHit &hit);
void HitBody(const ShotHit &hit,V3 direction,float strength);
void DrawPose();
void RestorePose();
void Forget(void *actor);
void Clear();
std::string State();
}
