#pragma once
#include <span>
struct MmlPedestrian;
class Ragdoll;
namespace headwear {
void Prepare();
void Tick(std::span<const MmlPedestrian> pedestrians);
void Drop(Ragdoll &body, float dt);
void Reset();
}
