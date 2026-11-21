#pragma once
class Ragdoll;
namespace headwear {
void Prepare();
void Tick();
void Drop(Ragdoll &body, float dt);
void Reset();
}
