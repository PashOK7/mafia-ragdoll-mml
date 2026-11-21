#pragma once
#include <span>
#include <cstdint>
struct MmlPedestrian;
struct V3;
class Ragdoll;
namespace JPH { class BodyID; }
namespace headwear {
void Prepare();
void ApplyPlayerFade();
void RestorePlayerFade();
void Tick(std::span<const MmlPedestrian> pedestrians);
void Advance(Ragdoll &body,float dt);
void FinishStep();
void Sync();
bool Near(V3 point);
void ApplyAirDrag(float dt);
bool OwnsBody(uint32_t body);
bool OwnsVisual(void *visual);
bool ReleasingFrom(uint32_t body,uint32_t wearerGroup);
bool Shot(V3 from,V3 axis,float range,V3 *point,uint32_t *body);
void BulletImpulse(uint32_t body,V3 direction,V3 point,float damage);
void BodyContact(uint32_t propBody);
void FlightContact(uint32_t body,const JPH::BodyID &other,unsigned layer,V3 point,V3 normal,float approach);
void Reset();
}
