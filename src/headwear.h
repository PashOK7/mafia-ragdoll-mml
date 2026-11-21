#pragma once
#include <span>
#include <cstdint>
struct MmlPedestrian;
struct V3;
class Ragdoll;
namespace headwear {
void Prepare();
void Tick(std::span<const MmlPedestrian> pedestrians);
void Drop(Ragdoll &body, float dt);
void Sync();
bool Near(V3 point);
void ApplyAirDrag(float dt);
bool OwnsBody(uint32_t body);
bool ReleasingFrom(uint32_t body,uint32_t wearerGroup);
bool Shot(V3 from,V3 axis,float range,V3 *point,uint32_t *body);
void BulletImpulse(uint32_t body,V3 direction,V3 point,float damage);
void BodyContact(uint32_t propBody);
void FlightContact(uint32_t body,unsigned layer,V3 normal,float approach);
void Reset();
}
