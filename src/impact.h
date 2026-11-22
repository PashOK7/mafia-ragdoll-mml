#pragma once
#include "mathx.h"

inline V3 BlastVelocity(V3 point,V3 centre,float radius,float speed,float lift) {
    const V3 offset=point-centre;
    const float distance=Len(offset);
    if (!(distance<radius && speed>0.f)) return {};
    const float falloff=std::pow(1.f-distance/radius,1.5f);
    const V3 up{0,1,0};
    return Norm(Norm(offset,up)+up*lift,up)*(speed*falloff);
}
