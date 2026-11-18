#pragma once
#include <cstdint>
#include "mathx.h"

// A temporary position used only while the native car sweep tests a pedestrian.
struct PedestrianPositionSwap {
    uint8_t *element;
    void *model;
    V3 before, temporary;
    bool committed = false;

    bool Restore(void *currentModel, V3 &position) const {
        if (committed || currentModel != model ||
            position.x != temporary.x || position.y != temporary.y || position.z != temporary.z)
            return false;
        position = before;
        return true;
    }
};
