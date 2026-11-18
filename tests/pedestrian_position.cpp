#include "pedestrian_position.h"
#include <cstdio>

int main() {
    void *model = reinterpret_cast<void *>(1);
    PedestrianPositionSwap swap{nullptr, model, {0, 0, 0}, {2, 0, 0}};
    V3 current = swap.temporary;
    if (!swap.Restore(model, current) || current.x != 0) return 1;

    // A real hit commits the animated position, even if the native call leaves the exact temporary value.
    current = swap.temporary;
    swap.committed = true;
    if (swap.Restore(model, current) || current.x != 2) return 2;

    // Native movement and a recycled model belong to the native call.
    swap.committed = false;
    current = {3, 0, 0};
    if (swap.Restore(model, current) || current.x != 3) return 3;
    current = swap.temporary;
    if (swap.Restore(reinterpret_cast<void *>(2), current) || current.x != 2) return 4;

    // Two reentrant sweeps that both observe the hit must both retain its committed position.
    PedestrianPositionSwap inner{nullptr, model, swap.temporary, {4, 0, 0}, true};
    swap.committed = true;
    current = inner.temporary;
    if (inner.Restore(model, current) || swap.Restore(model, current) || current.x != 4) return 5;
    std::puts("PASS: temporary positions restore, committed hits and native/model changes survive");
}
