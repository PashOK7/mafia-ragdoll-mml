// The cars (and trams) as the rigid-body world sees them: each a body made from its own model's meshes as the game
// draws them (the loader's actor_meshes: one convex hull a model part), placed by its model frame. Game thread.
#pragma once
#include <vector>
#include "mathx.h"
#include "smoke.h"

namespace vehicles {

// The vehicles (cars, trams) within [lo, hi] (world) as movers: a hull of their own model's parts (a car's mass from
// its C_car: the rigid-body world makes it a body of that mass; a tram's exact mesh, kinematic).
void GatherVehicles(const V3 &lo, const V3 &hi, std::vector<smoke::Mover> &out);
// A known striking vehicle, even before discovery's next refresh. Uses the same native model geometry.
bool MoverOf(void *car, smoke::Mover *out);
// The vehicles within `radius` of a point (actors of type 4 or 8), from the loader's world_near.
void ListVehicles(const V3 &at, float radius, std::vector<void *> &out);
void Clear();                 // the mission's world goes: every body read so far
void Forget(void *car);       // the car is destroyed
// A car's pose (its model frame: origin and orthonormal axes). False when it has no model (or reading it faulted).
bool Pose(void *car, V3 *origin, V3 axes[3]);
// Its model's triangles (three points each, in the axes Pose gives) and its parts (the points of one hull each).
bool ModelTriangles(void *car, const std::vector<V3> **tris);
bool HullParts(void *car, const std::vector<std::vector<V3>> **parts, unsigned *version);

} // namespace vehicles
