// The shapes the rigid-body world takes from the game (mafia-ragdoll's smoke.h, its two structures kept: its PhysX
// smoke is not a part of this mod): the static scenery's pieces (scenery.cpp, from the loader's scene_pieces) and
// what the game moves (cars, people's body parts) as movers.
#pragma once
#include <cstdint>
#include <vector>
#include "mathx.h"

namespace smoke {

// A static object of the scene: a triangle mesh in its own pose.
struct SceneryPiece {
    const void *key = nullptr;       // stable per object
    unsigned version = 0;            // changes when its pose or shape changes: its body is made again
    const void *shapeKey = nullptr;  // pieces with the same shapeKey share one cooked mesh: same pos and tris
    V3 t{}, ax[3]{};                 // pose: world = t + sum local_i * ax[i] (unit, orthogonal, right-handed axes)
    const std::vector<V3> *pos = nullptr;          // the shape, in the piece's frame
    const std::vector<uint32_t> *tris = nullptr;   // three indices per triangle
};

// Something the game moves: a human's body part, a car. Each becomes a kinematic body (or a car's body of its mass)
// that follows it every tick.
struct Mover {
    enum Kind { CAPSULE, SPHERE, BOX, HULL };
    const void *key = nullptr;   // stable identity from tick to tick
    Kind kind = CAPSULE;
    V3 p0{}, p1{};               // CAPSULE: segment ends (world); SPHERE: p0 = centre
    float r = 0;                 // CAPSULE, SPHERE: radius
    V3 c{}, ax[3]{};             // BOX, HULL: pose, centre and unit axes (rows: world = c + sum local_i * ax[i])
    V3 h{};                      // BOX: half extents
    const std::vector<V3> *hull = nullptr;   // HULL: points in the pose frame; alive and unchanged while the key uses it
    // HULL: the same points split by model part: a compound of one convex hull per part (a single hull of a whole
    // car fills the space under its bumpers)
    const std::vector<std::vector<V3>> *hullParts = nullptr;
    const std::vector<uint64_t> *hullRevisions = nullptr; // identity of each unchanged cooked hull
    unsigned shapeVersion = 0;   // HULL: changes when the points change (the shape is cooked again)
    bool hullIsTriangles = false;   // HULL: `hull` is a triangle list (three points per triangle)
    const void *owner = nullptr;    // the human a body-part capsule belongs to
    float partMass = 0;             // CAPSULE: that anatomical segment's mass, not the whole person's
    float mass = 0;                 // HULL of a car: its mass (kg): a body of that mass meeting the bodies both ways
    uint64_t wheelParts = 0;        // HULL of a car: which of hullParts are its wheels (bit i: part i)
    const std::vector<uint32_t> *nativeWheels = nullptr; // per hull part: native wheel index + 1, 0 for other parts
    // HULL of a vehicle with no body of its own mass (a tram): its exact shape, `hull`'s triangles from both sides
    bool hullAsMesh = false;
};

} // namespace smoke
