// The static world the bodies meet, from the visual models the game draws (not from the game's 2002 collision boxes):
// the loader's scene_focus / scene_pieces (its walk of the scene, its scans every 0.25 s, the meshes read from LS3D's
// own buffers), turned into pieces the rigid-body world cooks once a shape. mafia-ragdoll's scenery.h, the part
// jolt_world uses.
#pragma once
#include <string>
#include <vector>
#include "smoke.h"

namespace scenery {

enum Consumer { kRagdoll, kConsumers };
void SetFocus(Consumer c, const V3 &lo, const V3 &hi);
void ClearFocus(Consumer c);

// Game thread, every tick: the host's newest pieces taken when its scan changed them.
void Tick();
// The static pieces of the last scan (their shapes valid until the next scan or Clear).
const std::vector<smoke::SceneryPiece> &Pieces();
unsigned Scans();     // bumped by every new set of Pieces()
bool FirstScan();     // the last set was the first since Clear: make everything at once
unsigned Epoch();     // bumped by Clear: shape keys may be reused, consumers drop what they made from them
// Every visual around this box is among the pieces (the loader's scene_ready) and this set holds them.
bool Covers(const V3 &lo, const V3 &hi);
unsigned FullScan();  // the number (Scans()) of the last set that covered the focus whole; 0: none since Clear
std::string CoverReport(const V3 &lo, const V3 &hi);
void Clear();         // the mission's world goes: every piece and shape forgotten
std::string Describe();

} // namespace scenery
