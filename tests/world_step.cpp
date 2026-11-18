// Production scenery and Jolt step, with two instances of one mesh supplied by the SDK.
#include "mod.h"
#include "scenery.h"
#include "jolt_world.h"
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mod {
const MmlHost *H;
MmlMod *M;
void Log(const char *, ...) {}
}

int main() {
    static MmlMeshPiece input[2]{};
    const float pos[] = {0,0,0, 0,0,1, 1,0,0};
    const uint32_t index[] = {0,1,2};
    for (auto &p : input) {
        p.visual = &p;
        p.pos = pos; p.index = index; p.vertices = 3; p.triangles = 1;
        p.world[0] = p.world[5] = p.world[10] = p.world[15] = 1;
    }
    MmlHost host{};
    host.scene_focus = [](MmlMod *, const float *, const float *) { return 1; };
    host.scene_ready = host.scene_focus;
    host.world_shapes = [](MmlMod *, const float *, const float *, uint32_t, MmlShape *, int) { return 0; };
    host.scene_pieces = [](MmlMod *, MmlMeshPiece *out, int cap) {
        if (out) std::copy_n(input, std::min(cap, 2), out);
        return 2;
    };
    host.world_water = [](MmlMod *, const float *, MmlWater *w) { w->height = 0; return 1; };
    mod::H = &host;
    scenery::SetFocus(scenery::kRagdoll, {-5,-5,-5}, {5,5,5});
    for (int dry : {0,1,0}) {
        for (auto &p : input) { p.world[13] = &p == &input[dry] ? 2.f : 0.f; ++p.version; }
        scenery::Tick();
        const auto &pieces = scenery::Pieces();
        if (pieces.size() != 1 || pieces.front().key != &input[dry]) {
            std::puts("FAIL: waterline instances aliased");
            return 1;
        }
    }
    std::string error;
    if (!jolt_world::Start(nullptr, {}, &error)) { std::puts(error.c_str()); return 1; }
    smoke::Mover car;
    car.kind = smoke::Mover::BOX; car.key = &car; car.mass = 1200;
    car.h = {1,.5f,2}; car.c = {0,10,0};
    car.ax[0] = {1,0,0}; car.ax[1] = {0,1,0}; car.ax[2] = {0,0,1};
    jolt_world::Step(.016f, {car}, nullptr);
    JPH::BodyIDVector ids;
    jolt_world::System()->GetBodies(ids);
    auto id = std::find_if(ids.begin(), ids.end(), [&](const auto &v) { return jolt_world::KinematicOwner(v) == &car; });
    if (id == ids.end()) { std::puts("FAIL: missing driven car"); return 1; }
    for (int step = 1; step <= 6; ++step) {
        car.c.x = static_cast<float>(step);
        jolt_world::Step(.100f, {car}, nullptr);
        const float actual = jolt_world::Bodies().GetPosition(*id).GetX();
        if (std::abs(actual - car.c.x) > .001f) {
            std::printf("FAIL: 100 ms step reached %.4f, expected %.4f\n", actual, car.c.x);
            return 1;
        }
    }
    jolt_world::Stop();
    std::puts("PASS: waterline is per placement and six 100 ms steps reach the native car pose");
    return 0;
}
