# mafia-ragdoll-mml: physics ragdolls for Mafia (2002), a MafiaModLoader mod.
#
#   cmake --preset x86                  (once; MafiaModLoader's repository beside this one, or -DMML_ROOT=<it>)
#   cmake --build --preset x86-release  -> build/x86/out/mml/mods/ragdoll/ (ragdoll.dll, mod.json, config.json)
#   python tools/deploy.py              -> into the labs of MafiaModLoader's mml.config.json, reloaded in a running game
#
# The SDK (mml/mml.h, mml/game/*) and Jolt Physics are MafiaModLoader's: its sdk/include and its pinned Jolt
# (third_party/_deps/jolt, fetched by its tools/deps.py), built here with the same configuration as its own mods.
cmake_minimum_required(VERSION 3.24)
project(mafia_ragdoll_mml LANGUAGES C CXX)

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(FATAL_ERROR "the game is 32-bit: cmake --preset x86")
endif()

set(MML_ROOT "${CMAKE_SOURCE_DIR}/../MafiaModLoader" CACHE PATH "MafiaModLoader's repository: its SDK and Jolt")
cmake_path(ABSOLUTE_PATH MML_ROOT NORMALIZE)
if(NOT EXISTS ${MML_ROOT}/sdk/include/mml/mml.h)
    message(FATAL_ERROR "no MafiaModLoader at ${MML_ROOT} (-DMML_ROOT=<its repository>)")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")   # (static CRT, as the loader's mods)
add_compile_options(/W3 /utf-8 /permissive- /Zi)
add_compile_definitions(_CRT_SECURE_NO_WARNINGS WIN32_LEAN_AND_MEAN NOMINMAX)
add_link_options(/DEBUG /OPT:REF /OPT:ICF)

# Jolt Physics: MafiaModLoader's pin, configured as it configures it for its mods (SSE2: any x86 CPU; no GPU compute,
# profiler, debug renderer or object streams)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(COMMAND ${Python3_EXECUTABLE} ${MML_ROOT}/tools/deps.py jolt RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "MafiaModLoader's tools/deps.py jolt failed")
endif()
foreach(o USE_SSE4_1 USE_SSE4_2 USE_AVX USE_AVX2 USE_AVX512 USE_LZCNT USE_TZCNT USE_F16C USE_FMADD
          JPH_USE_DX12 JPH_USE_VK JPH_USE_MTL INTERPROCEDURAL_OPTIMIZATION FLOATING_POINT_EXCEPTIONS_ENABLED
          PROFILER_IN_DEBUG_AND_RELEASE DEBUG_RENDERER_IN_DEBUG_AND_RELEASE ENABLE_OBJECT_STREAM ENABLE_INSTALL
          ENABLE_ALL_WARNINGS)
    set(${o} OFF)
endforeach()
set(USE_STATIC_MSVC_RUNTIME_LIBRARY ON)
add_subdirectory(${MML_ROOT}/third_party/_deps/jolt/Build ${CMAKE_BINARY_DIR}/jolt EXCLUDE_FROM_ALL)
target_compile_options(Jolt PUBLIC /Zc:__cplusplus)

file(GLOB sources CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/*.cpp)
add_library(ragdoll SHARED ${sources})
set(dest ${CMAKE_BINARY_DIR}/out/mml/mods/ragdoll)
set_target_properties(ragdoll PROPERTIES OUTPUT_NAME ragdoll RUNTIME_OUTPUT_DIRECTORY ${dest}/$<0:>)
# (sdk/include/mml/game too: the ported code includes "game.h", "game_build.h", "mathx.h" as mafia-ragdoll had them)
target_include_directories(ragdoll PRIVATE ${MML_ROOT}/sdk/include ${MML_ROOT}/sdk/include/mml/game ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(ragdoll PRIVATE Jolt)
target_compile_options(ragdoll PRIVATE /Oy-)   # (the loader's sampler and crash reports walk frames by EBP)
target_link_options(ragdoll PRIVATE /MAP:${CMAKE_BINARY_DIR}/ragdoll.map)   # (a function by its offset: tools/where.py)
add_custom_command(TARGET ragdoll POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/mod/mod.json ${dest}/mod.json
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/mod/config.json ${dest}/config.json)

# Small geometry regression; opt-in so it adds no work to normal mod builds.
add_executable(ragdoll_trunk_axes EXCLUDE_FROM_ALL tests/trunk_axes.cpp)
target_include_directories(ragdoll_trunk_axes PRIVATE ${CMAKE_SOURCE_DIR}/src ${MML_ROOT}/sdk/include/mml/game)

add_executable(ragdoll_pedestrian_motion EXCLUDE_FROM_ALL tests/pedestrian_motion.cpp src/ragdoll.cpp src/collision.cpp)
target_include_directories(ragdoll_pedestrian_motion PRIVATE ${CMAKE_SOURCE_DIR}/src ${MML_ROOT}/sdk/include ${MML_ROOT}/sdk/include/mml/game)

add_executable(ragdoll_pedestrian_position EXCLUDE_FROM_ALL tests/pedestrian_position.cpp)
target_include_directories(ragdoll_pedestrian_position PRIVATE ${CMAKE_SOURCE_DIR}/src ${MML_ROOT}/sdk/include/mml/game)

add_executable(ragdoll_world_step EXCLUDE_FROM_ALL tests/world_step.cpp src/scenery.cpp src/jolt_world.cpp)
target_include_directories(ragdoll_world_step PRIVATE ${CMAKE_SOURCE_DIR}/src ${MML_ROOT}/sdk/include ${MML_ROOT}/sdk/include/mml/game)
target_link_libraries(ragdoll_world_step PRIVATE Jolt)
