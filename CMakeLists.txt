cmake_minimum_required(VERSION 3.16)
project(triplex_fc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Replicas must compute the same bits (ADR-006), and the host must compute the same bits as the target: no fused multiply-add,
# which GCC would otherwise contract on any target that has one. (-ffast-math stays off.)
add_compile_options(-ffp-contract=off)

# Portable flight-logic core: header-only, no heap, no exceptions, no RTTI.
add_library(tfc_core INTERFACE)
target_include_directories(tfc_core INTERFACE core/include)
target_compile_features(tfc_core INTERFACE cxx_std_17)

option(TFC_SANITIZE "Build host tests with ASan+UBSan" OFF)

add_executable(tfc_tests
  tests/test_main.cpp
  tests/test_crc_protocol.cpp
  tests/test_voter.cpp
  tests/test_fault_monitor.cpp
  tests/test_redundancy.cpp
  tests/test_fuzz.cpp
  tests/test_recovery.cpp
  tests/test_ground.cpp
  tests/test_triplex_scenarios.cpp
  tests/test_hw_logic.cpp
  tests/test_loop.cpp
  tests/test_act.cpp
  tests/test_protocol_v2.cpp
  tests/test_vehicle.cpp
  tests/test_flight.cpp
  tests/test_sync_clock.cpp
  tests/test_act_ground.cpp
  tests/test_runner.cpp
  tests/test_pico.cpp
  tests/test_pico_app.cpp
  tests/test_pico_fuzz.cpp
  tests/test_realism.cpp
  tests/test_pad.cpp
  tests/test_launch.cpp
  tests/test_supervisor_clock.cpp
  tests/test_resync.cpp)
target_link_libraries(tfc_tests PRIVATE tfc_core)
target_include_directories(tfc_tests PRIVATE tests tools/vehicle sim/vehicle firmware/app/src supervisor/include)
target_compile_options(tfc_tests PRIVATE
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wdouble-promotion -Werror
  -fno-exceptions -fno-rtti)
if(TFC_SANITIZE)
  target_compile_options(tfc_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(tfc_tests PRIVATE -fsanitize=address,undefined)
endif()

# Host tool: replays recorded bus traffic through core/ (see tools/replay/replay.cpp).
add_executable(tfc_replay tools/replay/replay.cpp)
target_link_libraries(tfc_replay PRIVATE tfc_core)
target_compile_options(tfc_replay PRIVATE
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wdouble-promotion -Werror
  -fno-exceptions -fno-rtti)
if(TFC_SANITIZE)
  target_compile_options(tfc_replay PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(tfc_replay PRIVATE -fsanitize=address,undefined)
endif()

# The flight computers' pitch program and gain schedule, designed on the vehicle model and written out as a header (tools/vehicle/gen_tables.cpp).
add_executable(tfc_gen_tables tools/vehicle/gen_tables.cpp)
target_link_libraries(tfc_gen_tables PRIVATE tfc_core)
target_include_directories(tfc_gen_tables PRIVATE sim/vehicle)
target_compile_options(tfc_gen_tables PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wdouble-promotion -Werror -fno-exceptions -fno-rtti)

# The vehicle simulator on a SocketCAN bus (tools/sim/tfc_simd.cpp): it follows SYNC, publishes the sensor inputs, and flies the vehicle with ACT's output.
add_executable(tfc_simd tools/sim/tfc_simd.cpp)
target_link_libraries(tfc_simd PRIVATE tfc_core)
target_include_directories(tfc_simd PRIVATE sim/vehicle)
target_compile_options(tfc_simd PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wdouble-promotion -Werror -fno-exceptions -fno-rtti)

# Sensitivity of the closed loop to sensor errors, a dispersed vehicle and late frames (tools/sim/tfc_sens.cpp).
find_package(Threads REQUIRED)
add_executable(tfc_sens tools/sim/tfc_sens.cpp)
target_link_libraries(tfc_sens PRIVATE tfc_core Threads::Threads)
target_include_directories(tfc_sens PRIVATE sim/vehicle)
target_compile_options(tfc_sens PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wdouble-promotion -Werror -fno-exceptions -fno-rtti)

# How often should the replicas resynchronise? (tools/sim/tfc_resync.cpp, TS-23)
add_executable(tfc_resync tools/sim/tfc_resync.cpp)
target_link_libraries(tfc_resync PRIVATE tfc_core Threads::Threads)
target_include_directories(tfc_resync PRIVATE sim/vehicle)
target_compile_options(tfc_resync PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wdouble-promotion -Werror -fno-exceptions -fno-rtti)

# Flight-code gate (docs/CODING_STANDARD.md): the strictest warning set, as errors, over the whole core, and every header
# compiled on its own so none depends on include order. Part of the default build, so CI fails on a regression.
set(TFC_STRICT_FLAGS -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Wdouble-promotion -Wold-style-cast
    -Wcast-align -Wcast-qual -Wnull-dereference -Wformat=2 -Wundef -Wzero-as-null-pointer-constant -Wswitch-enum
    -Wswitch-default -Wfloat-equal -Wmissing-declarations -Werror -fno-exceptions -fno-rtti)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  list(APPEND TFC_STRICT_FLAGS -Wuseless-cast -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wstack-usage=2048)
endif()
file(GLOB TFC_CORE_HEADERS ${CMAKE_SOURCE_DIR}/core/include/tfc/*.hpp)
set(TFC_STRICT_SOURCES core/check/strict_check.cpp)
foreach(hdr ${TFC_CORE_HEADERS})
  get_filename_component(hdr_name ${hdr} NAME_WE)
  file(WRITE ${CMAKE_BINARY_DIR}/strict/self_contained_${hdr_name}.cpp
       "#include \"tfc/${hdr_name}.hpp\"\nnamespace { [[maybe_unused]] constexpr int kAnchor = 0; }\n")
  list(APPEND TFC_STRICT_SOURCES ${CMAKE_BINARY_DIR}/strict/self_contained_${hdr_name}.cpp)
endforeach()
add_library(tfc_strict_check OBJECT ${TFC_STRICT_SOURCES})
target_link_libraries(tfc_strict_check PRIVATE tfc_core)
target_compile_options(tfc_strict_check PRIVATE ${TFC_STRICT_FLAGS} -O2)

# Cost of one frame of the manager (host numbers; the flight WCET is measured on the target).
add_executable(tfc_bench tools/bench/bench_end_frame.cpp)
target_link_libraries(tfc_bench PRIVATE tfc_core)
target_compile_options(tfc_bench PRIVATE -O2 -fno-exceptions -fno-rtti -Wall -Wextra -Wconversion -Werror -fstack-usage)

enable_testing()
add_test(NAME tfc_tests COMMAND tfc_tests)

# End-to-end: Python virtual peers -> candump log -> tfc_replay (real core/ code).
find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
  add_test(NAME peers_e2e
    COMMAND ${CMAKE_COMMAND} -E env TFC_REPLAY_BIN=$<TARGET_FILE:tfc_replay>
            ${Python3_EXECUTABLE} -m unittest discover -s tests -t .
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/sim)
endif()
