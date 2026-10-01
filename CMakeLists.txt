cmake_minimum_required(VERSION 3.16)
project(triplex_fc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
  tests/test_triplex_scenarios.cpp)
target_link_libraries(tfc_tests PRIVATE tfc_core)
target_include_directories(tfc_tests PRIVATE tests)
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
