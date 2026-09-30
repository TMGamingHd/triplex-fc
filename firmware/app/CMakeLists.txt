cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tfc_app LANGUAGES C CXX)

# core/ is consumed as-is: header-only, no heap, no exceptions, no RTTI.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../core/include)
target_sources(app PRIVATE src/main.cpp)
