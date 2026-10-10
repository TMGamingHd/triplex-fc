cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tfc_app LANGUAGES C CXX)

# core/ is consumed as-is: header-only, no heap, no exceptions, no RTTI.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../core/include ${CMAKE_CURRENT_LIST_DIR}/../common)
target_sources(app PRIVATE src/main.cpp)

# The release this image is (ADR-021): the first four hex digits of the source's git hash, unless CONFIG_TFC_RELEASE_ID names one. Two images built from different commits report
# different releases in their heartbeat; the golden release is simply the image built from an older tagged commit. Outside a git checkout it is 0, "not reported".
execute_process(COMMAND git rev-parse --short=4 HEAD WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} OUTPUT_VARIABLE TFC_GIT_SHORT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE TFC_GIT_RC)
if(TFC_GIT_RC EQUAL 0 AND TFC_GIT_SHORT MATCHES "^[0-9a-f][0-9a-f][0-9a-f][0-9a-f]$")
  target_compile_definitions(app PRIVATE TFC_RELEASE_HASH_AUTO=0x${TFC_GIT_SHORT})
else()
  target_compile_definitions(app PRIVATE TFC_RELEASE_HASH_AUTO=0)
endif()

# Another vehicle's tables: `west build ... -DTFC_TABLES_HEADER=/path/flight_tables.hpp` (main.cpp includes it in place of the committed one). Unset: the committed tables, as always.
if(DEFINED TFC_TABLES_HEADER)
  target_compile_definitions(app PRIVATE "TFC_FLIGHT_TABLES_HEADER=\"${TFC_TABLES_HEADER}\"")
endif()

# Replicas must compute the same bits as each other and as the host (ADR-006): no fused multiply-add (-ffast-math stays off).
target_compile_options(app PRIVATE -ffp-contract=off)
