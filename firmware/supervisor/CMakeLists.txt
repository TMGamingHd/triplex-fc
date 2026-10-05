cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tfc_supervisor LANGUAGES C CXX)

# The supervisor shares no code with core/ (TFC-SUP-001): only its own logic, supervisor/include.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../supervisor/include)
target_sources(app PRIVATE src/main.cpp src/usb_init.c)
