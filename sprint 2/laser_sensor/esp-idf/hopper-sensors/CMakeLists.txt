# ---------------------------------------------------------------------------
# Top-level project CMakeLists.txt
#
# Every ESP-IDF project needs exactly this shape:
#   1. a minimum CMake version
#   2. include($ENV{IDF_PATH}/tools/cmake/project.cmake)  -- pulls in the whole
#      IDF build system, which is what finds your components/ directory
#   3. project(<name>)                                    -- must come last
#
# Unlike the Arduino IDE, nothing here is implicit: the build system discovers
# components/ automatically, but each component lists its own source files.
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(hopper-sensors)
