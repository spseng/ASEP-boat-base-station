cmake_minimum_required(VERSION 3.22)
project(asep_base_station LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

option(BASESTATION_BUILD_GUI   "Build the SDL3 + Dear ImGui application" ON)
option(BASESTATION_BUILD_TESTS "Build the unit tests"                    ON)
option(BASESTATION_BUILD_SIM   "Build the fake base station simulator"   ON)

# ---------------------------------------------------------------------------
# Shared code from the ASEP-boat repository (git submodule, read-only).
# wirelink's own CMakeLists needs ament_cmake (ROS), so the pieces used here
# are compiled directly instead of through add_subdirectory.
# ---------------------------------------------------------------------------
set(ASEP_BOAT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/ASEP-boat)
if(NOT EXISTS ${ASEP_BOAT_DIR}/shared/wirelink/src/framing.cpp)
  message(FATAL_ERROR
    "external/ASEP-boat is missing. Run: git submodule update --init")
endif()

add_library(wirelink STATIC ${ASEP_BOAT_DIR}/shared/wirelink/src/framing.cpp)
target_include_directories(wirelink PUBLIC ${ASEP_BOAT_DIR}/shared/wirelink/include)
target_compile_features(wirelink PUBLIC cxx_std_17)

add_library(boat_defs INTERFACE)
target_include_directories(boat_defs INTERFACE ${ASEP_BOAT_DIR}/shared/boat_defs/include)

# ---------------------------------------------------------------------------
# Warnings for code owned by this repository.
# ---------------------------------------------------------------------------
add_library(basestation_warnings INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(basestation_warnings INTERFACE -Wall -Wextra -Wpedantic)
endif()

find_package(Threads REQUIRED)

# Protocol layer: codec + proposed LoRa messages + base-station-local messages.
add_library(basestation_proto INTERFACE)
target_include_directories(basestation_proto INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src/proto)
target_link_libraries(basestation_proto INTERFACE wirelink boat_defs)

# Core: everything that is not GUI. Testable headless.
add_library(basestation_core STATIC
  src/core/src/serial_port.cpp
  src/core/src/frame_splitter.cpp
  src/core/src/link_session.cpp
  src/core/src/frame_log.cpp
  src/core/src/fleet_model.cpp
  src/core/src/geo.cpp
  src/core/src/teleop.cpp
  src/core/src/teleop_sender.cpp
  src/core/src/commander.cpp
)
target_include_directories(basestation_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/core)
target_link_libraries(basestation_core
  PUBLIC basestation_proto Threads::Threads
  PRIVATE basestation_warnings)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_libraries(basestation_core PUBLIC util)  # openpty
endif()

include(cmake/Dependencies.cmake)

if(BASESTATION_BUILD_SIM)
  add_subdirectory(src/sim)
endif()

if(BASESTATION_BUILD_GUI)
  add_subdirectory(src/app)
endif()

if(BASESTATION_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
