# The simulator deliberately does not link basestation_core: it parses with
# upstream wirelink and encodes with basestation::codec only, so it can
# cross-check core instead of sharing its bugs.
add_executable(asep_fake_base fake_base.cpp sim_world.cpp)
target_link_libraries(asep_fake_base PRIVATE basestation_proto basestation_warnings)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_libraries(asep_fake_base PRIVATE util)  # openpty
endif()

add_executable(asep_log_dump log_dump.cpp)
target_link_libraries(asep_log_dump PRIVATE basestation_core basestation_warnings)
