# Command-line debugging tools. Core only: no SDL needed.
add_executable(wl_mon wl_mon.cpp)
target_link_libraries(wl_mon PRIVATE basestation_core basestation_warnings)
