add_executable(basestation_tests
  test_codec.cpp
  test_frame_splitter.cpp
  test_frame_log.cpp
  test_link_session.cpp
  test_fleet_model.cpp
  test_geo.cpp
  test_teleop.cpp
  test_commander.cpp
  test_sim_integration.cpp
  test_map_tiles.cpp
  test_tile_service.cpp
  test_msg_catalog.cpp
)
target_link_libraries(basestation_tests PRIVATE basestation_core basestation_warnings Catch2::Catch2WithMain)
# The simulator integration test launches the simulator binary.
if(TARGET asep_fake_base)
  add_dependencies(basestation_tests asep_fake_base)
  target_compile_definitions(basestation_tests PRIVATE
    ASEP_FAKE_BASE_PATH="$<TARGET_FILE:asep_fake_base>")
endif()

include(Catch)
catch_discover_tests(basestation_tests)
