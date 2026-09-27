# Dear ImGui + ImPlot as one static library, using the SDL3 platform backend
# and the SDL_Renderer backend (Metal on macOS, OpenGL/software on Linux).
set(IMGUI_DIR  ${imgui_SOURCE_DIR})
set(IMPLOT_DIR ${implot_SOURCE_DIR})

add_library(imgui STATIC
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_demo.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_sdl3.cpp
  ${IMGUI_DIR}/backends/imgui_impl_sdlrenderer3.cpp
  ${IMPLOT_DIR}/implot.cpp
  ${IMPLOT_DIR}/implot_items.cpp
  ${IMPLOT_DIR}/implot_demo.cpp
)
target_include_directories(imgui SYSTEM PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends ${IMPLOT_DIR})
target_link_libraries(imgui PUBLIC SDL3::SDL3-static)

# stb_image, compiled once without this project's warning flags.
add_library(stb_image STATIC stb_image_impl.c)
target_include_directories(stb_image SYSTEM PUBLIC ${stb_SOURCE_DIR})

add_executable(asep_base_station
  main.cpp
  cli.cpp
  app.cpp
  gamepad.cpp
  settings.cpp
  theme.cpp
  ui_util.cpp
  ui_toolbar.cpp
  ui_fleet_view.cpp
  ui_boats.cpp
  ui_commands.cpp
  ui_teleop.cpp
  ui_plots.cpp
  ui_link.cpp
  ui_events.cpp
  map_layer.cpp
  ui_map.cpp
)
target_link_libraries(asep_base_station PRIVATE basestation_core imgui stb_image basestation_warnings)
target_compile_definitions(asep_base_station PRIVATE BASESTATION_VERSION="${PROJECT_VERSION}")

if(APPLE)
  set_target_properties(asep_base_station PROPERTIES MACOSX_BUNDLE OFF)
endif()
