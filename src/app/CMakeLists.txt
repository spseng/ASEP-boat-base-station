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

add_executable(asep_base_station
  main.cpp
  cli.cpp
  app.cpp
)
target_link_libraries(asep_base_station PRIVATE basestation_core imgui basestation_warnings)

if(APPLE)
  set_target_properties(asep_base_station PROPERTIES MACOSX_BUNDLE OFF)
endif()
