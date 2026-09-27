# Third-party dependencies, fetched at configure time so macOS and Ubuntu
# build the same versions without relying on system packages.
include(FetchContent)

if(BASESTATION_BUILD_GUI)
  # --- SDL3 ---------------------------------------------------------------
  # Ubuntu 24.04 only packages SDL2, so SDL3 is always built from source.
  set(SDL_SHARED OFF CACHE BOOL "" FORCE)
  set(SDL_STATIC ON  CACHE BOOL "" FORCE)
  set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
  set(SDL_TESTS OFF CACHE BOOL "" FORCE)
  set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(SDL_INSTALL OFF CACHE BOOL "" FORCE)
  # Subsystems this app never uses.
  set(SDL_AUDIO OFF CACHE BOOL "" FORCE)
  set(SDL_CAMERA OFF CACHE BOOL "" FORCE)
  set(SDL_GPU OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG        release-3.4.16
    GIT_SHALLOW    TRUE
    EXCLUDE_FROM_ALL)

  # --- Dear ImGui (docking branch) -----------------------------------------
  FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.92.9-docking
    GIT_SHALLOW    TRUE)

  # --- ImPlot --------------------------------------------------------------
  FetchContent_Declare(implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG        v1.0
    GIT_SHALLOW    TRUE)

  # --- stb_image (PNG / JPEG decoding of map tiles) -------------------------
  # stb has no releases; pinned to a commit.
  FetchContent_Declare(stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG        2c980bb59875b0d32144a71867fbdebb2f77cd20)

  # imgui, implot and stb ship no CMakeLists.txt, so MakeAvailable only
  # downloads them; their targets are defined in src/app/CMakeLists.txt.
  FetchContent_MakeAvailable(SDL3 imgui implot stb)
endif()

if(BASESTATION_BUILD_TESTS)
  FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.9.1
    GIT_SHALLOW    TRUE
    EXCLUDE_FROM_ALL)
  FetchContent_MakeAvailable(Catch2)
endif()
