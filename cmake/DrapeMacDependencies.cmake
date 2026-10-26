# Mac-only dependencies (Phase B spec 3.2): metal-cpp, the prebuilt Filament release, the CC0 studio HDRI, and Qt.
# Downloads land in the build folder and are SHA-256 checked; nothing is installed on the system except Qt.

set(DRAPE_MAC_DEPS ${CMAKE_BINARY_DIR}/_deps)

# metal-cpp, pinned to release/metal-cpp_macOS26.4_iOS26.4 (header-only).
FetchContent_Declare(metal_cpp
  GIT_REPOSITORY https://github.com/apple/metal-cpp
  GIT_TAG c595afef4a5dc388f4047cd0c69f9e7f9468d9ed)
FetchContent_MakeAvailable(metal_cpp)
add_library(drape_metal_cpp INTERFACE)
target_include_directories(drape_metal_cpp SYSTEM INTERFACE ${metal_cpp_SOURCE_DIR})
target_link_libraries(drape_metal_cpp INTERFACE "-framework Metal" "-framework Foundation" "-framework QuartzCore")

# Pinned download with a SHA-256 check; skipped when the file is already there and matches.
function(drape_fetch_file url hash dest)
  if(EXISTS ${dest})
    file(SHA256 ${dest} have)
    if(have STREQUAL hash)
      return()
    endif()
  endif()
  message(STATUS "Downloading ${url}")
  file(DOWNLOAD ${url} ${dest} EXPECTED_HASH SHA256=${hash} STATUS st)
  list(GET st 0 code)
  if(NOT code EQUAL 0)
    message(FATAL_ERROR "Download failed: ${url} (${st})")
  endif()
endfunction()

# Filament v1.78.1, official prebuilt macOS release.
set(DRAPE_FILAMENT_TGZ ${DRAPE_MAC_DEPS}/filament-v1.78.1-mac.tgz)
drape_fetch_file(https://github.com/google/filament/releases/download/v1.78.1/filament-v1.78.1-mac.tgz
                 d27efabcc89d4547add654bbcd40679f07d1c5591efe0e74a7ebc0257fbc8f95 ${DRAPE_FILAMENT_TGZ})
set(DRAPE_FILAMENT_DIR ${DRAPE_MAC_DEPS}/filament-v1.78.1/filament)
if(NOT EXISTS ${DRAPE_FILAMENT_DIR}/include/filament/Engine.h)
  file(ARCHIVE_EXTRACT INPUT ${DRAPE_FILAMENT_TGZ} DESTINATION ${DRAPE_MAC_DEPS}/filament-v1.78.1)
endif()
set(DRAPE_MATC ${DRAPE_FILAMENT_DIR}/bin/matc)
set(DRAPE_CMGEN ${DRAPE_FILAMENT_DIR}/bin/cmgen)
add_library(drape_filament INTERFACE)
target_include_directories(drape_filament SYSTEM INTERFACE ${DRAPE_FILAMENT_DIR}/include)
foreach(lib filament backend bluegl bluevk filabridge filaflat utils geometry ibl-lite ktxreader image smol-v abseil zstd perfetto)
  target_link_libraries(drape_filament INTERFACE ${DRAPE_FILAMENT_DIR}/lib/arm64/lib${lib}.a)
endforeach()
target_link_libraries(drape_filament INTERFACE "-framework Metal" "-framework QuartzCore" "-framework Cocoa" "-framework OpenGL"
                      "-framework CoreVideo" "-framework IOKit")

# Poly Haven studio_small_09 (CC0), 2K; turned into an IBL with cmgen by drape_render.
set(DRAPE_HDRI ${DRAPE_MAC_DEPS}/studio_small_09_2k.hdr)
drape_fetch_file(https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/2k/studio_small_09_2k.hdr
                 36724313c0fc66dab126ff90f081b85a0b1ef47be65eda1417bd20b033f0573f ${DRAPE_HDRI})

# Qt 6.12 Widgets for the app (Homebrew qtbase/qt). Optional: without it everything but the app builds.
find_package(Qt6 6.12 COMPONENTS Widgets QUIET)
if(Qt6_FOUND)
  set(DRAPE_HAVE_QT ON)
else()
  set(DRAPE_HAVE_QT OFF)
  message(WARNING "Qt 6.12 (Widgets) not found: the Drape app is not built. Install Homebrew qtbase.")
endif()
