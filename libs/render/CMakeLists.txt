# Portable render math (normals, camera, image checks); the Filament renderer is added for Apple builds in Phase B Task 12.
drape_add_library(drape_render_math
  SOURCES src/normals.cpp src/orbit_camera.cpp src/image_metrics.cpp
  PUBLIC_DEPS drape_core
  PRIVATE_DEPS miniz)

drape_add_tests(drape_render_math_tests
  SOURCES tests/test_render_math.cpp
  DEPS drape_render_math)

# The Filament renderer (Apple only): materials compiled with Filament's matc, the HDRI turned into an IBL with cmgen.
if(APPLE)
  enable_language(OBJCXX)
  set(DRAPE_RENDER_ASSETS ${CMAKE_BINARY_DIR}/render_assets)
  set(DRAPE_RENDER_ASSETS_DIR ${DRAPE_RENDER_ASSETS} CACHE INTERNAL "compiled materials and IBL")
  set(asset_outputs)
  foreach(m mannequin cloth)
    set(out ${DRAPE_RENDER_ASSETS}/materials/${m}.filamat)
    add_custom_command(OUTPUT ${out}
      COMMAND ${CMAKE_COMMAND} -E make_directory ${DRAPE_RENDER_ASSETS}/materials
      COMMAND ${DRAPE_MATC} -a metal -p desktop -o ${out} ${CMAKE_CURRENT_SOURCE_DIR}/materials/${m}.mat
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/materials/${m}.mat
      VERBATIM)
    list(APPEND asset_outputs ${out})
  endforeach()
  set(ibl_stamp ${DRAPE_RENDER_ASSETS}/ibl.stamp)
  add_custom_command(OUTPUT ${ibl_stamp}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${DRAPE_RENDER_ASSETS}/ibl
    COMMAND ${DRAPE_CMGEN} --format=ktx --size=256 --deploy=${DRAPE_RENDER_ASSETS}/ibl ${DRAPE_HDRI}
    COMMAND ${CMAKE_COMMAND} -E touch ${ibl_stamp}
    DEPENDS ${DRAPE_HDRI}
    VERBATIM)
  list(APPEND asset_outputs ${ibl_stamp})
  add_custom_target(drape_render_assets ALL DEPENDS ${asset_outputs})

  drape_add_library(drape_render
    SOURCES src/renderer.cpp src/metal_layer.mm
    PUBLIC_DEPS drape_render_math drape_core
    PRIVATE_DEPS drape_filament)
  target_link_libraries(drape_render PUBLIC "-framework AppKit" "-framework QuartzCore")
  add_dependencies(drape_render drape_render_assets)

  drape_add_tests(drape_render_tests
    SOURCES tests/test_renderer.cpp
    DEPS drape_render drape_session
    LABELS gpu)
  target_compile_definitions(drape_render_tests PRIVATE DRAPE_RENDER_ASSETS="${DRAPE_RENDER_ASSETS}"
                             DRAPE_BINARY_DIR="${CMAKE_BINARY_DIR}")
endif()
