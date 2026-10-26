# Portable render math (normals, camera, image checks); the Filament renderer is added for Apple builds in Phase B Task 12.
drape_add_library(drape_render_math
  SOURCES src/normals.cpp src/orbit_camera.cpp src/image_metrics.cpp
  PUBLIC_DEPS drape_core)

drape_add_tests(drape_render_math_tests
  SOURCES tests/test_render_math.cpp
  DEPS drape_render_math)
