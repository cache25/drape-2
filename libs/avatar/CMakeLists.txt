drape_add_library(drape_avatar
  SOURCES src/primitives.cpp src/test_body.cpp src/collision_bake.cpp src/obj_reader.cpp src/mannequin.cpp
  PUBLIC_DEPS drape_core)
target_compile_definitions(drape_avatar PUBLIC DRAPE_ASSET_DIR="${PROJECT_SOURCE_DIR}/assets")

drape_add_tests(drape_avatar_tests
  SOURCES tests/test_primitives.cpp tests/test_collision_field.cpp tests/test_mannequin.cpp
  DEPS drape_avatar)

drape_add_tests(drape_avatar_perf_tests
  SOURCES tests/test_collision_perf.cpp
  DEPS drape_avatar
  LABELS perf)
