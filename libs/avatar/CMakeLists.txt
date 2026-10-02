drape_add_library(drape_avatar
  SOURCES src/primitives.cpp src/test_body.cpp src/collision_bake.cpp
  PUBLIC_DEPS drape_core)

drape_add_tests(drape_avatar_tests
  SOURCES tests/test_primitives.cpp tests/test_collision_field.cpp
  DEPS drape_avatar)

drape_add_tests(drape_avatar_perf_tests
  SOURCES tests/test_collision_perf.cpp
  DEPS drape_avatar
  LABELS perf)
