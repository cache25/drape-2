drape_add_library(drape_avatar
  SOURCES src/primitives.cpp src/test_body.cpp
  PUBLIC_DEPS drape_core)

drape_add_tests(drape_avatar_tests
  SOURCES tests/test_primitives.cpp
  DEPS drape_avatar)
