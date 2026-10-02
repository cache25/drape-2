drape_add_library(drape_garment
  SOURCES src/shapes.cpp src/validate.cpp
  PUBLIC_DEPS drape_core)

drape_add_tests(drape_garment_tests
  SOURCES tests/test_validate.cpp
  DEPS drape_garment)
