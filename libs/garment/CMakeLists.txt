drape_add_library(drape_garment
  SOURCES src/shapes.cpp src/validate.cpp src/spec_table.cpp src/test_tee.cpp
  PUBLIC_DEPS drape_core)

drape_add_tests(drape_garment_tests
  SOURCES tests/test_validate.cpp tests/test_spec_table.cpp tests/test_test_tee.cpp
  DEPS drape_garment)
