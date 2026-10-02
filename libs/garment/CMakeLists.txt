drape_add_library(drape_garment
  SOURCES src/shapes.cpp src/validate.cpp src/spec_table.cpp src/test_tee.cpp src/sim_mesh_builder.cpp
  PUBLIC_DEPS drape_core
  PRIVATE_DEPS CDT::CDT)

drape_add_tests(drape_garment_tests
  SOURCES tests/test_validate.cpp tests/test_spec_table.cpp tests/test_test_tee.cpp tests/test_sim_mesh.cpp
  DEPS drape_garment)
