drape_add_library(drape_core
  SOURCES src/version.cpp
  PUBLIC_DEPS Eigen3::Eigen nlohmann_json::nlohmann_json
  PRIVATE_DEPS miniz)
target_compile_definitions(drape_core PUBLIC JSON_DIAGNOSTICS=1)

drape_add_tests(drape_core_tests
  SOURCES tests/test_units.cpp
  DEPS drape_core)
