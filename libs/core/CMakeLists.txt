drape_add_library(drape_core
  SOURCES src/version.cpp src/bezier.cpp src/outline.cpp src/polygon.cpp
  PUBLIC_DEPS Eigen3::Eigen nlohmann_json::nlohmann_json
  PRIVATE_DEPS miniz)
target_compile_definitions(drape_core PUBLIC JSON_DIAGNOSTICS=1)

drape_add_tests(drape_core_tests
  SOURCES tests/test_units.cpp tests/test_bezier.cpp tests/test_outline.cpp tests/test_polygon.cpp
  DEPS drape_core)
