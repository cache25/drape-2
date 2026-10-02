drape_add_library(drape_sim
  SOURCES src/coloring.cpp src/stretch.cpp src/bending.cpp src/seams.cpp src/cpu_solver.cpp
  PUBLIC_DEPS drape_core)

drape_add_tests(drape_sim_tests
  SOURCES tests/test_coloring.cpp tests/test_stretch.cpp tests/test_cpu_solver.cpp tests/test_bending.cpp tests/test_seams.cpp
  DEPS drape_sim drape_garment drape_avatar)

drape_add_tests(drape_sim_slow_tests
  SOURCES tests/test_cantilever.cpp tests/test_seam_hinge.cpp
  DEPS drape_sim drape_garment drape_avatar
  LABELS slow)
