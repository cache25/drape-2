drape_add_library(drape_sim
  SOURCES src/bending.cpp src/collision.cpp src/energies.cpp src/cpu_solver.cpp src/drape_runner.cpp
  PUBLIC_DEPS drape_core)

drape_add_tests(drape_sim_tests
  SOURCES tests/test_stretch.cpp tests/test_cpu_solver.cpp tests/test_bending.cpp tests/test_seams.cpp tests/test_body_collision.cpp tests/test_cantilever.cpp tests/test_seam_hinge.cpp tests/test_drape_runner.cpp
  DEPS drape_sim drape_garment drape_avatar)

