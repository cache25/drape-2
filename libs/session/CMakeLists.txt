drape_add_library(drape_session
  SOURCES src/tee_scene.cpp src/sim_controller.cpp
  PUBLIC_DEPS drape_sim drape_garment drape_avatar)

drape_add_tests(drape_session_tests
  SOURCES tests/test_tee_scene.cpp tests/test_sim_controller.cpp
  DEPS drape_session)
