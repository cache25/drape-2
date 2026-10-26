drape_add_library(drape_session
  SOURCES src/tee_scene.cpp
  PUBLIC_DEPS drape_sim drape_garment drape_avatar)

drape_add_tests(drape_session_tests
  SOURCES tests/test_tee_scene.cpp
  DEPS drape_session)
