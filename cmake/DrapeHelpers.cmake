# GCC 13 reports false-positive bounds warnings from inlined std::map code in optimized builds
# (nlohmann/json with JSON_DIAGNOSTICS); keep them as warnings, not errors.
set(DRAPE_WARNINGS -Wall -Wextra -Wpedantic -Werror
    $<$<CXX_COMPILER_ID:GNU>:-Wno-error=array-bounds -Wno-error=stringop-overflow -Wno-error=maybe-uninitialized>)

# drape_add_library(<name> SOURCES ... [PUBLIC_DEPS ...] [PRIVATE_DEPS ...])
function(drape_add_library name)
  cmake_parse_arguments(ARG "" "" "SOURCES;PUBLIC_DEPS;PRIVATE_DEPS" ${ARGN})
  add_library(${name} STATIC ${ARG_SOURCES})
  target_include_directories(${name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                     PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_compile_features(${name} PUBLIC cxx_std_20)
  target_compile_options(${name} PRIVATE ${DRAPE_WARNINGS})
  if(ARG_PUBLIC_DEPS)
    target_link_libraries(${name} PUBLIC ${ARG_PUBLIC_DEPS})
  endif()
  if(ARG_PRIVATE_DEPS)
    target_link_libraries(${name} PRIVATE ${ARG_PRIVATE_DEPS})
  endif()
endfunction()

# drape_add_tests(<name> SOURCES ... DEPS ... [LABELS ...])
function(drape_add_tests name)
  cmake_parse_arguments(ARG "" "" "SOURCES;DEPS;LABELS" ${ARGN})
  add_executable(${name} ${ARG_SOURCES})
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_compile_options(${name} PRIVATE ${DRAPE_WARNINGS})
  target_link_libraries(${name} PRIVATE ${ARG_DEPS} GTest::gtest_main)
  if(ARG_LABELS)
    gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60 PROPERTIES LABELS "${ARG_LABELS}")
  else()
    gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60)
  endif()
endfunction()
