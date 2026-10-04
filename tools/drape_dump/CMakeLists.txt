add_library(drape_obj_writer STATIC obj_writer.cpp)
target_include_directories(drape_obj_writer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(drape_obj_writer PRIVATE ${DRAPE_WARNINGS})
target_link_libraries(drape_obj_writer PUBLIC drape_core)

add_executable(drape_dump main.cpp)
target_compile_options(drape_dump PRIVATE ${DRAPE_WARNINGS})
target_link_libraries(drape_dump PRIVATE drape_obj_writer drape_sim drape_garment drape_avatar)

add_executable(drape_obj_writer_tests test_obj_writer.cpp)
target_compile_options(drape_obj_writer_tests PRIVATE ${DRAPE_WARNINGS})
target_link_libraries(drape_obj_writer_tests PRIVATE drape_obj_writer GTest::gtest_main)
gtest_discover_tests(drape_obj_writer_tests DISCOVERY_TIMEOUT 60)

add_test(NAME DrapeDumpSmoke COMMAND drape_dump --out ${CMAKE_BINARY_DIR}/smoke.obj)
add_test(NAME DrapeDumpBadArguments
         COMMAND ${CMAKE_COMMAND} -DDUMP=$<TARGET_FILE:drape_dump> -P ${CMAKE_CURRENT_SOURCE_DIR}/check_usage.cmake)
