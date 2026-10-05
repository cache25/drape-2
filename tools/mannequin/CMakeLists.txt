# Offline unit tests for the mannequin builder (Python + numpy only).
find_package(Python3 COMPONENTS Interpreter)

set(DRAPE_MANNEQUIN_PYTHON_OK FALSE)
if(Python3_Interpreter_FOUND)
  execute_process(COMMAND ${Python3_EXECUTABLE} -c "import numpy"
                  RESULT_VARIABLE DRAPE_NUMPY_STATUS OUTPUT_QUIET ERROR_QUIET)
  if(DRAPE_NUMPY_STATUS EQUAL 0)
    set(DRAPE_MANNEQUIN_PYTHON_OK TRUE)
  endif()
endif()

if(DRAPE_MANNEQUIN_PYTHON_OK)
  add_test(NAME MannequinBuilderUnit
           COMMAND ${Python3_EXECUTABLE} -m unittest discover
                   -s ${CMAKE_CURRENT_SOURCE_DIR} -p "test_*.py")
else()
  message(STATUS "MannequinBuilderUnit skipped: python3 with numpy not found")
endif()
