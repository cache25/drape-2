# Bad arguments must print the usage line on stderr and exit 2.
execute_process(COMMAND ${DUMP} --size XXXL RESULT_VARIABLE rc ERROR_VARIABLE err OUTPUT_QUIET)
if(NOT rc EQUAL 2)
  message(FATAL_ERROR "drape_dump exited ${rc}, expected 2")
endif()
if(NOT err MATCHES "^usage: drape_dump --out <file.obj>")
  message(FATAL_ERROR "missing usage line, got: ${err}")
endif()
