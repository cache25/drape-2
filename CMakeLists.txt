cmake_minimum_required(VERSION 3.28)
project(drape VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(DrapeDependencies)
include(DrapeHelpers)

add_subdirectory(libs)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tools/drape_dump/CMakeLists.txt)
  add_subdirectory(tools/drape_dump)
endif()
