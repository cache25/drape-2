# The Drape Mac app (Phase B spec 4). Needs Qt 6.12 Widgets; skipped (with a configure warning) without it.
if(NOT DRAPE_HAVE_QT)
  return()
endif()

set(CMAKE_AUTOMOC ON)
add_executable(Drape MACOSX_BUNDLE main.cpp main_window.cpp viewport_widget.cpp self_test.cpp)
target_compile_options(Drape PRIVATE ${DRAPE_WARNINGS})
target_link_libraries(Drape PRIVATE Qt6::Widgets drape_session drape_render drape_sim)
set_target_properties(Drape PROPERTIES
  MACOSX_BUNDLE_BUNDLE_NAME Drape
  MACOSX_BUNDLE_GUI_IDENTIFIER com.drape.app
  MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION}
  MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION})
add_dependencies(Drape drape_kernel_source drape_render_assets)

# Contents/Resources: mannequins, compiled materials, IBL and the Metal kernel source.
set(res $<TARGET_BUNDLE_CONTENT_DIR:Drape>/Resources)
add_custom_command(TARGET Drape POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${res}/mannequins
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/assets/mannequins ${res}/mannequins
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${DRAPE_RENDER_ASSETS_DIR}/materials ${res}/materials
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${DRAPE_RENDER_ASSETS_DIR}/ibl ${res}/ibl
  COMMAND ${CMAKE_COMMAND} -E copy ${DRAPE_KERNEL_SOURCE_FILE} ${res}/drape_kernels.metal
  VERBATIM)
