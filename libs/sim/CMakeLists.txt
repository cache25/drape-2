drape_add_library(drape_sim
  SOURCES src/bending.cpp src/collision.cpp src/energies.cpp src/sim_model.cpp src/cpu_solver.cpp src/drape_runner.cpp src/drape_session.cpp src/scene_builder.cpp src/solver_factory.cpp
    src/gpu/compute_device.cpp src/gpu/emulated_device.cpp src/gpu/gpu_solver_core.cpp
  PUBLIC_DEPS drape_core)

drape_add_tests(drape_sim_tests
  SOURCES tests/test_stretch.cpp tests/test_cpu_solver.cpp tests/test_bending.cpp tests/test_seams.cpp tests/test_body_collision.cpp tests/test_friction.cpp tests/test_cantilever.cpp tests/test_seam_hinge.cpp tests/test_drape_runner.cpp tests/test_tee_drape.cpp tests/test_sim_model.cpp tests/test_drape_session.cpp tests/test_solver_factory.cpp tests/test_term_math.cpp tests/test_kernels.cpp tests/test_gpu_parity.cpp
  DEPS drape_sim drape_garment drape_avatar drape_session)

drape_add_tests(drape_sim_slow_tests
  SOURCES tests/test_tee_drape_slow.cpp tests/test_gpu_parity_slow.cpp
  DEPS drape_sim drape_garment drape_avatar drape_session
  LABELS slow)

# GPU kernel headers (compiled as C++ here and as Metal Shading Language on Macs).
target_include_directories(drape_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/kernels)
target_include_directories(drape_sim_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/kernels)

# Metal backend (Apple only). The kernel headers and wrappers are combined into one Metal source that MetalDevice
# compiles at runtime (the separate Metal toolchain download is not needed).
if(APPLE)
  set(DRAPE_KERNEL_INPUTS
    ${CMAKE_CURRENT_SOURCE_DIR}/kernels/gpu_compat.h ${CMAKE_CURRENT_SOURCE_DIR}/kernels/gpu_layout.h
    ${CMAKE_CURRENT_SOURCE_DIR}/kernels/term_math.h ${CMAKE_CURRENT_SOURCE_DIR}/kernels/kernels.h
    ${CMAKE_CURRENT_SOURCE_DIR}/kernels/drape_kernels.metal)
  set(DRAPE_KERNEL_SOURCE ${CMAKE_BINARY_DIR}/drape_kernels.metal)
  add_custom_command(OUTPUT ${DRAPE_KERNEL_SOURCE}
    COMMAND ${CMAKE_COMMAND} -DOUT=${DRAPE_KERNEL_SOURCE} "-DINPUTS=${DRAPE_KERNEL_INPUTS}"
            -P ${PROJECT_SOURCE_DIR}/cmake/CombineKernels.cmake
    DEPENDS ${DRAPE_KERNEL_INPUTS} ${PROJECT_SOURCE_DIR}/cmake/CombineKernels.cmake
    VERBATIM)
  add_custom_target(drape_kernel_source ALL DEPENDS ${DRAPE_KERNEL_SOURCE})
  set(DRAPE_KERNEL_SOURCE_FILE ${DRAPE_KERNEL_SOURCE} CACHE INTERNAL "combined Metal kernel source")
  target_sources(drape_sim PRIVATE src/gpu/metal_device.cpp src/gpu/metal_solver.cpp)
  target_link_libraries(drape_sim PRIVATE drape_metal_cpp)
  target_compile_definitions(drape_sim PRIVATE DRAPE_HAVE_METAL=1 DRAPE_KERNEL_SOURCE_PATH="${DRAPE_KERNEL_SOURCE}")
  add_dependencies(drape_sim drape_kernel_source)

  drape_add_tests(drape_sim_gpu_tests
    SOURCES tests/test_metal.cpp
    DEPS drape_sim drape_garment drape_avatar drape_session
    LABELS gpu)
  target_include_directories(drape_sim_gpu_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/kernels)

  drape_add_tests(drape_sim_perf_tests
    SOURCES tests/test_tee_benchmark.cpp
    DEPS drape_sim drape_session
    LABELS perf)
endif()
