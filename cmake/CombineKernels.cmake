# Combines the kernel headers and Metal wrappers into one self-contained Metal source (MetalDevice compiles it at
# runtime, where #include "..." cannot be resolved). Usage:
#   cmake -DOUT=<file> -DINPUTS="<a>;<b>;..." -P CombineKernels.cmake
set(combined "// Generated from libs/sim/kernels by cmake/CombineKernels.cmake. Do not edit.\n")
foreach(in ${INPUTS})
  file(READ ${in} text)
  string(REGEX REPLACE "#pragma once[^\n]*\n" "" text "${text}")
  string(REGEX REPLACE "#include \"[^\"]+\"[^\n]*\n" "" text "${text}")
  get_filename_component(name ${in} NAME)
  string(APPEND combined "\n// ---- ${name} ----\n${text}")
endforeach()
file(WRITE ${OUT}.tmp "${combined}")
file(COPY_FILE ${OUT}.tmp ${OUT} ONLY_IF_DIFFERENT)
file(REMOVE ${OUT}.tmp)
