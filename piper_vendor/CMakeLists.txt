cmake_minimum_required(VERSION 3.8)
project(piper_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
find_package(ament_cmake REQUIRED)

FetchContent_Declare(
  piper
  GIT_REPOSITORY https://github.com/rhasspy/piper.git
  GIT_TAG 2023.11.14-2
)
FetchContent_MakeAvailable(piper)

# Piper source include and lib directories
set(PIPER_INCLUDE_DIR      ${piper_SOURCE_DIR}/src/cpp)
set(PI_PIPER_INCLUDE_DIR   ${piper_BINARY_DIR}/pi/include)
set(FI_PIPER_INCLUDE_DIR   ${piper_BINARY_DIR}/fi/include)
set(SI_PIPER_INCLUDE_DIR   ${piper_BINARY_DIR}/si/include)

set(PI_PIPER_LIB_DIR       ${piper_BINARY_DIR}/pi/lib)
set(FI_PIPER_LIB_DIR       ${piper_BINARY_DIR}/fi/lib)
set(SI_PIPER_LIB_DIR       ${piper_BINARY_DIR}/si/lib)

# -----------------------------------------------------------------------------
# 1) Build a static "piper_lib" wrapper
# -----------------------------------------------------------------------------

add_library(piper_lib STATIC
  ${PIPER_INCLUDE_DIR}/piper.cpp
)
add_dependencies(piper_lib 
  fmt_external
  spdlog_external
  piper_phonemize_external
)

# Tell CMake that anything linking against piper_lib also needs to link the
# underlying libraries: fmt, spdlog, espeak-ng, piper_phonemize, onnxruntime.
# By making these PUBLIC, downstream targets will "see" those link-flags automatically.
target_link_libraries(piper_lib
  PRIVATE
    fmt
    spdlog
  PUBLIC
    ${piper_BINARY_DIR}/pi/lib/libespeak-ng.so
    ${piper_BINARY_DIR}/pi/lib/libpiper_phonemize.so
    ${piper_BINARY_DIR}/pi/lib/libonnxruntime.so
)

# Make sure all of Piper’s own “pi/fi/si” sub-directories are on the include path
target_include_directories(piper_lib
  PUBLIC
    $<BUILD_INTERFACE:${PIPER_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${PI_PIPER_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${FI_PIPER_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${SI_PIPER_INCLUDE_DIR}>
)

# Similarly, propagate Piper’s library search paths to anything that links piper_lib
target_link_directories(piper_lib
  PUBLIC
    $<BUILD_INTERFACE:${PI_PIPER_LIB_DIR}>
    $<BUILD_INTERFACE:${FI_PIPER_LIB_DIR}>
    $<BUILD_INTERFACE:${SI_PIPER_LIB_DIR}>
)

# -----------------------------------------------------------------------------
# 2) Install headers, data files, and the compiled piper_lib
# -----------------------------------------------------------------------------

# Install espeak-ng data folder
install(
  DIRECTORY ${piper_BINARY_DIR}/pi/share/espeak-ng-data
  DESTINATION share/${PROJECT_NAME}/
)

# Install the ONNX “tashkeel” model
install(
  FILES ${piper_BINARY_DIR}/pi/share/libtashkeel_model.ort
  DESTINATION share/${PROJECT_NAME}/
)

# Install all of Piper’s public headers
install(
  DIRECTORY ${PIPER_INCLUDE_DIR}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
install(
  DIRECTORY ${PI_PIPER_INCLUDE_DIR}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
install(
  DIRECTORY ${FI_PIPER_INCLUDE_DIR}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
install(
  DIRECTORY ${SI_PIPER_INCLUDE_DIR}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Install the built Piper sub-libs into <install_prefix>/lib
install(
  DIRECTORY ${PI_PIPER_LIB_DIR}/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)
install(
  DIRECTORY ${FI_PIPER_LIB_DIR}/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)
install(
  DIRECTORY ${SI_PIPER_LIB_DIR}/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

# Install our wrapper static library
install(
  TARGETS piper_lib
  EXPORT export_piper
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

# -----------------------------------------------------------------------------
# 3) Export everything so downstream “find_package(piper_vendor)” just works
# -----------------------------------------------------------------------------

# Export include directories so that downstream packages see “include/” automatically
ament_export_include_directories(
  include
  ${PI_PIPER_INCLUDE_DIR}
  ${FI_PIPER_INCLUDE_DIR}
  ${SI_PIPER_INCLUDE_DIR}
)

# Export the piper_lib target (so downstream can do “target_link_libraries(their_node piper_lib)”)
ament_export_targets(export_piper HAS_LIBRARY_TARGET)

# ALSO export the piper_lib library name itself so that the linker flags for
# espeak-ng, piper_phonemize, onnxruntime, etc., get passed along.
#
# Without this, a downstream that does `find_package(piper_vendor)` and
# `target_link_libraries(their_node piper_lib)` won’t know about espeak-ng, etc.,
# and you’ll see “cannot find -lespeak-ng” at link time.
ament_export_libraries(piper_lib)

ament_package()
