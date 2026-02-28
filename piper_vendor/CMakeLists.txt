cmake_minimum_required(VERSION 3.8)
project(piper_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(ExternalProject)
find_package(ament_cmake REQUIRED)

# -----------------------------------------------------------------------------
# 1) Build libpiper via ExternalProject (handles espeak-ng and onnxruntime)
# -----------------------------------------------------------------------------

set(PIPER_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/piper-install)

ExternalProject_Add(piper_external
  GIT_REPOSITORY https://github.com/OHF-Voice/piper1-gpl
  GIT_TAG v1.4.1
  SOURCE_SUBDIR libpiper
  PATCH_COMMAND find <SOURCE_DIR> -name CMakeLists.txt -exec
    sed -i "s/cmake_minimum_required(VERSION 3\\.2[6-9])/cmake_minimum_required(VERSION 3.16)/" {} +
  # Fix espeak-ng path length limit (N_PATH_HOME=160) that truncates filenames
  # during phoneme compilation in deep build trees (espeak-ng/espeak-ng#2048).
  # Use commit a06074c which raises the limit to PATH_MAX (espeak-ng PR #2220).
  COMMAND sed -i
    "s/212928b394a96e8fd2096616bfd54e17845c48f6/a06074c8fd80f2fd3632164dc01ebf1135395e11/"
    <SOURCE_DIR>/libpiper/CMakeLists.txt
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${PIPER_INSTALL_DIR}
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
  BUILD_BYPRODUCTS ${PIPER_INSTALL_DIR}/libpiper.so
  UPDATE_DISCONNECTED TRUE
)

# -----------------------------------------------------------------------------
# 2) Create an INTERFACE library wrapping the externally-built piper
# -----------------------------------------------------------------------------

add_library(piper_lib INTERFACE)
add_dependencies(piper_lib piper_external)

target_include_directories(piper_lib
  INTERFACE
    $<BUILD_INTERFACE:${PIPER_INSTALL_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# libpiper.so is installed to the prefix root by upstream;
# onnxruntime libs are installed to lib/
target_link_directories(piper_lib
  INTERFACE
    $<BUILD_INTERFACE:${PIPER_INSTALL_DIR}>
    $<BUILD_INTERFACE:${PIPER_INSTALL_DIR}/lib>
    $<INSTALL_INTERFACE:lib>
)

target_link_libraries(piper_lib
  INTERFACE
    piper
    onnxruntime
)

# -----------------------------------------------------------------------------
# 3) Install headers, data files, and shared libraries
# -----------------------------------------------------------------------------

# Install espeak-ng data
install(
  DIRECTORY ${PIPER_INSTALL_DIR}/espeak-ng-data
  DESTINATION share/${PROJECT_NAME}/
)

# Install public header
install(
  FILES ${PIPER_INSTALL_DIR}/include/piper.h
  DESTINATION include
)

# Install libpiper shared library (upstream places it in the prefix root)
install(
  FILES ${PIPER_INSTALL_DIR}/libpiper.so
  DESTINATION lib
)

# Install onnxruntime shared libraries
install(
  DIRECTORY ${PIPER_INSTALL_DIR}/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

# Install the INTERFACE target
install(
  TARGETS piper_lib
  EXPORT export_piper
)

# -----------------------------------------------------------------------------
# 4) Export everything so downstream "find_package(piper_vendor)" just works
# -----------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_targets(export_piper)
ament_export_libraries(piper onnxruntime)

ament_package()
