cmake_minimum_required(VERSION 3.10)
project(piper_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(ExternalProject)
find_package(ament_cmake REQUIRED)

# Prefer the ONNX Runtime released for the ROS 2 distro (onnxruntime_vendor).
# This avoids downloading a second ONNX Runtime and keeps a single ORT in the
# process. If the package is not available (e.g. Iron), libpiper vendors its
# own release as before.
find_package(onnxruntime_vendor QUIET)
set(PIPER_USE_DISTRO_ORT FALSE)
if(TARGET onnxruntime::onnxruntime)
  get_target_property(PIPER_ORT_LIB onnxruntime::onnxruntime IMPORTED_LOCATION)
  if(PIPER_ORT_LIB)
    get_filename_component(PIPER_ORT_LIB_DIR "${PIPER_ORT_LIB}" DIRECTORY)
    get_filename_component(PIPER_ORT_DIR "${PIPER_ORT_LIB_DIR}" DIRECTORY)
    if(EXISTS "${PIPER_ORT_DIR}/include/onnxruntime_cxx_api.h")
      set(PIPER_USE_DISTRO_ORT TRUE)
      message(STATUS "piper_vendor: using ONNX Runtime from onnxruntime_vendor (${PIPER_ORT_DIR})")
    endif()
  endif()
  if(NOT PIPER_USE_DISTRO_ORT)
    message(WARNING "onnxruntime_vendor found but its prefix could not be derived; vendoring ONNX Runtime")
  endif()
else()
  message(STATUS "piper_vendor: onnxruntime_vendor not found; vendoring ONNX Runtime")
endif()
unset(PIPER_ORT_LIB)
unset(PIPER_ORT_LIB_DIR)

set(PIPER_ORT_CMAKE_ARGS "")
if(PIPER_USE_DISTRO_ORT)
  set(PIPER_ORT_CMAKE_ARGS -DONNXRUNTIME_DIR=${PIPER_ORT_DIR})
else()
  # Drop a stale ONNXRUNTIME_DIR (e.g. from a previous build using the distro
  # package) so libpiper downloads and uses its own copy again
  set(PIPER_ORT_CMAKE_ARGS -UONNXRUNTIME_DIR)
endif()

# -----------------------------------------------------------------------------
# 1) Build libpiper via ExternalProject (handles espeak-ng and, as a fallback,
#    onnxruntime)
# -----------------------------------------------------------------------------

set(PIPER_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/piper-install)

ExternalProject_Add(piper_external
  GIT_REPOSITORY https://github.com/OHF-Voice/piper1-gpl
  GIT_TAG v1.8.0
  SOURCE_SUBDIR libpiper
  PATCH_COMMAND find <SOURCE_DIR> -name CMakeLists.txt -exec
    sed -i "s/cmake_minimum_required(VERSION 3\\.2[6-9])/cmake_minimum_required(VERSION 3.16)/" {} +
  # Fix espeak-ng path length limit (N_PATH_HOME=160) that truncates filenames
  # during phoneme compilation in deep build trees (espeak-ng/espeak-ng#2048).
  # Use commit a06074c which raises the limit to PATH_MAX (espeak-ng PR #2220).
  COMMAND sed -i
    "s/212928b394a96e8fd2096616bfd54e17845c48f6/a06074c8fd80f2fd3632164dc01ebf1135395e11/"
    <SOURCE_DIR>/libpiper/CMakeLists.txt
  COMMAND sed -i
    "s|add_subdirectory(src/main)|# add_subdirectory(src/main) - removed|"
    <SOURCE_DIR>/libpiper/CMakeLists.txt
  # Force find_library() to use the onnxruntime downloaded by libpiper instead
  # of another onnxruntime found through CMAKE_PREFIX_PATH (only relevant when
  # the distro onnxruntime is not used)
  COMMAND sed -i
    "s|/lib REQUIRED|/lib NO_DEFAULT_PATH REQUIRED|"
    <SOURCE_DIR>/libpiper/CMakeLists.txt
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${PIPER_INSTALL_DIR}
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    # Drop libpiper's cached find_library() result so it is resolved again
    # against the currently selected ONNXRUNTIME_DIR
    -UONNXRUNTIME_LIB
    # Never let libpiper pick up an arbitrary onnxruntime from
    # CMAKE_PREFIX_PATH; it either uses ONNXRUNTIME_DIR (from
    # onnxruntime_vendor) or downloads its own release as a fallback
    -DCMAKE_DISABLE_FIND_PACKAGE_onnxruntime=ON
    ${PIPER_ORT_CMAKE_ARGS}
  BUILD_BYPRODUCTS ${PIPER_INSTALL_DIR}/lib/libpiper.so
  UPDATE_DISCONNECTED FALSE
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

# Since piper 1.7.0, libpiper.so is installed to lib/
target_link_directories(piper_lib
  INTERFACE
    $<BUILD_INTERFACE:${PIPER_INSTALL_DIR}/lib>
    $<INSTALL_INTERFACE:lib>
)

# Propagate the ONNX Runtime that libpiper links against so consumers use the
# exact same copy (the distro one when available)
if(PIPER_USE_DISTRO_ORT)
  target_link_libraries(piper_lib
    INTERFACE
      piper
      onnxruntime::onnxruntime
  )
else()
  target_link_libraries(piper_lib
    INTERFACE
      piper
      onnxruntime
  )
endif()

# -----------------------------------------------------------------------------
# 3) Install headers, data files, and shared libraries
# -----------------------------------------------------------------------------

# Install espeak-ng data (upstream installs it under share/)
install(
  DIRECTORY ${PIPER_INSTALL_DIR}/share/espeak-ng-data
  DESTINATION share/${PROJECT_NAME}/
)

# Install public header
install(
  FILES ${PIPER_INSTALL_DIR}/include/piper.h
  DESTINATION include
)

# Install libpiper shared library (upstream installs it under lib/)
install(
  FILES ${PIPER_INSTALL_DIR}/lib/libpiper.so
  DESTINATION lib
)

# Install onnxruntime shared libraries only when libpiper vendors its own copy
if(NOT PIPER_USE_DISTRO_ORT)
  install(
    DIRECTORY ${PIPER_INSTALL_DIR}/lib/
    DESTINATION lib
    FILES_MATCHING PATTERN "libonnxruntime*"
  )
endif()

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
ament_export_libraries(piper)
if(PIPER_USE_DISTRO_ORT)
  ament_export_dependencies(onnxruntime_vendor)
else()
  ament_export_libraries(onnxruntime)
endif()

ament_package()
