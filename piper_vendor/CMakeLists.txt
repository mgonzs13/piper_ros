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
  GIT_TAG 2024.11.14-2
)

FetchContent_MakeAvailable(piper)

# Piper source include and lib
set(PIPER_INCLUDE_DIR ${piper_SOURCE_DIR}/src/cpp)
set(PI_PIPER_INCLUDE_DIR ${piper_BINARY_DIR}/pi/include)
set(FI_PIPER_INCLUDE_DIR ${piper_BINARY_DIR}/fi/include)
set(SI_PIPER_INCLUDE_DIR ${piper_BINARY_DIR}/si/include)

set(PI_PIPER_LIB_DIR ${piper_BINARY_DIR}/pi/lib)
set(FI_PIPER_LIB_DIR ${piper_BINARY_DIR}/fi/lib)
set(SI_PIPER_LIB_DIR ${piper_BINARY_DIR}/si/lib)

# Create piper lib
add_library(piper_lib STATIC ${PIPER_INCLUDE_DIR}/piper.cpp)
add_dependencies(piper_lib 
  fmt_external
  spdlog_external
  piper_phonemize_external
)

target_link_libraries(piper_lib
  fmt
  spdlog
  espeak-ng
  piper_phonemize
  onnxruntime
)

target_include_directories(piper_lib
  PUBLIC
    $<BUILD_INTERFACE:${PIPER_INCLUDE_DIR};${PI_PIPER_INCLUDE_DIR};${FI_PIPER_INCLUDE_DIR};${SI_PIPER_INCLUDE_DIR}>
)

target_link_directories(piper_lib
  PUBLIC
    $<BUILD_INTERFACE:${PI_PIPER_LIB_DIR};${FI_PIPER_LIB_DIR};${SI_PIPER_LIB_DIR}>
)

# Install
install(
  DIRECTORY ${piper_BINARY_DIR}/pi/share/espeak-ng-data
  DESTINATION share/${PROJECT_NAME}/
)

install(
  FILES ${piper_BINARY_DIR}/pi/share/libtashkeel_model.ort
  DESTINATION share/${PROJECT_NAME}/
)

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

install(
  DIRECTORY ${PI_PIPER_LIB_DIR}/
  DESTINATION lib
)

install(
  DIRECTORY ${FI_PIPER_LIB_DIR}/
  DESTINATION lib
)

install(
  DIRECTORY ${SI_PIPER_LIB_DIR}/
  DESTINATION lib
)

install(TARGETS piper_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_include_directories(${PI_PIPER_INCLUDE_DIR})
ament_export_include_directories(${FI_PIPER_INCLUDE_DIR})
ament_export_include_directories(${SI_PIPER_INCLUDE_DIR})
ament_export_libraries(piper_lib)
ament_export_libraries(onnxruntime)
ament_export_libraries(fmt)
ament_export_libraries(spdlog)
ament_export_libraries(piper_phonemize)
ament_export_libraries(espeak-ng)
ament_package()
