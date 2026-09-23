cmake_minimum_required(VERSION 3.16)
project(idr-core VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
if(MSVC)
    add_compile_options(/W4 /permissive- /O2)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Include directories
include_directories(include)

# Core source files
set(IDR_CORE_SOURCES
    src/math_utils.cpp
    src/gbdt_model.cpp
    src/vibration_filter.cpp
    src/feature_extractor.cpp
    src/alignment_engine.cpp
    src/eskf.cpp
    src/heading_supervisor.cpp
    src/osm_graph.cpp
    src/map_matcher_3d.cpp
    src/speed_calibrator.cpp
    src/longitudinal_sync.cpp
    src/navigation_engine.cpp
    src/c_api.cpp
    src/idrmap_reader.cpp
)

# Core static library
add_library(idr_core STATIC ${IDR_CORE_SOURCES})
target_include_directories(idr_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Core shared library for JNI / Python ctypes
add_library(idr_core_shared SHARED ${IDR_CORE_SOURCES})
target_include_directories(idr_core_shared PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
set_target_properties(idr_core_shared PROPERTIES OUTPUT_NAME "idr_core")

# Enable testing
enable_testing()
add_subdirectory(tests)
