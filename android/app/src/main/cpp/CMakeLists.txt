cmake_minimum_required(VERSION 3.16)
project(idr_jni VERSION 1.1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Root directory of idr-core
set(IDR_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../idr-core)

include_directories(
    ${IDR_CORE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(IDR_CORE_SOURCES
    ${IDR_CORE_DIR}/src/math_utils.cpp
    ${IDR_CORE_DIR}/src/gbdt_model.cpp
    ${IDR_CORE_DIR}/src/vibration_filter.cpp
    ${IDR_CORE_DIR}/src/feature_extractor.cpp
    ${IDR_CORE_DIR}/src/alignment_engine.cpp
    ${IDR_CORE_DIR}/src/eskf.cpp
    ${IDR_CORE_DIR}/src/heading_supervisor.cpp
    ${IDR_CORE_DIR}/src/osm_graph.cpp
    ${IDR_CORE_DIR}/src/map_matcher_3d.cpp
    ${IDR_CORE_DIR}/src/speed_calibrator.cpp
    ${IDR_CORE_DIR}/src/longitudinal_sync.cpp
    ${IDR_CORE_DIR}/src/navigation_engine.cpp
    ${IDR_CORE_DIR}/src/c_api.cpp
)

# Build shared JNI library
add_library(idr_jni SHARED
    ${IDR_CORE_SOURCES}
    jni_bridge.cpp
)

# Optimization and warning flags
if(MSVC)
    target_compile_options(idr_jni PRIVATE /W4 /permissive- /O2)
else()
    target_compile_options(idr_jni PRIVATE -Wall -Wextra -Wpedantic -O3 -fvisibility=hidden)
endif()

target_include_directories(idr_jni PUBLIC
    ${IDR_CORE_DIR}/include
)

# Link platform log library if on Android
if(ANDROID)
    find_library(log-lib log)
    target_link_libraries(idr_jni PRIVATE ${log-lib})
endif()
