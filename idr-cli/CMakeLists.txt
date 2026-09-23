cmake_minimum_required(VERSION 3.16)
project(idr-cli VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /permissive- /O2)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# idr-core include & library
include_directories(../idr-core/include)
link_directories(../idr-core/build)

add_executable(idr-cli main.cpp)
target_link_libraries(idr-cli PRIVATE idr_core)
