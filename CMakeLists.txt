cmake_minimum_required(VERSION 3.16)
project(esp32gfx LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(esp32gfx STATIC
    src/raster.cpp
    src/pipeline.cpp
    src/terrain.cpp
)

target_include_directories(esp32gfx PUBLIC include)

option(ESP32GFX_BUILD_TEST "Build SDL2 test viewer" ON)
if(ESP32GFX_BUILD_TEST)
    add_subdirectory(test)
endif()
