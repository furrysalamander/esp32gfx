cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

add_executable(esp32gfx_viewer viewer.cpp)

target_link_libraries(esp32gfx_viewer PRIVATE
    esp32gfx
    ${SDL2_LIBRARIES}
)

target_include_directories(esp32gfx_viewer PRIVATE
    ${SDL2_INCLUDE_DIRS}
)

target_compile_options(esp32gfx_viewer PRIVATE
    ${SDL2_CFLAGS_OTHER}
)
