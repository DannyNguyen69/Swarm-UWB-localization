cmake_minimum_required(VERSION 3.16)
project(uwb_bridge CXX)

# Self-contained: does not touch dark-landing/CMakeLists.txt.  Build with scripts/build_uwb.sh
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(MAVSDK REQUIRED)

add_executable(uwb_bridge
    src/main.cpp
    src/uwb_proto.cpp
    src/serial_port.cpp
    src/geometry.cpp
    src/tag_tracker.cpp
    src/fc_link.cpp
)
target_include_directories(uwb_bridge PRIVATE src)
target_link_libraries(uwb_bridge PRIVATE MAVSDK::mavsdk pthread)
