cmake_minimum_required(VERSION 3.10)
project(dart_vision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

set(SIM_RECEIVER ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB_RECURSE SRC_FILES "src/*.cpp")
list(FILTER SRC_FILES EXCLUDE REGEX ".*[/\\]\\..*")
list(FILTER SRC_FILES EXCLUDE REGEX ".*[/\\]__MACOSX[/\\].*")

if(SIM_RECEIVER)
{
  add_subdirectory(sim_receiver)
}
add_executable(dart_vision ${SRC_FILES})

target_include_directories(dart_vision PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(dart_vision ${OpenCV_LIBS} ${YAML_CPP_LDFLAGS} sim_receiver)
