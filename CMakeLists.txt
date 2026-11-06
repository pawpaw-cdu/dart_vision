cmake_minimum_required(VERSION 3.10)
project(dart_vision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 未指定构建类型时按 Release 构建：视觉循环对帧率敏感，默认 -O0 会白白浪费算力
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  message(STATUS "dart_vision: 未指定 CMAKE_BUILD_TYPE，默认使用 Release")
endif()

# core/imgproc/imgcodecs 就足够跑「检测 + 解算 + 回发」；
# highgui（显示窗口）与 VideoWriter（录像）会额外依赖 opencv_videoio，
# 在没有显示环境或 OpenCV/ffmpeg 版本不匹配的机器上可以用
#   cmake -DDART_VISION_WITH_MEDIA=OFF ..
# 构建纯无头版本。
option(DART_VISION_WITH_MEDIA
       "启用 OpenCV highgui 显示与 VideoWriter 录像（无头部署可关闭）" ON)

if(DART_VISION_WITH_MEDIA)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui)
else()
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
endif()
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

set(SIM_RECEIVER ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB_RECURSE SRC_FILES "src/*.cpp")
list(FILTER SRC_FILES EXCLUDE REGEX ".*[/\\]\\..*")
list(FILTER SRC_FILES EXCLUDE REGEX ".*[/\\]__MACOSX[/\\].*")
if(NOT DART_VISION_WITH_MEDIA)
  # 录像线程依赖 cv::VideoWriter，无头构建时整体排除
  list(FILTER SRC_FILES EXCLUDE REGEX ".*offline_video\\.cpp$")
endif()

add_subdirectory(sim_receiver)
add_executable(dart_vision ${SRC_FILES})

target_include_directories(dart_vision PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(dart_vision ${OpenCV_LIBS} ${YAML_CPP_LDFLAGS} sim_receiver Threads::Threads)

if(DART_VISION_WITH_MEDIA)
  target_compile_definitions(dart_vision PRIVATE DART_VISION_WITH_MEDIA=1)
  message(STATUS "dart_vision: 带显示/录像（highgui + VideoWriter）")
else()
  message(STATUS "dart_vision: 无头构建（无显示窗口、无录像）")
endif()
