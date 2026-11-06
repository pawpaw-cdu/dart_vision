cmake_minimum_required(VERSION 3.10)
project(sim_receiver LANGUAGES CXX)

# 只依赖 core / imgproc / imgcodecs：接收库本身不需要 videoio，
# 避免把 opencv_videoio 及其 ffmpeg 依赖连带进最终可执行文件。
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

add_library(sim_receiver STATIC
    src/tcp_input.cpp
    src/vision_date_recevier.cpp
)

target_include_directories(sim_receiver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(sim_receiver PRIVATE ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(sim_receiver PUBLIC ${OpenCV_LIBS})
target_link_libraries(sim_receiver PRIVATE ${YAML_CPP_LDFLAGS})

include(GNUInstallDirs)
install(TARGETS sim_receiver
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
