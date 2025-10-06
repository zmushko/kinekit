cmake_minimum_required(VERSION 3.16)
project(rpi_capturer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
find_package(CURL REQUIRED)

pkg_check_modules(LIBCAMERA REQUIRED libcamera)
pkg_check_modules(LIBTURBOJPEG REQUIRED libturbojpeg)

include_directories(${LIBCAMERA_INCLUDE_DIRS})
include_directories(${CURL_INCLUDE_DIRS})
include_directories(include)

add_executable(rpi_capturer src/main_v2.cpp)

target_link_libraries(rpi_capturer ${LIBCAMERA_LIBRARIES})
target_link_libraries(rpi_capturer ${LIBTURBOJPEG_LIBRARIES})
target_link_libraries(rpi_capturer ${CURL_LIBRARIES})
target_compile_options(rpi_capturer PRIVATE ${LIBCAMERA_CFLAGS})
