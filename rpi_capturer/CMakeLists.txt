cmake_minimum_required(VERSION 3.16)
project(rpi_capturer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
find_package(CURL REQUIRED)

pkg_check_modules(LIBCAMERA REQUIRED libcamera)
pkg_check_modules(LIBTURBOJPEG REQUIRED libturbojpeg)

# libav libraries for MP4 muxing
pkg_check_modules(LIBAVFORMAT REQUIRED libavformat)
pkg_check_modules(LIBAVCODEC REQUIRED libavcodec)
pkg_check_modules(LIBAVUTIL REQUIRED libavutil)

# ALSA for audio capture
pkg_check_modules(ALSA REQUIRED alsa)

# cpptoml is provided by system package libcpptoml-dev
# Header is in /usr/include/cpptoml.h

include_directories(${LIBCAMERA_INCLUDE_DIRS})
include_directories(${CURL_INCLUDE_DIRS})
include_directories(${LIBAVFORMAT_INCLUDE_DIRS})
include_directories(${LIBAVCODEC_INCLUDE_DIRS})
include_directories(${LIBAVUTIL_INCLUDE_DIRS})
include_directories(${ALSA_INCLUDE_DIRS})
include_directories(include)

add_executable(rpi_capturer src/main_v2.cpp)

target_link_libraries(rpi_capturer ${LIBCAMERA_LIBRARIES})
target_link_libraries(rpi_capturer ${LIBTURBOJPEG_LIBRARIES})
target_link_libraries(rpi_capturer ${CURL_LIBRARIES})
target_link_libraries(rpi_capturer ${LIBAVFORMAT_LIBRARIES})
target_link_libraries(rpi_capturer ${LIBAVCODEC_LIBRARIES})
target_link_libraries(rpi_capturer ${LIBAVUTIL_LIBRARIES})
target_link_libraries(rpi_capturer ${ALSA_LIBRARIES})
target_compile_options(rpi_capturer PRIVATE ${LIBCAMERA_CFLAGS})
