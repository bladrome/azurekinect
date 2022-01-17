# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.


project(mkv2dist)
cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)

find_library(OpenCV /usr/lib/cmake/opencv4)
include_directories(/usr/include/opencv4)

add_executable(exportall
# add_library(mkv2dist
    # SHARED
    # depth2color.cpp
    # mkvparser.cpp
    src/exportall.cpp
    )

target_link_libraries(exportall
    k4a
    k4arecord
    turbojpeg
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    opencv_highgui
    )




add_executable(mkv2dist
# add_library(mkv2dist
    # SHARED
    # depth2color.cpp
    # mkvparser.cpp
    src/getdistance.cpp
    )

target_link_libraries(mkv2dist
    k4a
    k4arecord
    turbojpeg
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    opencv_highgui
    )



add_executable(genpointclouds
# add_library(mkv2dist
    # depth2color.cpp
    # mkvparser.cpp
    src/genpointclouds.cpp
    )

target_link_libraries(genpointclouds
    k4a
    k4arecord
    turbojpeg
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    opencv_highgui
    )


add_executable(getground
# add_library(mkv2dist
    # depth2color.cpp
    # mkvparser.cpp
    src/getground.cpp
    )

target_link_libraries(getground
    k4a
    k4arecord
    turbojpeg
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    opencv_highgui
    )
