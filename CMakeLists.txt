# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.


project(depth2color)
cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)

add_executable(depth2color
    # depth2color.cpp
    # mkvparser.cpp
    main.cpp
    )


find_library(OpenCV /usr/lib/cmake/opencv4)
include_directories(/usr/include/opencv4)

target_link_libraries(depth2color
    k4a
    k4arecord
    turbojpeg
    opencv_core
    opencv_calib3d
    opencv_imgcodecs
    opencv_imgproc
    opencv_highgui
    )
