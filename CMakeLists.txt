cmake_minimum_required(VERSION 3.10)
project(GoProMoto)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED PATHS E:/software/opencv/build)
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        /usr/include/x86_64-linux-gnu
)

link_directories(
        /usr/lib/x86_64-linux-gnu
)

add_executable(GoProMoto
        ./GPMF/GPMF_parser.h
        ./GPMF/GPMF_parser.c
        ./GPMF/GPMF_utils.h
        ./GPMF/GPMF_utils.c
        ./GPMF/GPMF_bitstream.h
        ./GPMF/GPMF_mp4reader.c
        ./GPMF/GPMF_mp4reader.h
        extract_GPMF.cpp
        extract_GPMF.h
        main.cpp
        overlay_functions.cpp
        overlay_functions.h
        compute_laps.cpp
        compute_laps.h
        merge_audio.cpp
        merge_audio.h
        merge_audio_video.cpp
        merge_audio_video.h
        images.h
        images.cpp)


target_link_libraries(GoProMoto
        ${OpenCV_LIBS}
        ${CMAKE_SOURCE_DIR}/libgpmf-parser.a
        -lavformat
        -lavcodec
        -lswscale
        -lavutil
        -lswresample
        # -lm
        # -lz
        )

