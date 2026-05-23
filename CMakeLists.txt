cmake_minimum_required(VERSION 3.10)

set(projectName "GoProMoto")

project(${projectName})

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CAIRO REQUIRED cairo)


include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${CAIRO_INCLUDE_DIRS}
        /usr/include/x86_64-linux-gnu
)

link_directories(
        /usr/lib/x86_64-linux-gnu
)
set(SOURCES
        ./GPMF/GPMF_parser.c        
        ./GPMF/GPMF_utils.c        
        ./GPMF/GPMF_mp4reader.c        
        extract_GPMF.cpp        
        lean_functions.cpp                
        # overlay_functions.cpp
        overlay_functions_cairo.cpp
        compute_laps.cpp        
        merge_audio.cpp        
        merge_audio_video.cpp        
        # images.cpp
        main.cpp
        )

set(HEADER_FILES
        ./GPMF/GPMF_parser.h
        ./GPMF/GPMF_utils.h        
        ./GPMF/GPMF_bitstream.h        
        ./GPMF/GPMF_mp4reader.h        
        extract_GPMF.h
        lean_functions.hpp        
        # overlay_functions.h
        overlay_functions_cairo.hpp
        compute_laps.h        
        merge_audio.h        
        merge_audio_video.h
        # images.h
        )

add_executable(${projectName} ${SOURCES} ${HEADER_FILES})


target_link_libraries(${projectName}
        ${OpenCV_LIBS}
        ${CAIRO_LIBRARIES}
        ${CMAKE_SOURCE_DIR}/libgpmf-parser.a
        -lavformat
        -lavcodec
        -lswscale
        -lavutil
        -lswresample
        # -lm
        # -lz
        )

# Needed so the linker finds libcairo in non-standard prefix (e.g. Homebrew)
target_link_directories(${projectName} PRIVATE
    ${CAIRO_LIBRARY_DIRS}
)

# Pass any compile flags cairo's .pc file requests (e.g. -D_REENTRANT)
target_compile_options(${projectName} PRIVATE
    ${CAIRO_CFLAGS_OTHER}
)

