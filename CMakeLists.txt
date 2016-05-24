cmake_minimum_required(VERSION 2.8)

PROJECT(MODS)

add_definitions(-DA64)
add_definitions(-std=c++11)

SET(WITH_CAFFE FALSE)

message("Looking for OpenCV")
SET (OpenCV_DIR "C:/opencv-2.4.8/opencv-2.4.8/build/install")
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/windows/mods/mods-video/binary_desc/brisk/")
FIND_PACKAGE( OpenCV  REQUIRED core imgproc highgui)
if (OPENCV_FOUND)
  message("Found!")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenCV_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenCV_CXX_FLAGS}")
include_directories(${OpenCV_INDLUDE_DIRS})
  message("${OpenCV_CXX_FLAGS}")
else(OPENCV_FOUND)
  message("Not found!")
  set(OpenCV_CXX_FLAGS "-L/usr/local/opencv-2-4-9/lib/")
include_directories(/usr/local/opencv-2.4.9/include)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenCV_CXX_FLAGS}")
  link_directories(/usr/local/opencv-2.4.9/lib)
endif (OPENCV_FOUND)
#FIND_PACKAGE(CUDA REQUIRED)

enable_language (Fortran)
#  set(LAPACK_DIR "${CMAKE_SOURCE_DIR}/lapack_for_windows")

FIND_PACKAGE(LAPACK)
if (LAPACK_FOUND)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LAPACK_CXX_FLAGS}")
else (LAPACK_FOUND)
  link_directories(${CMAKE_SOURCE_DIR}/lapack_for_windows/lib)
  include_directories(./lapack_for_windows/include)
endif (LAPACK_FOUND)


FIND_PACKAGE(OpenMP)
if (OPENMP_FOUND)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif (OPENMP_FOUND)

SET(CMAKE_BUILD_TYPE "RELEASE")

IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-strict-aliasing")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra -Wno-write-strings")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -ansi")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ftree-vectorize -funroll-loops")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

#add_subdirectory( ./lib)
#include_directories(
#.
#./libdaisy/include
#./libdaisy/include/daisy
#)

add_subdirectory( ./inih)
add_subdirectory( ./matutls)
add_subdirectory( ./detectors)
add_subdirectory( ./detectors/affinedetectors)
add_subdirectory( ./detectors/mser )
add_subdirectory( ./opensurf )
add_subdirectory( ./libMatch)
add_subdirectory( ./libNumerics)
add_subdirectory( ./akaze)
add_subdirectory( ./mrogh)
#add_subdirectory( ./TILDE/c++)

####### DEGENSAC
include_directories(
.
./degensac)
set(degensac_srcs
        degensac/DegUtils.c
        degensac/exp_ranF.c
        degensac/exp_ranH.c
        degensac/Ftools.c
        degensac/hash.c
        degensac/Htools.c
        degensac/ranF.c
        degensac/ranH2el.c
        degensac/ranH.c
        degensac/rtools.c
        degensac/utools.c
        degensac/lapwrap.c
)
link_directories(${CMAKE_SOURCE_DIR}/lapack_for_windows/lib)

add_library(degensac ${degensac_srcs})
TARGET_LINK_LIBRARIES(degensac matutls lapack)
##### End of DEGENSAC

###################Matching
include_directories(.
./matching
 )

set(matching_srcs
        matching/siftdesc.cpp
        matching/matching.cpp
        orsa.cpp
)
link_directories(${CMAKE_SOURCE_DIR}/lapack_for_windows/lib)


add_library(matching ${matching_srcs})
TARGET_LINK_LIBRARIES(matching opencv_features2d248 opencv_core248 opencv_highgui248 degensac lapack Match Numerics)

################## View synthesis

set(synth-detection_srcs
        synth-detection.cpp)

include_directories(.)

link_directories(c:/opencv-2.4.8/opencv-2.4.8/build/install/x86/mingw/lib)
add_library(synth-detection ${synth-detection_srcs})

TARGET_LINK_LIBRARIES(synth-detection matching opensurf
mser affinedetectors AKAZE helpers  opencv_features2d248 opencv_core248 opencv_highgui248)

#include_directories(.
#./dali)
#./smsld)

link_directories(${CMAKE_SOURCE_DIR}/lapack_for_windows/lib)
link_directories(c:/opencv-2.4.8/opencv-2.4.8/build/install/x86/mingw/lib)
add_executable(mods mods.cpp correspondencebank.cpp imagerepresentation.cpp
io_mods.cpp  ssdesc-cpp-1.1.1/ssdesc.cc)#smsld/wzhlib.cpp smsld/descriptor.cpp smsld/Match.cpp smsld/Image.cpp)

 link_directories(${CMAKE_SOURCE_DIR}/lapack_for_windows/lib)
TARGET_LINK_LIBRARIES(mods inih matching synth-detection mser affinedetectors  AKAZE  mrogh  
opencv_features2d248 opencv_core248 opencv_highgui248 m )

add_executable(extract_features extract_features.cpp correspondencebank.cpp imagerepresentation.cpp
io_mods.cpp  ssdesc-cpp-1.1.1/ssdesc.cc) # smsld/wzhlib.cpp smsld/descriptor.cpp smsld/Match.cpp smsld/Image.cpp)
TARGET_LINK_LIBRARIES(extract_features inih matching synth-detection mser affinedetectors  AKAZE  mrogh  
opencv_features2d248 opencv_core248 opencv_highgui248 m )
