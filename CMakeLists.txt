cmake_minimum_required(VERSION 2.8)

PROJECT(MODS_NEW)

add_definitions(-DA64)
add_definitions(-std=c++11)

SET(WITH_CAFFE TRUE)

message("Looking for OpenCV")
#SET (OpenCV_DIR "/usr/local/opencv-2.4.9/")
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/windows/mods/mods-video/binary_desc/brisk/")
FIND_PACKAGE( OpenCV  )
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
FIND_PACKAGE(LAPACK REQUIRED)
if (LAPACK_FOUND)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LAPACK_CXX_FLAGS}")
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


if(WITH_CAFFE)
add_definitions(-DWITH_CAFFE) #caffe cpu_only
include_directories(/home/old-ufo/dev/caffe/include)
include_directories(/home/old-ufo/dev/caffe/build/src/)
include_directories(/opt/OpenBLAS/include)
include_directories(/usr/local/cuda/include)
link_directories(/home/old-ufo/dev/caffe/build/lib)
link_directories(/opt/OpenBLAS/lib)
endif(WITH_CAFFE)

link_directories(/home/old-ufo/dev/mods/wxbs-journal/vlfeat/bin/glnxa64)



add_subdirectory( ./inih)
add_subdirectory( ./matutls)
add_subdirectory( ./detectors)
add_subdirectory(./detectors/new-saddle)
add_subdirectory( ./detectors/affinedetectors)
add_subdirectory( ./detectors/mser )
add_subdirectory( ./opensurf )
add_subdirectory( ./libMatch)
add_subdirectory( ./libNumerics)
add_subdirectory( ./scv)
add_subdirectory( ./akaze)
add_subdirectory( ./mrogh)

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
add_library(degensac ${degensac_srcs})
TARGET_LINK_LIBRARIES(degensac matutls lapack)
##### End of DEGENSAC

#Add vlfeat
include_directories(./vlfeat
./vlfeat/vl)

###################Matching
include_directories(.
./matching
 )

set(matching_srcs
        matching/siftdesc.cpp
        matching/matching.cpp
        orsa.cpp
)
add_library(matching ${matching_srcs})
TARGET_LINK_LIBRARIES(matching ${OpenCV_LIBS} degensac lapack Match Numerics)

################## View synthesis

set(synth-detection_srcs
        synth-detection.cpp)

include_directories(.)

add_library(synth-detection ${synth-detection_srcs})

if(WITH_CAFFE)
TARGET_LINK_LIBRARIES(synth-detection matching opensurf
mser AKAZE affinedetectors ${OpenCV_LIBS} helpers vl caffe glog boost_system  protobuf lmdb blas)
else(WITH_CAFFE)
TARGET_LINK_LIBRARIES(synth-detection matching opensurf
mser affinedetectors AKAZE helpers vl ${OpenCV_LIBS})
endif(WITH_CAFFE)



add_executable(mods mods.cpp correspondencebank.cpp imagerepresentation.cpp  io_mods.cpp)
if(WITH_CAFFE)
TARGET_LINK_LIBRARIES(mods inih matching synth-detection mser affinedetectors scv
AKAZE vl mrogh caffe glog boost_system  protobuf leveldb saddlepts ${OpenCV_LIBS} vl m )
else(WITH_CAFFE)
TARGET_LINK_LIBRARIES(mods inih matching synth-detection mser affinedetectors scv AKAZE vl mrogh saddlepts
${OpenCV_LIBS} z vl m)
endif(WITH_CAFFE)

set (TT FALSE)
if (TT)
add_executable(extract_features extract_features.cpp correspondencebank.cpp imagerepresentation.cpp io_mods.cpp)
if(WITH_CAFFE)
TARGET_LINK_LIBRARIES(extract_features inih matching synth-detection mser affinedetectors scv
AKAZE vl mrogh saddlepts caffe glog boost_system  protobuf leveldb ${OpenCV_LIBS} blas m z)
else(WITH_CAFFE)
TARGET_LINK_LIBRARIES(extract_features inih matching synth-detection mser affinedetectors scv AKAZE vl mrogh saddlepts
${OpenCV_LIBS} m z)
endif(WITH_CAFFE)

add_executable(extract_regions_for_benchmark extract_regions_for_benchmark correspondencebank.cpp
imagerepresentation.cpp io_mods.cpp)
if(WITH_CAFFE)
TARGET_LINK_LIBRARIES(extract_regions_for_benchmark inih matching synth-detection mser affinedetectors scv
AKAZE vl mrogh saddlepts caffe glog boost_system  protobuf leveldb ${OpenCV_LIBS} m z blas)
else(WITH_CAFFE)
TARGET_LINK_LIBRARIES(extract_regions_for_benchmark inih matching synth-detection mser affinedetectors scv AKAZE vl mrogh saddlepts
${OpenCV_LIBS} m z)
endif(WITH_CAFFE)

add_executable(export_descriptors export_descriptors correspondencebank.cpp
imagerepresentation.cpp io_mods.cpp )
if(WITH_CAFFE)
TARGET_LINK_LIBRARIES(export_descriptors inih matching synth-detection mser affinedetectors scv
AKAZE vl mrogh caffe glog boost_system  protobuf leveldb saddlepts ${OpenCV_LIBS} blas m z)
else(WITH_CAFFE )
TARGET_LINK_LIBRARIES(export_descriptors inih matching synth-detection mser affinedetectors scv AKAZE vl mrogh saddlepts
 opencv_core m  z)
endif(WITH_CAFFE)
#${OpenCV_LIBS}
add_executable(mods_multi mods_multi.cpp correspondencebank.cpp
imagerepresentation.cpp io_mods.cpp )
if(WITH_CAFFE)
TARGET_LINK_LIBRARIES(mods_multi inih matching synth-detection mser affinedetectors scv
AKAZE vl mrogh caffe glog boost_system  protobuf leveldb saddlepts ${OpenCV_LIBS} blas m z)
else(WITH_CAFFE)
TARGET_LINK_LIBRARIES(mods_multi inih matching synth-detection mser affinedetectors scv AKAZE vl mrogh saddlepts
${OpenCV_LIBS} m z)
endif(WITH_CAFFE)

endif(TT)
