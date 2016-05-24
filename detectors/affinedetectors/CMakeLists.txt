CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(AffineDetectors) #DoG-Affine, HarrisAffine and HessianAffine

#FIND_PACKAGE(OpenCV)
#if (OPENCV_FOUND)
#  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenCV_CXX_FLAGS}")
#endif (OPENCV_FOUND)

IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-strict-aliasing")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra -Wno-write-strings")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -ansi")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ftree-vectorize -funroll-loops")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)


set(affdets_srcs
       scale-space-detector.cpp
       affine.cpp
       pyramid.cpp
)
include_directories(.)
if(WITH_CAFFE)
include_directories(/home/old-ufo/clear-caffe/include)
include_directories(/home/old-ufo/clear-caffe/build/src/)
link_directories(/home/old-ufo/clear-caffe/build/lib)
endif(WITH_CAFFE)

add_library(affinedetectors ${affdets_srcs})

if(WITH_CAFFE)
TARGET_LINK_LIBRARIES(affinedetectors ${OpenCV_LIBS} helpers caffe glog protobuf leveldb openblas)
else(WITH_CAFFE)
TARGET_LINK_LIBRARIES(affinedetectors helpers ${OpenCV_LIBS} TILDE)
endif(WITH_CAFFE)
