CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(AffineDetectors) #DoG-Affine, HarrisAffine and HessianAffine


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

add_library(affinedetectors ${affdets_srcs})


TARGET_LINK_LIBRARIES(affinedetectors helpers opencv_features2d opencv_core opencv_highgui)
