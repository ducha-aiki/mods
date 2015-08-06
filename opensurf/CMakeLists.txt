CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(opensurf) #DoG-Affine, HarrisAffine and HessianAffine

FIND_PACKAGE(OpenCV)
if (OPENCV_FOUND)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenCV_CXX_FLAGS}")
endif (OPENCV_FOUND)

IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-strict-aliasing")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra -Wno-write-strings")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -ansi")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ftree-vectorize -funroll-loops")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)


set(opensurf_srcs
       fasthessian.cpp
       integral.cpp
       ipoint.cpp
       surf.cpp
       utils.cpp
)
include_directories(.)

add_library(opensurf ${opensurf_srcs})
TARGET_LINK_LIBRARIES(opensurf ${OpenCV_LIBS})
