CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(daisy) 

IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-strict-aliasing")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra -Wno-write-strings")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -ansi")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ftree-vectorize -funroll-loops")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

set(daisy_srcs
       src/corecv.cpp
       src/daisy.cpp
       src/general.cpp
       src/image_manipulation.cpp
       src/interaction.cpp)

include_directories(
.
./include
./include/daisy
./include/kutility)

add_library(daisy ${daisy_srcs})
TARGET_LINK_LIBRARIES(daisy helpers ${OpenCV_LIBS})
