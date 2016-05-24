CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND CMAKE_POLICY)


set(PROJECT_NAME "TILDE")
project(${PROJECT_NAME})


find_package(OpenCV )

FIND_PACKAGE( OpenMP QUIET)
if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


set(SRCS src/libTILDE.cpp src/NonMaxSup.cpp)
set(DEMOSRC src/demo.cpp)
set(DEMOEXE demo)


#option(WITH_TEST_GENERATORS "Build Generator tests" OFF)
#option(WITH_TEST_STATIC "Build Generator tests" OFF)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

IF(UNIX)
  IF (NOT APPLE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DISLINUX")
  else()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DISMAC")
  endif()
endif()

if (OpenCV_FOUND)
  include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/3rdParties ${OPENCV_INCLUDE_DIR})

  add_library(${PROJECT_NAME} SHARED ${SRCS})
  add_library(${PROJECT_NAME}_static STATIC ${SRCS})
  target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

  add_executable(${DEMOEXE} ${DEMOSRC})
  target_link_libraries(${DEMOEXE} ${PROJECT_NAME})


  
  set(TILDE_LIBRARY_FULLPATH $<TARGET_FILE:${PROJECT_NAME}_static>)
  set(TILDE_LIBRARY_PROJECT ${PROJECT_NAME} CACHE STRING "${PROJECT_NAME}" FORCE )


  install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${PROJECT_SOURCE_DIR})

else()    
  set( OT_MISSING_DEPS "")

  if(NOT OPENCV_FOUND)
    set(OT_MISSING_DEPS "OpenCV, ${OT_MISSING_DEPS}")
  endif()

  message(STATUS "NOTICE: This project requires ${OT_MISSING_DEPS} and will not be compiled.")
endif()

