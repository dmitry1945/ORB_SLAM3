cmake_minimum_required(VERSION 2.8)
project(DBoW2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

  FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system)

# FIND_PACKAGE(Boost)
#IF (Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
    ADD_DEFINITIONS( "-DHAS_BOOST" )
#ENDIF()
set(Boost_USE_STATIC_LIBS   ON)
message("Boost_INCLUDE_DIR - ${Boost_INCLUDE_DIR}")

  find_package(Eigen3 REQUIRED)
message("EIGEN3_INCLUDE_DIRS - ${EIGEN3_INCLUDE_DIRS}")

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
  #  find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
message("OpenCV_INCLUDE_DIRS - ${OpenCV_INCLUDE_DIRS}")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(Boost_LIBRARY_DIRS ${Boost_LIBRARY_DIR})

link_directories(${Boost_LIBRARY_DIRS})
message("boost_libs - ${Boost_LIBRARY_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 STATIC ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS} ${Boost_LIBRARIES})

