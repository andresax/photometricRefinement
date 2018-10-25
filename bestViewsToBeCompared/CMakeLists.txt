cmake_minimum_required(VERSION 2.8.3)
set( CMAKE_BUILD_TYPE "Debug")
set( CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE ) # avoid annoying messages
set( OpenGL_GL_PREFERENCE "GLVND" )
project(bestViewsToBeCompared)

# ----------------------- LIBRARIES ---------------------------

if (OPENMP_FOUND) 
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -frounding-math  ")
endif()
if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -frounding-math  ")
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math  ")
endif()

# ----------------------- INCLUDES ----------------------------
include_directories(	
			${PROJECT_SOURCE_DIR}/include/bestViewsToBeCompared/
			${PROJECT_SOURCE_DIR}/include/
			) 

add_subdirectory(src)
