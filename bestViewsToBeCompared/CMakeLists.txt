cmake_minimum_required(VERSION 2.8.3)
set( CMAKE_BUILD_TYPE "Debug")
set( CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE ) # avoid annoying messages
set( OpenGL_GL_PREFERENCE "GLVND" )
project(bestViewsToBeCompared)

# ----------------------- LIBRARIES ---------------------------
#find_package(Boost COMPONENTS filesystem system thread REQUIRED)
#find_package(PkgConfig REQUIRED)
#find_package(glfw3 REQUIRED)
#find_package(OpenGL REQUIRED)
#find_package(OpenCV REQUIRED)
#find_package(GLUT REQUIRED)
#find_package(glm REQUIRED)
#find_package(CGAL REQUIRED)
#find_package(GLEW REQUIRED)
#find_package(MPFR REQUIRED) 
#find_package(OpenMP) 

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
include_directories(	include/bestViewsToBeCompared/ 
			include/photometricGradient/
			include/TransforMesh/
			include/photometricRefinement/misc/
			) 

add_subdirectory(src)
