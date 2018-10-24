cmake_minimum_required(VERSION 3.0)
 
project(photometricRefinementProject)   
set( CMAKE_BUILD_TYPE "Release")
set( OpenGL_GL_PREFERENCE "GLVND" )
 
#----------------------- CMAKE MODULES ------------------------ 

# ----------------------- LIBRARIES ---------------------------
find_package(Boost COMPONENTS filesystem system thread REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(CGAL REQUIRED)
find_package(GLEW REQUIRED) 
find_package(glm REQUIRED)
find_package(MPFR REQUIRED)  
find_package(OpenMP) 

if (OPENMP_FOUND) 
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
 
# ----------------------- GCC FLAGS ----------------------------
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  MESSAGE( STATUS "CMAKE_BUILD_TYPE:    release->     " ${CMAKE_BUILD_TYPE} )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math  ")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "")
  MESSAGE( STATUS "CMAKE_BUILD_TYPE:    release->     " ${CMAKE_BUILD_TYPE} )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math  ")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  MESSAGE( STATUS "CMAKE_BUILD_TYPE:  debug->      " ${CMAKE_BUILD_TYPE} )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math  ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math -fsanitize=address ")
endif()

add_subdirectory("external/TransforMesh")
add_subdirectory(photometricGradient)
add_subdirectory(bestViewsToBeCompared)
  
# ----------------------- INCLUDES ----------------------------
include_directories( 
                      photometricGradient/include/photometricGradient/
                      bestViewsToBeCompared/include/
                      include/photometricRefinement/
                      include/test/
                      include/photometricRefinement/misc/
                      include/photometricRefinement/subdivider/
                      external/
                      ) 

 
# ------------------------ BUILD -------------------------------
add_subdirectory(src)
add_subdirectory("src/misc")
add_subdirectory("src/subdivider")
add_subdirectory(test)
