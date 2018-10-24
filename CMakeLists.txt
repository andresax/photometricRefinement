cmake_minimum_required(VERSION 3.0)
 
project(photometricRefinementProject)   
set( CMAKE_BUILD_TYPE "Debug")
set( OpenGL_GL_PREFERENCE "GLVND" )
set(photometricRefinement_LIBRARIES "configurator photometricRefinement surfaceEvolver openMvgParser utilities meshSubdivider polySubdivider polySubdividerOpenGl subdivider checkReprojAreaProgram testMeshSubdivisions")

#----------------------- CMAKE MODULES ------------------------ 

# ----------------------- LIBRARIES ---------------------------
find_package(Boost COMPONENTS filesystem system thread REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(CGAL REQUIRED)
find_package(GLEW REQUIRED) 
find_package(glm REQUIRED)
find_package(glfw3 REQUIRED)
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
endif()

add_subdirectory("external/TransforMesh")
add_subdirectory("photometricGradient")
add_subdirectory("bestViewsToBeCompared")
add_subdirectory("src")
add_subdirectory("src/misc")
add_subdirectory("src/subdivider")
add_subdirectory("test")
  
# ----------------------- INCLUDES ----------------------------
include_directories( 	${PROJECT_SOURCE_DIR}/include/photometricRefinement/
			${PROJECT_SOURCE_DIR}/photometricGradient/include/ 
			${PROJECT_SOURCE_DIR}/bestViewsToBeCompared/include/ 
			${PROJECT_SOURCE_DIR}/external/ ) 

link_directories( ${photometricGradient_BUILD_DIR}/build/src )	      
link_directories( ${transforMesh_BUILD_DIR}/build/src )	      
link_directories( ${bestViewsToBeCompared_BUILD_DIR}/build/src )	      
 
# ------------------------ BUILD -------------------------------
message("**** HERE TEST VARIABLES" ) 
message("${bestViewsToBeCompared_LIBRARIES}") 
message("${photometricGradient_LIBRARIES}") 
message("${photometricRefinement_LIBRARIES}") 
message("${photometricGradient_BUILD_DIR}")
message("${transforMesh_BUILD_DIR}")
message("${bestViewsToBeCompared_BUILD_DIR}")
message("**** HERE TEST VARIABLES" ) 
