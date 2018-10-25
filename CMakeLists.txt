cmake_minimum_required(VERSION 3.12)
 
project(photometricRefinement)   
set( CMAKE_BUILD_TYPE "Debug")
set( CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE) 
set( OpenGL_GL_PREFERENCE "GLVND" )

# ----------------------- LIBRARIES ---------------------------
find_package(CGAL REQUIRED)
include( ${CGAL_USE_FILE} )
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED) 
find_package(glm REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenMP REQUIRED) 
find_package(Boost COMPONENTS system filesystem thread REQUIRED)

set(EXTERNAL_LIBRARIES ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${OpenCV_LIBRARIES} ${GLUT_LIBRARIES} ${CGAL_LIBRARIES} glfw glm)


if (OPENMP_FOUND) 
    message("OpenMP found!")
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

# ----------------------- INCLUDES ----------------------------
include_directories( 	${PROJECT_SOURCE_DIR}/include/photometricRefinement/
			${PROJECT_SOURCE_DIR}/photometricGradient/include/ 
			${PROJECT_SOURCE_DIR}/bestViewsToBeCompared/include/ 
			${PROJECT_SOURCE_DIR}/external/ ) 

link_directories( "photometricGradient/build/src/" )	      
link_directories( "external/TransforMesh/build/" )	      
link_directories( "bestViewsToBeCompared/build/src/" )	      
 
# ------------------------ BUILD -------------------------------
message("**** HERE TEST VARIABLES" ) 
message("${EXTERNAL_LIBRARIES}")
message("**** HERE TEST VARIABLES" ) 

add_subdirectory("src")
add_subdirectory("test")
add_subdirectory("test/bestViewsToBeCompared")
