cmake_minimum_required(VERSION 3.0)
 
project(photometricRefinementProject)   
set( CMAKE_BUILD_TYPE "Release")
 
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
                      ${OPENGL_INCLUDE_DIR} 
                      ${GLEW_INCLUDE_DIRS}
                      ${GMPXX_INCLUDE_DIR}
                      ${OpenCV_INCLUDE_DIRS} 
                      ${CGAL_INCLUDE_DIRS}
		      ${GLM_INCLUDE_DIRS}/glm/
                      photometricGradient/include/photometricGradient/
                      photometricGradient/include/
                      bestViewsToBeCompared/include/
                      external/
                      include/photometricRefinement/
                      include/test/
                      include/photometricRefinement/misc/
                      include/photometricRefinement/subdivider/
                      external/TransforMesh/include
                      ) 

# ------------------------ MESSAGES ----------------------------
 
# ------------------------ BUILD -------------------------------

file(GLOB_RECURSE meshRec_SOURCE 
                  src/*.cpp
                  src/misc/*.cpp
                  src/subdivider/*.cpp
                  test/*.cpp
                  )

add_executable(photometricRefinement ${meshRec_SOURCE})
target_link_libraries(photometricRefinement ${OpenCV_LIBS} 
                                            ${MPRF_LIBRARIES}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
                                            ${OPENGL_LIBRARIES}
                                            ${GLEW_LIBRARIES}
					    ${GLM_LIBRARIES}
					    glfw 
                                            ${GLFW_LIBRARIES}
                                            TransforMesh
                                            mpfr
                                            photometricGradient
                                            bestViewsToBeCompared
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES})


