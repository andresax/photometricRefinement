cmake_minimum_required(VERSION 2.8.3)
 
project(photometricRefinement)  
 
#----------------------- CMAKE MODULES ------------------------ 

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)  
    
# ----------------------- LIBRARIES ---------------------------
find_package(Boost COMPONENTS filesystem system thread REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(CGAL REQUIRED)
find_package(GLEW REQUIRED) 
find_package(OpenCV REQUIRED)
find_package(MPRF REQUIRED) 
find_package(OpenMP) 

if (OPENMP_FOUND) 
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ----------------------- GCC FLAGS ----------------------------
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  link_directories(external/TransforMesh/build)
  MESSAGE( STATUS "CMAKE_BUILD_TYPE:    release->     " ${CMAKE_BUILD_TYPE} )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math  ")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "")
  link_directories(external/TransforMesh/build)
  MESSAGE( STATUS "CMAKE_BUILD_TYPE:    release->     " ${CMAKE_BUILD_TYPE} )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math  ")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  link_directories(external/TransforMesh/buildDebug)
  MESSAGE( STATUS "CMAKE_BUILD_TYPE:  debug->      " ${CMAKE_BUILD_TYPE} )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math  ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math -fsanitize=address ")
endif()

add_subdirectory(external/glfw)
add_subdirectory(photometricGradient)
 
# ----------------------- INCLUDES ----------------------------
include_directories(
                      ${OPENGL_INCLUDE_DIR} 
                      ${GLEW_INCLUDE_DIRS}
                      ${GMPXX_INCLUDE_DIR}
                      ${OpenCV_INCLUDE_DIRS} 
                      ${CGAL_INCLUDE_DIRS}
                      external/glm/glm/
                      external/glfw/include/
                      photometricGradient/include/photometricGradient/
                      external/
                      include/photometricRefinement/
                      include/photometricRefinement/misc/
                      external/TransforMesh/include
                      ) 

# ------------------------ MESSAGES ----------------------------
 
# ------------------------ BUILD -------------------------------

file(GLOB_RECURSE meshRec_SOURCE 
                  src/*.cpp
                  src/misc/*.cpp
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
					                                  glfw ${GLFW_LIBRARIES}
                                            TransforMesh
                                            photometricGradient
                                            -lmpfr
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES})


