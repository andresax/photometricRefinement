cmake_minimum_required(VERSION 2.8.3)

# ----------------------- LIBRARIES ---------------------------
find_package(Boost COMPONENTS filesystem system thread REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLUT REQUIRED)
find_package(glm REQUIRED)
find_package(CGAL REQUIRED)
find_package(GLEW REQUIRED)
find_package(MPFR REQUIRED) 
find_package(OpenMP) 

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

link_directories(${PROJECT_SOURCE_DIR}/../openGLUtilities/build/)
link_directories(${PROJECT_SOURCE_DIR}/../camParsers/build/)

# ----------------------- INCLUDES ----------------------------
include_directories(  
                      ${OPENGL_INCLUDE_DIR} 
                      ${GLEW_INCLUDE_DIRS}
                      ${GMPXX_INCLUDE_DIR} 
                      ${CGAL_INCLUDE_DIRS}
                      ${SOIL_INCLUDE_DIRS}
                      ${GLFW_INCLUDE_DIRS}
		      ${GLM_INCLUDE_DIRS}/glm/
                      ${PROJECT_SOURCE_DIR}/../include/photometricRefinement/misc/
                      ${PROJECT_SOURCE_DIR}/../photometricGradient/include/photometricGradient/
                      ${PROJECT_SOURCE_DIR}/../camParsers/include/camParser/
                      ${PROJECT_SOURCE_DIR}/../misc/include/
                      ${PROJECT_SOURCE_DIR}/include
                      ${PROJECT_SOURCE_DIR}/test
                      ${PROJECT_SOURCE_DIR}/../include
                      ${PROJECT_SOURCE_DIR}/../external/TransforMesh/include
                      ../external/
                      ./include/
                      ../photometricGradient/include/photometricGradient/
                      ../external/TransforMesh/include/
                      ../misc/include/
                      ../include/photometricRefinement/misc/
                      ) 

# ------------------------ MESSAGES ----------------------------
 
# ------------------------ BUILD -------------------------------

file(GLOB_RECURSE bestViewsToBeCompared_SOURCE 
                  src/*.cpp
                  # test/*.cpp
                  ../external/opengm/src/external/MaxFlow-v3.02.src-patched/*.cpp
                  )

file(GLOB_RECURSE bestViewsToBeCompared_H 
                  ./include/*.h
                  )

add_library(bestViewsToBeCompared ${bestViewsToBeCompared_SOURCE} ${bestViewsToBeCompared_H})
target_link_libraries(bestViewsToBeCompared ${OpenCV_LIBS} 
                                            ${MPRF_LIBRARIES}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
					    ${GLM_LIBRARIES}
                                            ${OPENGL_LIBRARIES}
                                            ${GLEW_LIBRARIES}
                                            ${GLFW_STATIC_LIBRARIES}
                                            photometricGradient
                                            mpfr
                                            TransforMesh
                                            glfw3
                                            gmp 
                                            gmpxx
                                            ${CGAL_LIBRARY}
                                            ${Boost_LIBRARIES})


# ------------------------ INSTALLATION ------------------------


# --------------------------------------------------------------


