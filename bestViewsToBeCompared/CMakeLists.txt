cmake_minimum_required(VERSION 2.8.3)

#----------------------- CMAKE MODULES ------------------------ 

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)  
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/modules) # main (top) cmake dir
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules) # project specific cmake dir
set(CMAKE_CXX_STANDARD 11) # tODO move up to a general cmake config for all sub projects ?
    
# ----------------------- LIBRARIES ---------------------------
find_package(Boost COMPONENTS filesystem system thread REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLUT REQUIRED)
find_package(CGAL REQUIRED)
find_package(GLEW REQUIRED)
find_package(MPRF REQUIRED) 
find_package(OpenMP) 

if (OPENMP_FOUND) 
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  link_directories(${PROJECT_SOURCE_DIR}/../photometricGradient/Release/)
  link_directories(../external/TransforMesh/build)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -frounding-math  ")
endif()
if(CMAKE_BUILD_TYPE STREQUAL "")
  link_directories(${PROJECT_SOURCE_DIR}/../photometricGradient/Release/)
  link_directories(../external/TransforMesh/build)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -frounding-math  ")
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  link_directories(${PROJECT_SOURCE_DIR}/../photometricGradient/Debug/)
  #link_directories(../external/TransforMesh/buildDebug)
  link_directories(../external/TransforMesh/build)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math  ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math -fsanitize=address ")
endif()
#add_subdirectory(external/glfw)
#message(STATUS "PROJECT_BINARY_DIR "${PROJECT_BINARY_DIR})
#message(STATUS "PROJECT_SOURCE_DIR "${PROJECT_SOURCE_DIR})
#message(STATUS "CMAKE_SOURCE_DIR "${CMAKE_SOURCE_DIR})
#message(STATUS "CMAKE_HOME_DIRECTORY "${CMAKE_HOME_DIRECTORY})
#message(STATUS "CMAKE_CURRENT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR})
#message(STATUS "CMAKE_CURRENT_LIST_FILE "${CMAKE_CURRENT_LIST_FILE})
#message(STATUS "CULO " ${GLFW_STATIC_LIBRARIES})



  link_directories(${PROJECT_SOURCE_DIR}/../openGLUtilities/build/)
  link_directories(${PROJECT_SOURCE_DIR}/../camParsers/build/)
#  link_directories(/usr/local/lib)
  #link_directories(${PROJECT_SOURCE_DIR}/../external/glfw/)`pkg-config glfw3 --static --libs`
# ----------------------- INCLUDES ----------------------------
include_directories(  
                      ${OPENGL_INCLUDE_DIR} 
                      ${GLEW_INCLUDE_DIRS}
                      ${GMPXX_INCLUDE_DIR} 
                      ${CGAL_INCLUDE_DIRS}
                      ${SOIL_INCLUDE_DIRS}
                      ${GLFW_INCLUDE_DIRS}
                      ${PROJECT_SOURCE_DIR}/../include/photometricRefinement/misc/
                      ${PROJECT_SOURCE_DIR}/../photometricGradient/include/photometricGradient/
                      ${PROJECT_SOURCE_DIR}/../camParsers/include/camParser/
                      ${PROJECT_SOURCE_DIR}/../misc/include/
                      ${PROJECT_SOURCE_DIR}/include
                      ${PROJECT_SOURCE_DIR}/test
                      ${PROJECT_SOURCE_DIR}/../include
                      ../external/glm/glm/
                      ${PROJECT_SOURCE_DIR}/../external/TransforMesh/include
                      ../external/
                      ../external/opengm/include
                      ./include/
                      ../photometricGradient/include/photometricGradient/
                      ../external/TransforMesh/include/
                      ../misc/include/
                      ../include/photometricRefinement/misc/
                      #../external/glfw/include
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
# add_executable(bestViewsToBeCompared ${bestViewsToBeCompared_SOURCE})

add_library(bestViewsToBeCompared ${bestViewsToBeCompared_SOURCE} ${bestViewsToBeCompared_H})
# target_include_directories (bestViewsToBeCompared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(bestViewsToBeCompared ${OpenCV_LIBS} 
                                            ${MPRF_LIBRARIES}
                                            ${GMPXX_LIBRARIES}
                                            ${GMP_LIBRARIES}
                                            ${LAPACK_LIBRARIES}
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


