cmake_minimum_required(VERSION 3.2)
set( CMAKE_BUILD_TYPE "Debug")
set( CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE ) # avoid annoying messages
project(TransforMesh)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# include
include_directories(${PROJECT_SOURCE_DIR}/)

# target
add_library(mesh Mesh.cpp MeshNoiser.cpp MeshIntersections.cpp)
