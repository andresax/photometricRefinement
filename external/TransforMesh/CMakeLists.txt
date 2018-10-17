cmake_minimum_required(VERSION 2.6)
project(TransforMesh)

# find dependencies
find_package(CGAL REQUIRED COMPONENTS Core )

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
# include dependencies
include( ${CGAL_USE_FILE} )

# defines
ADD_DEFINITIONS( "-DCGAL_USE_F2C -DCGAL_NDEBUG -DCONVEX_SEG -DWITH_CGAL_BLAS_NO -DCGAL_CFG_NO_CPP0X_VARIADIC_TEMPLATES" )

# sources
set(proj_SRC
  MeshNoiser.cpp
  Mesh.cpp
  MeshIntersections.cpp
  ../Utils/GeoIntegral.cpp
)

set(proj_H
  MeshNoiser.h
  Mesh.h
  ../Utils/GeoIntegral.h
)

# include
include_directories(
#  ${CMAKE_CURRENT_SOURCE_DIR}
  "../Utils/"
  "../"
#  "./"
)

# target
add_library(TransforMesh SHARED ${proj_SRC} ${proj_H})
