cmake_minimum_required(VERSION 3.2)
set( CMAKE_BUILD_TYPE "Debug")
set( CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE ) # avoid annoying messages
project(TransforMesh)

# find dependencies
find_package(CGAL REQUIRED COMPONENTS Core )
include( ${CGAL_USE_FILE} )

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# defines
ADD_DEFINITIONS( "-DCGAL_USE_F2C -DCGAL_NDEBUG -DCONVEX_SEG -DWITH_CGAL_BLAS_NO -DCGAL_CFG_NO_CPP0X_VARIADIC_TEMPLATES" )

# include
include_directories(	${PROJECT_SOURCE_DIR}/	)

# target
add_library(meshNoiser MeshNoiser.cpp)
add_library(mesh Mesh.cpp)
add_library(meshIntersections MeshIntersections.cpp)
