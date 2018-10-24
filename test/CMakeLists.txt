add_library(testMeshSubdivision TestMeshSubdivision.cpp)

add_executable(main Main.cpp)
target_link_libraries(main ${photometricRefinement_LIBRARIES} ${bestViewsToBeCompared_LIBRARIES} ${photometricRefinement_LIBRARIES})
