add_executable(main Main.cpp TestMeshSubdivision.cpp)
target_link_libraries(main photometricRefinement bestViewsToBeCompared photometricGradient mesh ${EXTERNAL_LIBRARIES})
