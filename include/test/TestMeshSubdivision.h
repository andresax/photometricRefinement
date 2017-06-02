#ifndef TEST_TESTMESHSUBDIVISION_H_
#define TEST_TESTMESHSUBDIVISION_H_

#include <typesRefinement.hpp>
#include <MeshSubdivider.h>

class TestMeshSubdivision {
public:
  TestMeshSubdivision();
  virtual ~TestMeshSubdivision();

  void testMeshSubdivision();
private:

  void createMesh();
  void createCamera();
  void showMeshOnCamera();

  photometricGradient::CameraType camera_;
  MeshSurface mesh_;


};

#endif /* TEST_TESTMESHSUBDIVISION_H_ */
