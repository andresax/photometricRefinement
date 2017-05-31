#ifndef TEST_TESTMESHSUBDIVISION_H_
#define TEST_TESTMESHSUBDIVISION_H_

#include <typesRefinement.hpp>
#include <Mesh.h>

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
  Mesh mesh_;


};

#endif /* TEST_TESTMESHSUBDIVISION_H_ */
