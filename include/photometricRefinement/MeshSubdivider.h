#ifndef SRC_MESHSUBDIVIDER_H_
#define SRC_MESHSUBDIVIDER_H_

#include <Mesh.h>

class MeshSubdivider {
public:
  MeshSubdivider();
  virtual ~MeshSubdivider();

  void subdivide(Polyhedron &p, glm::mat4 cameraMatrix);

  void setAreaMax(int areaMax) {
    areaMax_ = areaMax;
  }

private:

  int areaMax_;
};

#endif /* SRC_MESHSUBDIVIDER_H_ */
