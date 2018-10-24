#ifndef SRC_POLYSUBDIVIDER_H_
#define SRC_POLYSUBDIVIDER_H_

#include <vector>
#include <subdivider/Subdivider.h>
#include <TransforMesh/Mesh.h>

class PolySubdivider : public Subdivider {
public:
  PolySubdivider();
  PolySubdivider(int areaMax, int numIt);
  virtual ~PolySubdivider();
  void subdivide(Mesh &p, glm::mat4 cameraMatrix);
  void subdivide(Mesh &p, std::vector<glm::mat4> cameraMatrices);
};

#endif /* SRC_POLYSUBDIVIDER_H_ */
