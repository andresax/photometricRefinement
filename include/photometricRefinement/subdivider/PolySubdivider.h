#ifndef SRC_POLYSUBDIVIDER_H_
#define SRC_POLYSUBDIVIDER_H_

#include <Subdivider.h>
#include <Mesh.h>
#include <vector>

class PolySubdivider : public Subdivider {
public:
  PolySubdivider();
  PolySubdivider(int areaMax, int numIt);
  virtual ~PolySubdivider();
  void subdivide(Mesh &p, glm::mat4 cameraMatrix);
  void subdivide(Mesh &p, std::vector<glm::mat4> cameraMatrices);
};

#endif /* SRC_POLYSUBDIVIDER_H_ */
