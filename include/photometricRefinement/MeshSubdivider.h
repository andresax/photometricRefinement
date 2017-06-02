#ifndef SRC_MESHSUBDIVIDER_H_
#define SRC_MESHSUBDIVIDER_H_




#include <glm.hpp>


#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

typedef CGAL::Simple_cartesian<double>                   Ker;
typedef CGAL::Surface_mesh<Ker::Point_3> MeshSurface;

class MeshSubdivider {
public:
  MeshSubdivider();
  virtual ~MeshSubdivider();

  void subdivide(MeshSurface &p, glm::mat4 cameraMatrix);

  void setAreaMax(int areaMax) {
    areaMax_ = areaMax;
  }

private:

  int areaMax_;
};

#endif /* SRC_MESHSUBDIVIDER_H_ */