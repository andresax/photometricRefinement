#ifndef SRC_MESHSUBDIVIDER_H_
#define SRC_MESHSUBDIVIDER_H_

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <Subdivider.h>

typedef CGAL::Simple_cartesian<double>                   Ker;
typedef CGAL::Surface_mesh<Ker::Point_3> MeshSurface;

class MeshSubdivider : public Subdivider {
public:
  MeshSubdivider();
  MeshSubdivider(int areaMax, int numIt);
  virtual ~MeshSubdivider();

  void subdivide(MeshSurface &p, glm::mat4 cameraMatrix);

};

#endif /* SRC_MESHSUBDIVIDER_H_ */
