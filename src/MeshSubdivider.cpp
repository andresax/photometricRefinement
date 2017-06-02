#include <MeshSubdivider.h>
#include <CGAL/Iterator_range.h>


#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <boost/function_output_iterator.hpp>

namespace PMP = CGAL::Polygon_mesh_processing;
typedef boost::graph_traits<MeshSurface>::face_iterator fac_it;

MeshSubdivider::MeshSubdivider() {
  areaMax_ = 16;
}

MeshSubdivider::~MeshSubdivider() {
}

void MeshSubdivider::subdivide(MeshSurface &p, glm::mat4 cameraMatrix) {

  double target_edge_length = 0.04;
  unsigned int nb_iter = 3;
  std::vector<MeshSurface::Face_index> fiv;
  fiv.push_back(MeshSurface::Face_index(0));
  fiv.push_back(MeshSurface::Face_index(2));
  fiv.push_back(MeshSurface::Face_index(3));


  std::cout << "Split border...";
  std::cout << "done." << std::endl;
  //PMP::isotropic_remeshing(faces(p), target_edge_length, p, PMP::parameters::number_of_iterations(nb_iter));
  PMP::isotropic_remeshing(CGAL::make_range(fiv.begin(), fiv.end()), target_edge_length, p, PMP::parameters::number_of_iterations(nb_iter));
  std::cout << "Remeshing done." << std::endl;
}
