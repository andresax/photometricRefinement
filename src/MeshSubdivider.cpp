#include <MeshSubdivider.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <boost/function_output_iterator.hpp>

typedef boost::graph_traits<Polyhedron>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<Polyhedron>::edge_descriptor edge_descriptor;

namespace PMP = CGAL::Polygon_mesh_processing;

MeshSubdivider::MeshSubdivider() {
  areaMax_ = 16;
}

MeshSubdivider::~MeshSubdivider() {
}

void MeshSubdivider::subdivide(Polyhedron &p, glm::mat4 cameraMatrix) {

  double target_edge_length = 0.04;
  unsigned int nb_iter = 3;
  std::cout << "Split border...";
  std::cout << "done." << std::endl;
  PMP::isotropic_remeshing(faces(p), target_edge_length, p, PMP::parameters::number_of_iterations(nb_iter));
  std::cout << "Remeshing done." << std::endl;
}
