#include <MeshSubdivider.h>
#include <CGAL/Iterator_range.h>

#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <boost/function_output_iterator.hpp>
#include <utilities.hpp>

namespace PMP = CGAL::Polygon_mesh_processing;
typedef boost::graph_traits<MeshSurface>::face_iterator fac_it;
typedef MeshSurface::Halfedge_index halfedge_descriptor;
typedef MeshSurface::Vertex_index vertex_descriptor;
typedef MeshSurface::Face_index face_descriptor;
typedef MeshSurface::Edge_index edge_descriptor;

MeshSubdivider::MeshSubdivider() {
  areaMax_ = 16;
  numIt_ = 1;
}

MeshSubdivider::~MeshSubdivider() {
}

void MeshSubdivider::subdivide(MeshSurface &p, glm::mat4 cameraMatrix) {

  for (int curIt = 0; curIt < numIt_; ++curIt) {

    MeshSurface::Property_map<vertex_descriptor, Ker::Point_3> location = p.points();
    std::vector<halfedge_descriptor> eiv;
    std::vector<face_descriptor> fiv;


    for (auto he : p.halfedges()) {
      vertex_descriptor vTarget = CGAL::target(he, p);
      vertex_descriptor vSource = CGAL::source(he, p);

      bool toCheck = false;

      if (!CGAL::is_border((he), p)) {
        if (std::find(fiv.begin(), fiv.end(), CGAL::face(he, p)) == fiv.end()) {
          if (!CGAL::is_border(CGAL::opposite(he, p), p)) {
            if (std::find(fiv.begin(), fiv.end(), CGAL::face(CGAL::opposite(he, p), p)) == fiv.end()) {
              toCheck = true;
            }
          } else {
            toCheck = true;
          }
        }
      }

      if (toCheck) {

        vertex_descriptor vd1 = CGAL::target(he, p);
        vertex_descriptor vd2 = CGAL::target(CGAL::next(he, p), p);
        vertex_descriptor vd3 = CGAL::target(CGAL::next(CGAL::next(he, p), p), p);

        glm::vec2 pt2D_1 = utilities::projectPoint(cameraMatrix, glm::vec3(location[vd1].x(), location[vd1].y(), location[vd1].z()));
        glm::vec2 pt2D_2 = utilities::projectPoint(cameraMatrix, glm::vec3(location[vd2].x(), location[vd2].y(), location[vd2].z()));
        glm::vec2 pt2D_3 = utilities::projectPoint(cameraMatrix, glm::vec3(location[vd3].x(), location[vd3].y(), location[vd3].z()));

        float d1 = glm::length(pt2D_1 - pt2D_3);
        float d2 = glm::length(pt2D_1 - pt2D_2);
        float d3 = glm::length(pt2D_3 - pt2D_2);

        float area = 0.5 * glm::abs(orientPoint(pt2D_1, pt2D_2, pt2D_3));

        if (area > areaMax_ && d1 > d2 && d1 > d3) {
          eiv.push_back(he);
          fiv.push_back(CGAL::face(he, p));
          if (!CGAL::is_border(CGAL::opposite(he, p), p)) {
            fiv.push_back(CGAL::face(CGAL::opposite(he, p), p));
          }
        }
      }
    }

    std::cout << "Split border...it num." << curIt << std::endl;
    for (auto he : eiv) {

      vertex_descriptor vTarget = CGAL::target(he, p);
      vertex_descriptor vSource = CGAL::source(he, p);

      halfedge_descriptor he2 = CGAL::opposite(he, p);
      vertex_descriptor vd1 = CGAL::target(he, p);
      vertex_descriptor vd2 = CGAL::source(he, p);

      halfedge_descriptor heCur = CGAL::Euler::split_edge(he, p);
      vertex_descriptor vdCur = CGAL::target(heCur, p);


      location[vdCur] = Ker::Point_3((location[vd1].x() + location[vd2].x()) / 2, (location[vd1].y() + location[vd2].y()) / 2,
          (location[vd1].z() + location[vd2].z()) / 2);
      CGAL::Euler::split_face(heCur, CGAL::next(he, p), p);

      if (!CGAL::is_border(CGAL::opposite(he, p), p)) {
        CGAL::Euler::split_face(CGAL::opposite(he, p), CGAL::next(CGAL::opposite(heCur, p), p), p);
      }
    }
    std::cout << "done." << std::endl;

  }

//  std::ofstream file("out.off");
//  file << p;
//  std::cout << "done." << std::endl;
//  file.close();
//  //exit(0);

  std::cout << "Remeshing done." << std::endl;
}
/*
 void MeshSubdivider::subdivideold(MeshSurface &p, glm::mat4 cameraMatrix) {

 double target_edge_length = 0.04;
 std::vector<face_descriptor> fiv;

 MeshSurface::Property_map<vertex_descriptor, Ker::Point_3> location = p.points();

 for (auto f : p.faces()) {
 halfedge_descriptor he1 = CGAL::halfedge(f, p);
 halfedge_descriptor he2 = CGAL::next(CGAL::halfedge(f, p), p);
 halfedge_descriptor he3 = CGAL::next(CGAL::next(CGAL::halfedge(f, p), p), p);
 vertex_descriptor vd1 = CGAL::target(he1, p);
 vertex_descriptor vd2 = CGAL::target(he2, p);
 vertex_descriptor vd3 = CGAL::target(he3, p);

 glm::vec2 pt2D_1 = utilities::projectPoint(cameraMatrix, glm::vec3(location[vd1].x(), location[vd1].y(), location[vd1].z()));
 glm::vec2 pt2D_2 = utilities::projectPoint(cameraMatrix, glm::vec3(location[vd2].x(), location[vd2].y(), location[vd2].z()));
 glm::vec2 pt2D_3 = utilities::projectPoint(cameraMatrix, glm::vec3(location[vd3].x(), location[vd3].y(), location[vd3].z()));

 float area = 0.5 * glm::abs(orientPoint(pt2D_1, pt2D_2, pt2D_3));

 if (area > areaMax_) {
 fiv.push_back(f);
 }
 }

 std::cout << "Split border...";
 std::cout << "done." << std::endl;

 PMP::isotropic_remeshing(CGAL::make_range(fiv.begin(), fiv.end()), target_edge_length, p, PMP::parameters::number_of_iterations(1));

 std::cout << "Remeshing done." << std::endl;
 }*/

float MeshSubdivider::orientPoint(const glm::vec2& v0, const glm::vec2& v1, const glm::vec2& p) {
  glm::mat2 m;
  m[0][0] = (v1.x - v0.x);
  m[0][1] = (p.x - v0.x);
  m[1][0] = (v1.y - v0.y);
  m[1][1] = (p.y - v0.y);

  return m[0][0] * m[1][1] - m[0][1] * m[1][0];
}
