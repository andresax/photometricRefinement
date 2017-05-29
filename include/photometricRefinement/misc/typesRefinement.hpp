#ifndef TYPESPHOTOMETRICREFINEMENT_HPP_
#define TYPESPHOTOMETRICREFINEMENT_HPP_

#include <glm.hpp>

#include <vector>
#include <types.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_hierarchy_3.h>
#include <CGAL/Triangulation_cell_base_with_info_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/intersections.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>




struct PhotometricRefinementConfiguration {
  std::string fileOpenMVG_;
  std::string pathInitMesh_;
  std::string pathOutputDir_;
  float lambdaPhoto_;
  float lambdaSmooth_;
  float maxDistanceCamFeatureRef;
  int numIt_;
  float ensureedgemax_;
  int ensureedgeit_;
  std::string optionalNameDir_;
};
 
struct SfMData {

  int numPoints_;
  int numCameras_;

  std::vector<glm::vec3> points_;
  std::vector<photometricGradient::CameraType> camerasList_;
  std::vector<std::string> camerasPaths_;

  std::vector<std::vector<int> > camViewingPointN_;
  std::vector<std::vector<int> > pointsVisibleFromCamN_;
  std::vector<std::vector<glm::vec2> > point2DoncamViewingPoint_;

  int imageWidth_, imageHeight_;
};


#endif /* TYPESSEMANTICREC_HPP_ */
