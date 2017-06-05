#include <PolySubdivider.h>
#include <utilities.hpp>

PolySubdivider::PolySubdivider() {
}
PolySubdivider::PolySubdivider(int areaMax, int numIt) :
    Subdivider(areaMax, numIt) {
}

PolySubdivider::~PolySubdivider() {
}

void PolySubdivider::subdivide(Mesh &p, glm::mat4 cameraMatrix) {

  for (int curIt = 0; curIt < numIt_; ++curIt) {

    std::vector<Halfedge_handle> eiv;
    std::vector<Facet_handle> fiv;

    for (Halfedge_iterator heIt = p.p.halfedges_begin(); heIt != p.p.halfedges_end(); heIt++) {

      bool toCheck = false;

      if (!heIt->is_border()) {
        if (std::find(fiv.begin(), fiv.end(), heIt->facet()) == fiv.end()) {
          if (!heIt->opposite()->is_border()) {
            if (std::find(fiv.begin(), fiv.end(), heIt->opposite()->facet()) == fiv.end()) {
              toCheck = true;
            }
          } else {
            toCheck = true;
          }
        }
      }

      if (toCheck) {

        Vertex_handle vd1 = heIt->vertex();
        Vertex_handle vd2 = heIt->next()->vertex();
        Vertex_handle vd3 = heIt->next()->next()->vertex();

        glm::vec2 pt2D_1 = utilities::projectPoint(cameraMatrix, glm::vec3(vd1->point().x(), vd1->point().y(), vd1->point().z()));
        glm::vec2 pt2D_2 = utilities::projectPoint(cameraMatrix, glm::vec3(vd2->point().x(), vd2->point().y(), vd2->point().z()));
        glm::vec2 pt2D_3 = utilities::projectPoint(cameraMatrix, glm::vec3(vd3->point().x(), vd3->point().y(), vd3->point().z()));

        float d1 = glm::length(pt2D_1 - pt2D_3);
        float d2 = glm::length(pt2D_1 - pt2D_2);
        float d3 = glm::length(pt2D_3 - pt2D_2);

        float area = 0.5 * glm::abs(orientPoint(pt2D_1, pt2D_2, pt2D_3));

        if (area > areaMax_ && d1 > d2 && d1 > d3) {
          eiv.push_back(heIt);
          fiv.push_back(heIt->facet());
          if (!heIt->opposite()->is_border()) {
            fiv.push_back(heIt->opposite()->facet());
          }
        }
      }
    }

    // std::cout << "Split border...it num." << curIt << std::endl;
    for (auto he : eiv) {

      Vertex_handle vd1 = he->vertex();
      Vertex_handle vd2 = he->prev()->vertex();

      Halfedge_handle heCur = p.p.split_edge(he);

      heCur->vertex()->point() = Point((vd1->point().x() + vd2->point().x()) / 2, (vd1->point().y() + vd2->point().y()) / 2,
          (vd1->point().z() + vd2->point().z()) / 2);

      p.p.split_facet(heCur, he->next());

      if (!he->opposite()->is_border()) {
        p.p.split_facet(he->opposite(), heCur->opposite()->next());
      }
    }
    //std::cout << "done." << std::endl;

  }

  //std::cout << "Remeshing done." << std::endl;
}

void PolySubdivider::subdivide(Mesh &p, std::vector<glm::mat4> cameraMatrices) {
  int count =0;
  for(auto c:cameraMatrices){
    std::cout<<"Cam "<<count <<std::endl;
    subdivide(p, c);
    p.saveFormat("Post.off");
    count++;
  }
}
//  void PolySubdivider::subdivide(Mesh &p, std::vector<glm::mat4> cameraMatrices) {
//
//  for (int curIt = 0; curIt < numIt_; ++curIt) {
//
//    std::vector<Halfedge_handle> eiv;
//    std::vector<Facet_handle> fiv;
//
//    for (Halfedge_iterator heIt = p.p.halfedges_begin(); heIt != p.p.halfedges_end(); heIt++) {
//
//      bool toCheck = false;
//
//      if (!heIt->is_border()) {
//        if (std::find(fiv.begin(), fiv.end(), heIt->facet()) == fiv.end()) {
//          if (!heIt->opposite()->is_border()) {
//            if (std::find(fiv.begin(), fiv.end(), heIt->opposite()->facet()) == fiv.end()) {
//              toCheck = true;
//            }
//          } else {
//            toCheck = true;
//          }
//        }
//      }
//
//      if (toCheck) {
//
//        Vertex_handle vd1 = heIt->vertex();
//        Vertex_handle vd2 = heIt->next()->vertex();
//        Vertex_handle vd3 = heIt->next()->next()->vertex();
//
//        bool count = 0;
//        std::vector<glm::mat4>::iterator itCam = cameraMatrices.begin();
//        while (count < 2 && itCam != cameraMatrices.end()) {
//
//          glm::vec2 pt2D_1 = utilities::projectPoint(*itCam, glm::vec3(vd1->point().x(), vd1->point().y(), vd1->point().z()));
//          glm::vec2 pt2D_2 = utilities::projectPoint(*itCam, glm::vec3(vd2->point().x(), vd2->point().y(), vd2->point().z()));
//          glm::vec2 pt2D_3 = utilities::projectPoint(*itCam, glm::vec3(vd3->point().x(), vd3->point().y(), vd3->point().z()));
//
//          float d1 = glm::length(pt2D_1 - pt2D_3);
//          float d2 = glm::length(pt2D_1 - pt2D_2);
//          float d3 = glm::length(pt2D_3 - pt2D_2);
//
//          float area = 0.5 * glm::abs(orientPoint(pt2D_1, pt2D_2, pt2D_3));
//
//          if (area > areaMax_ && d1 > d2 && d1 > d3) {
//            eiv.push_back(heIt);
//            fiv.push_back(heIt->facet());
//            if (!heIt->opposite()->is_border()) {
//              fiv.push_back(heIt->opposite()->facet());
//            }
//            count++;
//          }
//          itCam++;
//
//        }
//      }
//    }
//
//     std::cout << "Split border...it num." << curIt << std::endl;
//    for (auto he : eiv) {
//
//      Vertex_handle vd1 = he->vertex();
//      Vertex_handle vd2 = he->prev()->vertex();
//
//      Halfedge_handle heCur = p.p.split_edge(he);
//
//      heCur->vertex()->point() = Point((vd1->point().x() + vd2->point().x()) / 2, (vd1->point().y() + vd2->point().y()) / 2,
//          (vd1->point().z() + vd2->point().z()) / 2);
//
//      p.p.split_facet(heCur, he->next());
//
//      if (!he->opposite()->is_border()) {
//        p.p.split_facet(he->opposite(), heCur->opposite()->next());
//      }
//    }
//    //std::cout << "done." << std::endl;
//
//  }
//
//  //std::cout << "Remeshing done." << std::endl;
//}
