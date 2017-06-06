/*
 * PolySubdividerOpenGl.cpp
 *
 *  Created on: Jun 5, 2017
 *      Author: andrea
 */

#include <PolySubdividerOpenGl.h>
#include <DepthMapProgram.h>
#include <CheckReprojAreaProgram.h>

PolySubdividerOpenGl::PolySubdividerOpenGl(int imageWidth, int imageHeight, GLFWwindow* window) {
  window_ = window;
  imageWidth_ = imageWidth;
  imageHeight_ = imageHeight;
  numActiveVertices_ = 0;
  vertexBufferObj_ = -1;
  maxArea_ = 16;
  numIt_ = 3;
  initShaders();
}

PolySubdividerOpenGl::PolySubdividerOpenGl(int imageWidth, int imageHeight, GLFWwindow* window, int maxArea, int numit) {
  window_ = window;
  imageWidth_ = imageWidth;
  imageHeight_ = imageHeight;
  numActiveVertices_ = 0;
  vertexBufferObj_ = -1;
  maxArea_ = maxArea;
  numIt_ = numit;

  initShaders();

}

PolySubdividerOpenGl::~PolySubdividerOpenGl() {
}

void PolySubdividerOpenGl::initShaders() {
  //************************depth********************************
  std::cout << "DepthMapProgram init...";
  depthMapProgram_ = new photometricGradient::DepthMapProgram(imageWidth_, imageHeight_);
  depthMapProgram_->initializeProgram();
  depthMapProgram_->setUseElementsIndices(false);
  static_cast<photometricGradient::DepthMapProgram *>(depthMapProgram_)->initializeFramebufAndTex(framebufferDepth_, depthTexture_);
  std::cout << "DONE" << std::endl;

  std::cout << "GradientCollectorProgram init...";
  reprojectionProgram_ = new CheckReprojAreaProgram(imageWidth_, imageHeight_);
  reprojectionProgram_->initializeProgram();
  static_cast<CheckReprojAreaProgram *>(reprojectionProgram_)->createTransformFeedback(0);
  reprojectionProgram_->setUseElementsIndices(false);
  std::cout << "DONE" << std::endl;
}

void PolySubdividerOpenGl::subdivide(Mesh& p, std::vector<photometricGradient::CameraType> cameraMatrices, int numActiveVertices) {
  numActiveVertices_ = numActiveVertices;
  int count = 0;

  std::cout << "PolySubdividerOpenGl::subdivide : " << numActiveVertices_ << " max area " << maxArea_ << std::endl;
  for (auto c : cameraMatrices) {
    depthMapProgram_->setArrayBufferObj(vertexBufferObj_, numActiveVertices_);
    static_cast<photometricGradient::DepthMapProgram *>(depthMapProgram_)->computeDepthMap(framebufferDepth_, c.mvp);
    glFinish();

    std::cout << "Cam " << count << "  " << std::flush;
    count++;

    //*******************GRAD COLL***********************************
    reprojectionProgram_->setArrayBufferObj(vertexBufferObj_, numActiveVertices_);
    static_cast<CheckReprojAreaProgram *>(reprojectionProgram_)->resetTransformFeedback(numActiveVertices_);
    static_cast<CheckReprojAreaProgram *>(reprojectionProgram_)->setMvp(c.mvp);
    static_cast<CheckReprojAreaProgram *>(reprojectionProgram_)->setArea(maxArea_);
    static_cast<CheckReprojAreaProgram *>(reprojectionProgram_)->setCameraMatrix(c.cameraMatrix);
    static_cast<CheckReprojAreaProgram *>(reprojectionProgram_)->setFeedbackLength(numActiveVertices_);
    static_cast<CheckReprojAreaProgram *>(reprojectionProgram_)->setDepthTexture(depthTexture_);
    static_cast<CheckReprojAreaProgram *>(reprojectionProgram_)->compute(false);
    glFinish();
    //*/
    SwapBuffers();
    // sleep(5.0);

    for (auto v : curActiveVertices_) {
      v->flag[3] = false;
    }
    int curV = 0;

//    std::ofstream f("ver.off");
//    std::vector<Vertex_handle> vers;
    std::vector<float> feedbackTr = static_cast<CheckReprojAreaProgram *>(reprojectionProgram_)->getFeedbackTr();
    for (auto v : feedbackTr) {
//      if (v > 0.5)
//        vers.push_back(curActiveVertices_[curV]);
     // std::cout<<v<<std::endl;
      curActiveVertices_[curV]->flag[3] = (curActiveVertices_[curV]->flag[3] || v > 0.5);
      curV++;
    }
//    std::cout<<"vers "<<vers.size()<<std::endl;
//    f << "OFF" << std::endl << vers.size() << " 0 0" << std::endl;
//    for (auto v : vers) {
//      f << v->point().x() << " " << v->point().y() << " " << v->point().z() << std::endl;
//    }
//    f.close();
  }

  std::vector<Halfedge_handle> eiv;
  std::vector<Facet_handle> fiv;
  for (Halfedge_iterator heIt = p.p.halfedges_begin(); heIt != p.p.halfedges_end(); heIt++) {
    if (!heIt->is_border()) {
      Vertex_handle v1 = heIt->vertex();
      Vertex_handle v2 = heIt->opposite()->vertex();
      if (v1->flag[3] && v2->flag[3]) {
        eiv.push_back(heIt);
        fiv.push_back(heIt->facet());
        if (!heIt->opposite()->is_border()) {
          fiv.push_back(heIt->opposite()->facet());
        }
      }
    }
  }

  std::cout << "Split border...." << std::flush;
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

  std::cout << "Subdivision Done" << std::endl;
}
