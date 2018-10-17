#include <SurfaceEvolver.h>

//#include <utilities.hpp>
#include <sstream>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <output_transforMesh.hpp>

#include <Logger.h>
// #include <CGAL/Subdivision_method_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Iterator_range.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>

SurfaceEvolver::SurfaceEvolver() {
}

SurfaceEvolver::SurfaceEvolver(PhotometricRefinementConfiguration config, std::shared_ptr<SfMData> sfmdata, bool initEvol) {
  initSurfaceEvolver(config, sfmdata, initEvol);
}

void SurfaceEvolver::initSurfaceEvolver(PhotometricRefinementConfiguration config, std::shared_ptr<SfMData> sfmdata, bool initEvol) {
  config_ = config;
  sfmData_ = sfmdata;
  mesh_.loadFormat(config_.pathInitMesh_.c_str(), false);
  std::size_t i = 0;
  for (Vertex_iterator it = mesh_.p.vertices_begin(); it != mesh_.p.vertices_end(); ++it) {
    it->id = i;
    i++;
  }

  initVisibility();
  imageHeight_ = sfmData_->camerasList_[0].imageHeight;
  imageWidth_ = sfmData_->camerasList_[0].imageWidth;
  loadImages();
  if (initEvol) {
    initEvolver();
  }

  pathCurMesh_ = std::string("tmpneeeeenovATTENZIONE.off");
}

SurfaceEvolver::~SurfaceEvolver() {
}

void SurfaceEvolver::initEvolver() {
  init();
  photometricGradient_ = new photometricGradient::PhotometricGradient(imageWidth_, imageHeight_, window_);
  subdivider_ = new PolySubdividerOpenGl(imageWidth_, imageHeight_, window_, config_.ensureareamax_, config_.ensureareait_);
  photometricGradient_->setWindowNcc(config_.window_NCC_);
}

void SurfaceEvolver::beginEvolver() {
  createVertexArrayBuffer();
  createImageVerticesBuffer();
  photometricGradient_->setEboSimGrad(eboSimGrad_);
  photometricGradient_->setVboSimGrad(vboSimGrad_);
}

void SurfaceEvolver::restartWithNewMesh(const Mesh& mesh) {
  mesh_ = mesh;
  resetMeshInfo();
}

void SurfaceEvolver::resetMeshInfo() {
  mesh_.updateMeshData(false, false);
  std::size_t i = 0;
  for (Vertex_iterator it = mesh_.p.vertices_begin(); it != mesh_.p.vertices_end(); ++it) {
    it->id = i++;
  }
  resetVertexArrayBuffer();
}

void SurfaceEvolver::refine() {
  std::vector<int> frames;
//  for (int i = 0; i < 11; ++i){
  for (int i = 0; i < sfmData_->camerasList_.size(); ++i) {
    frames.push_back(i);
  }
  refine(frames);
}

void SurfaceEvolver::refine(std::vector<int> frames) {

  computeCameraPairs(frames, curPairwiseCam_);
  mesh_.smooth(config_.lambdaSmooth_, 0);
  whichCamsAmIUsing(curPairwiseCam_, camerasOptimized);

  for (int curLevelOfDetail = 4; curLevelOfDetail > 0; --curLevelOfDetail) {
    remesh(curLevelOfDetail);

    // removeUnusedMesh(camsCur, false);

    for (Facet_iterator it = mesh_.p.facets_begin(); it != mesh_.p.facets_end(); it++) {
      it->setIntersectionStatus(true);
    }

    for (int curGradIter = 0; curGradIter < config_.numIt_; ++curGradIter) {

      log.startEvent();
      std::cout << "++++Iteration num. " << (4 - curLevelOfDetail) * config_.numIt_ + curGradIter << " num vert " << mesh_.p.size_of_vertices() << std::endl;
      gradientVectorsField_.assign(mesh_.p.size_of_vertices(), glm::vec3(0, 0, 0));
      numGradientContribField_.assign(mesh_.p.size_of_vertices(), 0);

      log.startEvent();
      //one gradient step
      std::cout << "comparison ";
      for (auto p : curPairwiseCam_) {

        log.startEvent();
//        log.startEvent();
        removeInvisible(sfmData_->camerasList_[p.first], sfmData_->camerasList_[p.second]);
        //log.endEventAndPrint("removeInvisible ", false);

        //log.startEvent();
        resetVertexArrayBuffer();
        //log.endEventAndPrint("  resetVertexArrayBuffer ", true);
        std::vector<glm::vec3> feedbackTr = photometricGradient_->twoImageGradient(images_[p.first], images_[p.second], sfmData_->camerasList_[p.first],
            sfmData_->camerasList_[p.second], numActiveVertices_, curLevelOfDetail);

        updateGradient(feedbackTr, config_.lambdaPhoto_);

        std::cout << " " << p.first << " & " << p.second << " ";
        log.endEventAndPrint("  ", true);
        std::vector<glm::vec3>().swap(feedbackTr);
      }
      log.endEventAndPrint("all two images", true);

      //saveVectorField(gradientVectorsField_, numGradientContribField_);
      log.startEvent();
      evolveMesh(gradientVectorsField_, numGradientContribField_);
      log.endEventAndPrint("evolve", true);

      log.startEvent();
      mesh_.smooth(config_.lambdaSmooth_, 0, true);
      mesh_.smooth(config_.lambdaSmooth_, 0, true);
      log.endEventAndPrint("addLaplacianUmbrellaOperator", true);

      resetMeshInfo();
      log.endEventAndPrint("Iteration ended", true);

      if (curGradIter % 1 == 0) {
        std::stringstream s;
        s << config_.pathOutputDir_ << "CurMesh" << (4 - curLevelOfDetail) * config_.numIt_ + curGradIter << ".off";
        mesh_.saveFormat(s.str().c_str());
      }
    }
  }

  std::stringstream s;
  s << config_.pathOutputDir_ << "/Mesh_Final.off";
  mesh_.saveFormat(s.str().c_str());

}

void SurfaceEvolver::computeCameraPairs(std::vector<int> frames, std::vector<std::pair<int, int> > &pairwiseCam) {

  utilities::Logger logg_;
  logg_.printOn("Pairwise Cameras: ");
  for (int it = 1; it < frames.size(); ++it) {
    int f1 = frames[it];
    int f2 = frames[it - 1];
    pairwiseCam.push_back(std::pair<int, int>(f2, f1));
    pairwiseCam.push_back(std::pair<int, int>(f1, f2));
    std::stringstream sss;
    sss << " " << f2 << " & " << f1 << " ";
    logg_.printOn(sss.str());
  }

  for (int it = 2; it < frames.size(); ++it) {
    int f1 = frames[it];
    int f2 = frames[it - 2];
    pairwiseCam.push_back(std::pair<int, int>(f2, f1));
    pairwiseCam.push_back(std::pair<int, int>(f1, f2));
    std::stringstream sss;
    sss << " " << f2 << " & " << f1 << " ";
    logg_.printOn(sss.str());
  }

  for (int it = 3; it < frames.size(); ++it) {
    int f1 = frames[it];
    int f2 = frames[it - 3];
    pairwiseCam.push_back(std::pair<int, int>(f2, f1));
    pairwiseCam.push_back(std::pair<int, int>(f1, f2));
    std::stringstream sss;
    sss << " " << f2 << " & " << f1 << " ";
    logg_.printOn(sss.str());
  }

  for (int it = 4; it < frames.size(); ++it) {
    int f1 = frames[it];
    int f2 = frames[it - 4];
    pairwiseCam.push_back(std::pair<int, int>(f2, f1));
    pairwiseCam.push_back(std::pair<int, int>(f1, f2));
    std::stringstream sss;
    sss << " " << f2 << " & " << f1 << " ";
    logg_.printOn(sss.str());
  }

}

void SurfaceEvolver::whichCamsAmIUsing(std::vector<std::pair<int, int> > &pairwiseCam, std::vector<photometricGradient::CameraType> & cams) {
  std::vector<int> camerasOptimizedId;
  for (auto p : curPairwiseCam_) {
    camerasOptimizedId.push_back(p.first);
    camerasOptimizedId.push_back(p.second);
  }
  std::sort(camerasOptimizedId.begin(), camerasOptimizedId.end());

  std::vector<int>::iterator newEnd = std::unique(camerasOptimizedId.begin(), camerasOptimizedId.end());
  for (auto it = camerasOptimizedId.begin(); it != newEnd; it++) {
    std::cout << " " << *it << std::endl;
    cams.push_back(sfmData_->camerasList_[*it]);
  }

}

void SurfaceEvolver::evolveMesh(const std::vector<glm::vec3>& displacements, const std::vector<int> count) {

  std::vector<float> values;
  float sign = -1.0;

  int curV = 0;
  for (Vertex_iterator v = mesh_.p.vertices_begin(); v != mesh_.p.vertices_end(); v++) {
    if (count[curV] > 0 && !isnan(displacements[curV].x) && !isnan(displacements[curV].y) && !isnan(displacements[curV].z)) {
      //glm::vec3 normVal = sign * displacements[curV] / static_cast<float>(count[curV]);
      glm::vec3 normVal = sign * displacements[curV];
      values.push_back(glm::length(normVal));
    }
    curV++;
  }

  float median = 0.0;
  std::sort(values.begin(), values.end());
  if (values.size() > 0) {
    median = values[values.size() / 2];
  }

  curV = 0;

  for (Vertex_iterator v = mesh_.p.vertices_begin(); v != mesh_.p.vertices_end(); v++) {
    if (count[curV] > 0 && !isnan(displacements[curV].x) && !isnan(displacements[curV].y) && !isnan(displacements[curV].z)) {
//      glm::vec3 normVal = sign * displacements[curV] / static_cast<float>(count[curV]);
      glm::vec3 normVal = sign * displacements[curV];

      if (glm::length(normVal) < 200 * median) {
//        if (1 / 50 * median < glm::length(normVal) && glm::length(normVal) < 50 * median) {
        Vector the_shift = Vector(normVal.x, normVal.y, normVal.z);
        v->move(the_shift);
      }
    }
    curV++;
  }

  mesh_.updateMeshData(false, false);

}

void SurfaceEvolver::updateGradient(const std::vector<glm::vec3>& curDisplacements, float alpha) {
  if (alpha < 0) {
    alpha = config_.lambdaPhoto_;
  }
  utilities::Logger logger;
  logger.disable();
  logger.startEvent();
  int curV = 0;

  std::vector<float> maxElem;
  for (auto g : curDisplacements) {
    if (!isnan(g.x) && !isnan(g.y) && !isnan(g.z) && !isinf(g.x) && !isinf(g.y) && !isinf(g.z) && glm::length(g) > 0.0) {
      maxElem.push_back(glm::length(g));
    }
  }
  std::sort(maxElem.begin(), maxElem.end());

  float median;
  if (maxElem.size() > 0) {
    median = maxElem[maxElem.size() / 2];
  } else {
    std::cout << "maxElem.size()<0" << std::endl;
  }

  std::vector<int> numGradientContribField(numGradientContribField_.size(), 0);
  for (auto v : curDisplacements) {
    int curId = curActiveVertices_[curV]->id;

    if (!isnan(v.x) && !isnan(v.y) && !isnan(v.z) && !isinf(v.x) && !isinf(v.y) && !isinf(v.z) && glm::length(v) < 50 * median) {
      gradientVectorsField_[curId] += -alpha * v;
      if ((v.x != 0 || v.y != 0 || v.z != 0)) {
        numGradientContribField_[curId]++;
      }
    }
    curV++;
  }
  logger.endEventAndPrint("Mesh Update ", false);
}

void SurfaceEvolver::loadImages() {
  images_.clear();
  for (int curC = 0; curC < sfmData_->camerasList_.size(); ++curC) {

    cv::Mat im = cv::imread(sfmData_->camerasList_[curC].pathImage);
    std::cout << "path " << sfmData_->camerasList_[curC].pathImage << std::endl;

    images_.push_back(im);
//      cv::imshow("im",im);
//      cv::waitKey();
  }
}

void SurfaceEvolver::resetVertexArrayBuffer() {
  glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObj_);
  photometricGradient_->setVertexBufferObj(vertexBufferObj_);
  subdivider_->setVertexBufferObj(vertexBufferObj_);

  std::vector<glm::vec4> verticesUnwrapped;

  curActiveVertices_.clear();
  std::vector<float> ver;

  for (Facet_iterator itFac = mesh_.p.facets_begin(); itFac != mesh_.p.facets_end(); itFac++) {

    Halfedge_handle h0, h1, h2;
    h0 = itFac->halfedge();
    h1 = h0->next();
    h2 = h1->next();

    Vertex_handle v0, v1, v2;
    v0 = h0->vertex();
    v1 = h1->vertex();
    v2 = h2->vertex();

    if (v0->getVisibility() || v1->getVisibility() || v2->getVisibility()) {
      verticesUnwrapped.push_back(glm::vec4(v0->point().x(), v0->point().y(), v0->point().z(), 1.0));
      verticesUnwrapped.push_back(glm::vec4(v1->point().x(), v1->point().y(), v1->point().z(), 1.0));
      verticesUnwrapped.push_back(glm::vec4(v2->point().x(), v2->point().y(), v2->point().z(), 1.0));
      curActiveVertices_.push_back(v0);
      curActiveVertices_.push_back(v1);
      curActiveVertices_.push_back(v2);
    }

  }
  numActiveVertices_ = verticesUnwrapped.size();
  photometricGradient_->setNumActiveVertices(numActiveVertices_);

  glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObj_);
  glBufferData(GL_ARRAY_BUFFER, verticesUnwrapped.size() * sizeof(glm::vec4), &verticesUnwrapped[0], GL_DYNAMIC_DRAW);
  verticesUnwrapped.clear();
}

void SurfaceEvolver::createVertexArrayBuffer() {
  glGenBuffers(1, &vertexBufferObj_);

  photometricGradient_->setVertexBufferObj(vertexBufferObj_);
  resetVertexArrayBuffer();
}

void SurfaceEvolver::initVisibility() {
  for (Vertex_iterator h = mesh_.p.vertices_begin(); h != mesh_.p.vertices_end(); ++h) {
    h->setVisibility(true);
  }
}

void SurfaceEvolver::removeUnusedMesh(std::vector<photometricGradient::CameraType> cams, bool checkVis) {
  utilities::Logger log;
  log.startEvent();
  for (Vertex_iterator h = mesh_.p.vertices_begin(); h != mesh_.p.vertices_end(); ++h) {
    h->setVisibility(false);
  }
  for (Facet_iterator it = mesh_.p.facets_begin(); it != mesh_.p.facets_end(); it++) {
    it->setIntersectionStatus(false);
    //it->setIntersectionStatus(true);
  }

  for (Vertex_iterator h = mesh_.p.vertices_begin(); h != mesh_.p.vertices_end(); ++h) {

    bool tobeset1 = false;
    bool tobeset2 = false;
    for (auto c : cams) {
      glm::mat4 mvp1 = c.mvp;
      glm::vec4 p1 = glm::vec4(h->point().x(), h->point().y(), h->point().z(), 1.0f);
      glm::vec3 p1b = glm::vec3(h->point().x(), h->point().y(), h->point().z());
      glm::vec4 pRes1 = mvp1 * p1;
      glm::vec3 pRes2 = c.rotation * p1b + c.translation;
      glm::vec3 pRes3 = p1b * c.rotation + c.translation;

      float dist1 = glm::length(c.center - glm::vec3(p1.x, p1.y, p1.z));

      if (-1.0f < pRes1.x / pRes1.w && pRes1.x / pRes1.w < 1.0f //
      && -1.0f < pRes1.y / pRes1.w && pRes1.y / pRes1.w < 1.0f //
      && -1.0f < pRes1.z / pRes1.w && pRes1.z / pRes1.w < 1.0f) {

        tobeset1 = true;
      }

//      std::cout<<"+++++++++++++++"<<std::endl;
//      utilities::printMatrix("pRes1",pRes1);
//      utilities::printMatrix("pRes2",pRes2);
//      utilities::printMatrix("pRes3",pRes3);
//      utilities::printMatrix("c.rotation * p1b",c.rotation * p1b);
//      utilities::printMatrix("p1b*c.rotation",p1b*c.rotation);
//      utilities::printMatrix("c.translation",c.translation);
//      utilities::printMatrix("c.center",c.center);
//      utilities::printMatrix("p1",p1);
//      std::cout<<dist1<<std::endl;
//      std::cout<<"+-------------+"<<std::endl;

      if (pRes3.z < config_.maxDistanceCamFeatureRef && dist1 < 2 * config_.maxDistanceCamFeatureRef) {
        tobeset2 = true;
      }
    }

//    if (tobeset2 ) {
    if (tobeset1 && tobeset2) {
      h->setVisibility(true);
    }

  }
//  exit(0);
//
//  int count = 0;
//  for (auto c : cams) {
//    if (count % 49 == 0) {
//      std::vector<Point> pts;
//      for (Vertex_iterator h = mesh_.p.vertices_begin(); h != mesh_.p.vertices_end(); ++h) {
//
//        glm::mat4 mvp1 = c.mvp;
//        glm::vec4 p1 = glm::vec4(h->point().x(), h->point().y(), h->point().z(), 1.0f);
//        glm::vec3 p1b = glm::vec3(h->point().x(), h->point().y(), h->point().z());
//        glm::vec4 pRes1 = mvp1 * p1;
//        glm::vec3 pRes3 = p1b * c.rotation + c.translation;
//        float dist1 = glm::length(c.center - glm::vec3(p1.x, p1.y, p1.z));
//
//        if (pRes3.z < config_.maxDistanceCamFeatureRef && dist1 < 2*config_.maxDistanceCamFeatureRef) {
//
//          pts.push_back(Point(pRes3.x, pRes3.y, pRes3.z));
//        }
//
//      }
//      std::stringstream sfi;
//      sfi << "ptsss" << count;
//      output_transforMesh::printPointVec(pts, sfi.str());
//    }
//    count++;
//  }

  for (Facet_iterator itFac = mesh_.p.facets_begin(); itFac != mesh_.p.facets_end(); itFac++) {

    Halfedge_handle h0, h1, h2;
    h0 = itFac->halfedge();
    h1 = h0->next();
    h2 = h1->next();

    Vertex_handle v0, v1, v2;
    v0 = h0->vertex();
    v1 = h1->vertex();
    v2 = h2->vertex();

    if ((v0->getVisibility() || v1->getVisibility() || v2->getVisibility())) {  //|| (!v0->flag[3] || !v1->flag[3] || !v2->flag[3])) {
      itFac->setIntersectionStatus(true);
    }
  }

}

void SurfaceEvolver::removeInvisible(photometricGradient::CameraType cam1, photometricGradient::CameraType cam2) {
  utilities::Logger log;
  log.startEvent();
  for (Vertex_iterator h = mesh_.p.vertices_begin(); h != mesh_.p.vertices_end(); ++h) {
    h->setVisibility(false);
  }

  glm::mat4 mvp1 = cam1.mvp;
  glm::mat4 mvp2 = cam2.mvp;

  for (Vertex_iterator h = mesh_.p.vertices_begin(); h != mesh_.p.vertices_end(); ++h) {

    glm::vec4 p1 = glm::vec4(h->point().x(), h->point().y(), h->point().z(), 1.0f);
    glm::vec3 p13 = glm::vec3(h->point().x(), h->point().y(), h->point().z());
    glm::vec4 pRes1 = mvp1 * p1;

    glm::vec4 p2 = glm::vec4(h->point().x(), h->point().y(), h->point().z(), 1.0f);
    glm::vec3 p23 = glm::vec3(h->point().x(), h->point().y(), h->point().z());
    glm::vec4 pRes2 = mvp2 * p2;
    glm::vec3 pResA = p13 * cam1.rotation + cam1.translation;
    glm::vec3 pResB = p23 * cam2.rotation + cam2.translation;

    float dist1 = glm::length(cam1.center - glm::vec3(p1.x, p1.y, p1.z));
    float dist2 = glm::length(cam2.center - glm::vec3(p1.x, p1.y, p1.z));
    //dist2=dist1=0.0;

    if (((-1.0f < pRes1.x / pRes1.w && pRes1.x / pRes1.w < 1.0f //
    && -1.0f < pRes1.y / pRes1.w && pRes1.y / pRes1.w < 1.0f //
    && -1.0f < pRes1.z / pRes1.w && pRes1.z / pRes1.w < 1.0f) //
        &&//
        (-1.0f < pRes2.x / pRes2.w && pRes2.x / pRes2.w < 1.0f //
        && -1.0f < pRes2.y / pRes2.w && pRes2.y / pRes2.w < 1.0f //
        && -1.0f < pRes2.z / pRes2.w && pRes2.z / pRes2.w < 1.0f) && pResA.z < config_.maxDistanceCamFeatureRef && pResB.z < config_.maxDistanceCamFeatureRef
        && dist1 < 2 * config_.maxDistanceCamFeatureRef && dist2 < 2 * config_.maxDistanceCamFeatureRef)) {

//        if ((dist1< 2*config_.maxDistanceCamFeatureRef && dist2< 2*config_.maxDistanceCamFeatureRef && pResA.z < config_.maxDistanceCamFeatureRef && pResB.z < config_.maxDistanceCamFeatureRef)) {
      h->setVisibility(true);
    }
  }

}

void SurfaceEvolver::remesh(int lod) {
  mesh_.saveFormat("Prima.off");
  for (int var = 0; var < config_.ensureareait_; ++var) {

    subdivider_->setCurActiveVertices(curActiveVertices_);
    subdivider_->setAreaMax(lod * lod * config_.ensureareamax_);
    subdivider_->subdivide(mesh_, camerasOptimized, numActiveVertices_);
    resetVertexArrayBuffer();
  }
  resetMeshInfo();
  mesh_.saveFormat("Dopo.off");
  //exit(0);
}

void SurfaceEvolver::removeInvisible(photometricGradient::CameraType cam1) {
  utilities::Logger log;
  log.startEvent();
  for (Vertex_iterator h = mesh_.p.vertices_begin(); h != mesh_.p.vertices_end(); ++h) {
    h->setVisibility(false);
  }

  glm::mat4 mvp1 = cam1.mvp;

  for (Vertex_iterator h = mesh_.p.vertices_begin(); h != mesh_.p.vertices_end(); ++h) {

    glm::vec4 p1 = glm::vec4(h->point().x(), h->point().y(), h->point().z(), 1.0f);
    glm::vec3 p13 = glm::vec3(h->point().x(), h->point().y(), h->point().z());
    glm::vec4 pRes1 = mvp1 * p1;
    glm::vec3 pResA = p13 * cam1.rotation + cam1.translation;

    float dist1 = glm::length(cam1.center - glm::vec3(p1.x, p1.y, p1.z));
    if (((-1.0f < pRes1.x / pRes1.w && pRes1.x / pRes1.w < 1.0f //
    && -1.0f < pRes1.y / pRes1.w && pRes1.y / pRes1.w < 1.0f //
    && -1.0f < pRes1.z / pRes1.w && pRes1.z / pRes1.w < 1.0f) //
    && pResA.z < config_.maxDistanceCamFeatureRef && dist1 < 2 * config_.maxDistanceCamFeatureRef)) {
      h->setVisibility(true);
    }
  }

}

void SurfaceEvolver::findTwoNearestCam(int idx1, std::vector<int> & myints) {
  myints.assign(2, 0);
  float dist1 = 1000000;
  float dist2 = 1000000;

  for (int curId2 = 0; curId2 < sfmData_->camerasList_.size(); ++curId2) {
    if (curId2 != idx1) {
      glm::vec3 curCenter = sfmData_->camerasList_[idx1].center;
      glm::vec3 cent2 = sfmData_->camerasList_[curId2].center;
      float dist = glm::length(curCenter - cent2);
      if (dist < dist1) {
        float oldDist1 = dist1;
        float oldidDist1 = myints[0];

        dist1 = dist;
        myints[0] = curId2;

        if (oldDist1 < dist2) {
          dist2 = oldDist1;
          myints[1] = oldidDist1;
        }
      } else if (dist < dist2) {
        dist2 = dist;
        myints[1] = curId2;
      }
    }
  }
}

void SurfaceEvolver::saveVectorField(std::vector<glm::vec3>& displacements, const std::vector<int> count) {

  std::vector<float> values;
  int curV = 0;
  std::ofstream outfileB;
  outfileB.open("saveVectorField.ply");

  outfileB << "ply" << std::endl << "format ascii 1.0" << std::endl;
  outfileB << "element vertex " << mesh_.p.size_of_vertices() * 2 << std::endl;
  outfileB << "property float x " << std::endl << "property float y " << std::endl << "property float z " << std::endl;
  outfileB << "element edge " << mesh_.p.size_of_vertices() << std::endl;
  outfileB << "property int vertex1 " << std::endl << "property int vertex2" << std::endl;
  outfileB << "end_header" << std::endl;

  for (Vertex_iterator v = mesh_.p.vertices_begin(); v != mesh_.p.vertices_end(); v++) {

    glm::vec3 normVal;
    outfileB << static_cast<float>(v->point().x()) << " " << static_cast<float>(v->point().y()) << " " << static_cast<float>(v->point().z()) << " "
        << std::endl;
    if (count[curV] > 0 && !isnan(displacements[curV].x) && !isnan(displacements[curV].y) && !isnan(displacements[curV].z)) {
//      glm::vec3 normVal = sign * displacements[curV] / static_cast<float>(count[curV]);
      GLfloat scale = 1.0;
      normVal = -glm::vec3(scale * displacements[curV].x, scale * displacements[curV].y, scale * displacements[curV].z);
    }

    glm::vec3 p = glm::vec3(static_cast<float>(v->point().x()), static_cast<float>(v->point().y()), static_cast<float>(v->point().z())) + normVal;

    outfileB << static_cast<float>(p.x) << " " << static_cast<float>(p.y) << " " << static_cast<float>(p.z) << " " << std::endl;
    curV++;
  }

  for (int count = 0; count < mesh_.p.size_of_vertices() * 2; count += 2) {
    outfileB << count << " " << count + 1 << std::endl;
  }
  outfileB.close();

}

