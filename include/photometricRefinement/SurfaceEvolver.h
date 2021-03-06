#ifndef SURVACEEVOLVER_H_
#define SURVACEEVOLVER_H_

#include <iostream>

#include <types.hpp>
#include <typesRefinement.hpp>
#include <PhotometricGradient.h>
#include <OpenGLProgram.h>

#include <PolySubdividerOpenGl.h>

#include <Mesh.h>
#include <Logger.h>
#include <typesRefinement.hpp>

#include <opencv2/core/core.hpp>

#include <glm.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

//#include <MeshBuilder.h>

//typedef typename boost::graph_traits<Polyhedron>::face_descriptor face_descriptor;

class SurfaceEvolver : public OpenGLProgram{
public:
  SurfaceEvolver();
  
  SurfaceEvolver(PhotometricRefinementConfiguration config, std::shared_ptr<SfMData> sfmData,bool initEvolver = true);
  virtual ~SurfaceEvolver() = default;

  void initSurfaceEvolver(PhotometricRefinementConfiguration config, std::shared_ptr<SfMData> sfmData, bool initEvolver = true);

  void restartWithNewMesh(const Mesh& mesh);

  void initEvolver();
  void beginEvolver();
  void refineWithNearCams();
  void refineWithBest2SfM();
  void refine();

  void saveVectorField(std::vector<glm::vec3> &feedbackTr, const std::vector<int> count);

  const Mesh& getMesh() const {
    return mesh_;
  }

  void findTwoNearestCam(int idx1, std::vector<int> & myints) ;


  const Mesh& getCurMeshRefined() const {
    return mesh_;
  }
  
  void saveCurMeshRefined(const std::string& path) {
    mesh_.saveFormat(path.c_str());
  }

  const std::string& getPathCurMesh() const {
    return pathCurMesh_;
  }

  const std::vector<cv::Mat>& getImages() const {
    return images_;
  }

  int getNumActiveVertices() const {
    return numActiveVertices_;
  }

  void resetVertexArrayBuffer();
private:

  void evolveMesh(const std::vector<glm::vec3>& displacements, const std::vector<int> count);
  void updateGradient(const std::vector<glm::vec3>& curDisplacements, float alpha = -1.0);

  void removeInvisible(photometricGradient::CameraType cam1, photometricGradient::CameraType cam2);
  void removeInvisible(photometricGradient::CameraType cam1);
  void removeUnusedMesh(std::vector<photometricGradient::CameraType> cams, bool checkVis = false);
  void initVisibility();

  void loadImages();
  void resetMeshInfo();
  void computeCameraPairs(std::vector<int> frames,std::vector<std::pair<int, int> > &pairwiseCam);
  void whichCamsAmIUsing(std::vector<std::pair<int, int> > &pairwiseCam, std::vector<photometricGradient::CameraType>  & cams);

  void createVertexArrayBuffer();
  void remesh(int lod);


  std::vector<photometricGradient::CameraType> camerasOptimized;
  PhotometricRefinementConfiguration config_;
  photometricGradient::PhotometricGradient *photometricGradient_;

  std::vector<cv::Mat> images_;
  std::vector<glm::vec3> gradientVectorsField_;
  std::vector<int> numGradientContribField_;

  std::vector<std::pair<int, int> > curPairwiseCam_;

  std::vector<Vertex_handle> curActiveVertices_;

  GLuint gradientValuesTexture_,vertexBufferObj_;
  GLuint facetBufferObj_, normalBufferObj_, idBufferObj_;

  int numActiveVertices_;
  utilities::Logger log;

  Mesh mesh_;

  std::string pathCurMesh_;
  std::shared_ptr<SfMData> sfmData_;
  PolySubdividerOpenGl *subdivider_;
  std::vector<int> consideredIdx_;


};

#endif /* SURVACEEVOLVER_H_ */
