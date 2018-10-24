#ifndef SURVACEEVOLVER_H_
#define SURVACEEVOLVER_H_

#include <sstream>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include <glm/glm.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
// #include <CGAL/Subdivision_method_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Iterator_range.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <photometricGradient/Logger.h>
#include <photometricGradient/types.hpp>
#include <photometricGradient/PhotometricGradient.h>
#include <photometricGradient/OpenGLProgram.h>
#include <photometricGradient/Logger.h>
#include <TransforMesh/Mesh.h>
#include <bestViewsToBeCompared/CameraChooser.h>

#include <misc/typesRefinement.hpp>
#include <misc/typesRefinement.hpp>
#include <subdivider/PolySubdividerOpenGl.h> 
//#include <MeshBuilder.h>

//#include <utilities.hpp>

//#include <output_transforMesh.hpp>


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
