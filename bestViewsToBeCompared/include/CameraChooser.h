#ifndef BestView_CAMERACHOOSER_H_
#define BestView_CAMERACHOOSER_H_

#include <ShaderManager.h>
#include <ShaderProgram.h>
#include <opencv2/core/core.hpp>
#include <glm.hpp>
#include <types.hpp>
#include <OpenGLProgram.h>
#include <Mesh.h>
#include <typesRefinement.hpp>

class CameraChooser : public OpenGLProgram {
public:
  CameraChooser(int imageWidth, int imageHeight);
  CameraChooser(int imageWidth, int imageHeight, GLuint vertexBufferObj);
  CameraChooser();
  virtual ~CameraChooser();
  void getExaustive(const std::vector<photometricGradient::CameraType>& cameraList, std::vector<std::vector<int>>& out, std::vector<int>& consideredIdx);
  void getKNear(const std::vector<photometricGradient::CameraType>& cameraList, int curIdx, int num, std::vector<int> &out, std::vector<int> consideredIdx = std::vector<int>());
  void getKNear(const std::vector<photometricGradient::CameraType>& cameraList, int maxCam, std::vector<std::vector<int>>& out, std::vector<int> consideredIdx = std::vector<int>());
  void getKNearWithoutRep(const std::vector<photometricGradient::CameraType>& cameraList, int maxCam, std::vector<std::vector<int>>& out, std::vector<int> consideredIdx =
      std::vector<int>());
  void getKNearWithCoverage(const std::vector<photometricGradient::CameraType>& cameraList, int maxCam, std::vector<std::vector<int>>& out, std::vector<int> consideredIdx =
       std::vector<int>());
  void getNearByThreshold(const std::vector<photometricGradient::CameraType>& cameraList, int curIdx, float th, std::vector<int> &out);
  void getBestBySfmOutput(const std::shared_ptr<SfMData>& sfm, int curIdx, int minMatchesNum, int maxCam, std::vector<int> &out);
  void getBestBySfmOutput(const std::shared_ptr<SfMData>& sfm, int minMatchesNum, int maxCam, std::vector<std::vector<int>>& out);
  void setVertexBufferObj(GLuint vertexBufferObj) {
    vertexBufferObj_ = vertexBufferObj;
  }
  void setNumActiveVertices(int numActiveVertices) {
    numActiveVertices_ = numActiveVertices;
  }

  void swap() {
    SwapBuffers();
  }

  void setMesh(std::shared_ptr<Mesh> mesh, bool checkVisibility = false) {
    mesh_ = mesh;
    resetMeshInfo(checkVisibility);
  }

  float getOverlap() const {
    return overlap_;
  }

  void setMu1(float mu1) {
    mu_1 = mu1;
  }

  void setMu2(float mu2) {
    mu_2 = mu2;
  }

  void setMu3(float mu3) {
    mu_3 = mu3;
  }

private:
  void computeMutualMatches(const std::shared_ptr<SfMData>& sfm);
  void resetMeshInfo(bool checkVisibility);
  void initOpenGLStuff();
  void initShaders();
  float angleThreePoints(glm::vec3 p1, glm::vec3 p, glm::vec3 p2);

  void computeGoodness(const photometricGradient::CameraType &cam1, const photometricGradient::CameraType &cam2, float &goodness1);
  void computeCoverage(const photometricGradient::CameraType &cam1, const photometricGradient::CameraType &cam2, std::vector<bool> &coverage);
  void meanAndVarVec(std::vector<int> vis, float &mean, float&stddev);

//  std::vector<photometricGradient::CameraType> cameraList_;

  ShaderProgram *depthMapProgram_;
  ShaderProgram *angleProgram_;
  ShaderProgram *coverageProgram_;
  ShaderProgram *collectAngleProgram_;
  GLuint framebufferDepth_, framebufferDepth2_;
  GLuint depthTexture_, depthTexture2_, image2AngleTex_;
  GLuint vertexBufferObj_;
  int numActiveVertices_;
  std::shared_ptr<Mesh> mesh_;
  std::vector<std::vector<int>> mutualMatches;
  int ref, cur;
  float overlap_;
  std::vector<std::pair<int, float> > cams;
  float mu_1,mu_2,mu_3;
};

#endif /* PHOTOMETRIC_REFINEMENT_CAMERACHOOSER_H_ */
