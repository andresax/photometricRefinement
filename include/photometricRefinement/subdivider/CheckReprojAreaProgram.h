/*
 * CheckReprojAreaProgram.h
 *
 *  Created on: Jun 5, 2017
 *      Author: andrea
 */

#ifndef SRC_SUBDIVIDER_CHECKREPROJAREAPROGRAM_H_
#define SRC_SUBDIVIDER_CHECKREPROJAREAPROGRAM_H_

#include <ShaderProgram.h>
#include <glm.hpp>
class CheckReprojAreaProgram : public ShaderProgram{
public:
  CheckReprojAreaProgram(int imageWidth, int imageHeight);
  virtual ~CheckReprojAreaProgram();

  void compute(bool show =false);
  void createTransformFeedback(int length);
  void resetTransformFeedback(int length);

  void setFeedbackLength(GLuint feedbackLength) {
    feedbackLength_ = feedbackLength;
  }

  const std::vector<float>& getFeedbackTr() const {
    return feedbackTr;
  }

  void setDepthTexture(GLuint depthTexture) {
    depthTexture_ = depthTexture;
  }


  void setMvp(const glm::mat4& mvp) {
    this->mvp = mvp;
  }

  void setArea(const float area) {
    this->area_ = area;
  }

  void setCameraMatrix(const glm::mat4& cameraMatrix) {
    cameraMatrix_ = cameraMatrix;
  }

private:
  void init();

  void createAttributes();
  void createUniforms();

  GLuint posAttrib_;
  GLuint feedbackBuffer_;
  GLuint feedbackLength_;
  std::vector<float> feedbackTr;
  GLuint depthTexture_,camCollID_;
  GLuint mvp1CollID_, shadowMapIdColl_,areaId_;

  GLint posAttribCollId_;

  float area_;
  glm::mat4 mvp;
  glm::mat4 cameraMatrix_;
};

#endif /* SRC_SUBDIVIDER_CHECKREPROJAREAPROGRAM_H_ */
