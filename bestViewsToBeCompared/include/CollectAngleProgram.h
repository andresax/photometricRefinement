/*
 * CollectAngleProgram.h
 *
 *  Created on: Nov 21, 2016
 *      Author: andrea
 */

#ifndef PHOTOMETRIC_REFINEMENT_COLLECTANGLEPROGRAM_H_
#define PHOTOMETRIC_REFINEMENT_COLLECTANGLEPROGRAM_H_

#include <ShaderProgram.h>

#include <glm.hpp>


class CollectAngleProgram : public ShaderProgram{
public:
  CollectAngleProgram(int imageWidth, int imageHeight);
  virtual ~CollectAngleProgram();
  void createTransformFeedback();
  void compute(bool renderFrameBuf = false);
  const GLfloat& getFeedbackTr() const {
    return feedbackTr;
  }
  void setAngleTex(GLuint angleTex) {
      angleTex_ = angleTex;
    }
private:

  void init();

  void createAttributes();
  void createUniforms();
  /*uniforms id*/
  GLuint imWid_, imHid_, angleTexId_,pointsId_;
  /*tex id*/
  GLuint angleTex_;
  GLuint query_;

  GLuint feedbackBuffer_;
  GLuint feedbackLength_;
  GLfloat feedbackTr;
};

#endif /* PHOTOMETRIC_REFINEMENT_COLLECTANGLEPROGRAM_H_ */
