/*
 * CollectAngleProgram.cpp
 *
 *  Created on: Nov 21, 2016
 *      Author: andrea
 */

#include "CollectAngleProgram.h"

CollectAngleProgram::CollectAngleProgram(int imageWidth, int imageHeight) :
    ShaderProgram(imageWidth, imageHeight) {
   imWid_=0, imHid_=0, angleTexId_=0,pointsId_=0;
  /*tex id*/
   angleTex_=0;
   query_=0;

   feedbackBuffer_=0;
   feedbackLength_=0;
   feedbackTr = 0;

}

CollectAngleProgram::~CollectAngleProgram() {
}

void CollectAngleProgram::createTransformFeedback() {
  glGenBuffers(1, &feedbackBuffer_);
  glBindBuffer(GL_ARRAY_BUFFER, feedbackBuffer_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float), nullptr, GL_DYNAMIC_READ);

  glGenQueries(1, &query_);
}

void CollectAngleProgram::compute(bool renderFrameBuf) {
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  shaderManager_.enable();

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, angleTex_);
  glUniform1i(angleTexId_, 0);

  glUniform1i(imWid_, imageWidth_);
  glUniform1i(imHid_, imageHeight_);

  glEnableVertexAttribArray(pointsId_);
  glBindBuffer(GL_ARRAY_BUFFER, arrayBufferObj_);
  glVertexAttribPointer(pointsId_, 2, GL_FLOAT, GL_FALSE, 0, 0);

  glEnable(GL_RASTERIZER_DISCARD);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, arrayBufferObj_);

  glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, feedbackBuffer_);/*addd*/
  glBeginQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, query_);
  glBeginTransformFeedback(GL_POINTS);

  if (useElements_Indices) {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementsBufferObj_);
    glDrawElements(GL_TRIANGLES, numElements_, GL_UNSIGNED_INT, 0);
  } else {
    glBindBuffer(GL_ARRAY_BUFFER, arrayBufferObj_);
    glDrawArrays(GL_TRIANGLES, 0, sizeArray_);
  }
  glEndTransformFeedback();
  glEndQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN);

  glDisableVertexAttribArray(pointsId_);

  glFlush();
  glDisable(GL_RASTERIZER_DISCARD);

  GLuint primitives;
  glGetQueryObjectuiv(query_, GL_QUERY_RESULT, &primitives);

  feedbackLength_ = primitives;
  GLfloat feedback[1];
  glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, feedbackLength_ * sizeof(float), feedback);
  feedbackTr = feedback[0];
}

void CollectAngleProgram::init() {
  shaderManager_.init();
  shaderManager_.addShader(GL_VERTEX_SHADER, "shaders/angle_collect_vertex_shader.glsl");
  shaderManager_.addShader(GL_GEOMETRY_SHADER, "shaders/angle_collect_geometry_shader.glsl");
  shaderManager_.addFeedbackTransform("mean");
  shaderManager_.finalize();
}

void CollectAngleProgram::createAttributes() {
  pointsId_ = shaderManager_.getAttribLocation("point");

}

void CollectAngleProgram::createUniforms() {
  angleTexId_ = shaderManager_.getUniformLocation("angles");
  imWid_ = shaderManager_.getUniformLocation("imW");
   imHid_ = shaderManager_.getUniformLocation("imH");
}
