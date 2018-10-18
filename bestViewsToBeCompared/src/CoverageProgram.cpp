#include <CoverageProgram.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

CoverageProgram::CoverageProgram(int imageWidth, int imageHeight) :
    ShaderProgram(imageWidth, imageHeight) {

  framebufferReproj_ = -1;
  imageTex_ = -1;

  posAttribReprojId_ = -1, mvp1IDReproj_ = -1, mvp2IDReproj_ = -1, shadowMap1IdReproj_ = -1, shadowMap2IdReproj_ = -1;

  depthTexture_ = -1, depthTexture2_ = -1, camCenter1ID_ = -1, camCenter2ID_ = -1;
  feedbackLength_ = -1;
  feedbackBuffer_ = -1;
}

void CoverageProgram::initTex() {
}

void CoverageProgram::compute(bool show) {

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  shaderManager_.enable();

  glUniformMatrix4fv(mvp1IDReproj_, 1, GL_FALSE, &mvp1_[0][0]);
  glUniformMatrix4fv(mvp2IDReproj_, 1, GL_FALSE, &mvp2_[0][0]);

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, depthTexture_);
  glUniform1i(shadowMap1IdReproj_, 0);

  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, depthTexture2_);
  glUniform1i(shadowMap2IdReproj_, 1);

  glUniform3fv(camCenter1ID_, 1, &camCenter1_[0]);
  glUniform3fv(camCenter2ID_, 1, &camCenter2_[0]);

  glEnableVertexAttribArray(posAttribReprojId_);
  glBindBuffer(GL_ARRAY_BUFFER, arrayBufferObj_);
  glVertexAttribPointer(posAttribReprojId_, 3, GL_FLOAT, GL_FALSE, 0, 0);

  if (!show)
    glEnable(GL_RASTERIZER_DISCARD);

  glBindBuffer(GL_ARRAY_BUFFER, arrayBufferObj_);
  glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, feedbackBuffer_);
  glBeginTransformFeedback(GL_TRIANGLES);

  if (useElements_Indices) {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementsBufferObj_);
    glDrawElements(GL_TRIANGLES, numElements_, GL_UNSIGNED_INT, 0);

  } else {
    glBindBuffer(GL_ARRAY_BUFFER, arrayBufferObj_);
    glDrawArrays(GL_TRIANGLES, 0, sizeArray_);

  }
  glEndTransformFeedback();
  glDisableVertexAttribArray(posAttribReprojId_);

  if (!show)
    glDisable(GL_RASTERIZER_DISCARD);
  if (useElements_Indices) {
    feedbackTr.resize(numElements_, (0.0));
    feedbackLength_ = numElements_;

  } else {
    feedbackTr.resize(sizeArray_, (0.0));
    feedbackLength_ = sizeArray_;

  }

  glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, feedbackTr.size() * sizeof(float), &feedbackTr[0]);

}

void CoverageProgram::initializeFramebufAndTex(GLuint &imageReprojTex) {
  glGenFramebuffers(1, &framebufferReproj_);
  glBindFramebuffer(GL_FRAMEBUFFER, framebufferReproj_);
  initRedTex(imageReprojTex);

  glGenTextures(1, &imageReprojTex);
  glBindTexture(GL_TEXTURE_2D, imageReprojTex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, imageWidth_, imageHeight_, 0, GL_RGB, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, imageReprojTex, 0);
  GLuint tmpdeptht;
  initDepthTex(tmpdeptht);
  glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, tmpdeptht, 0);
  checkFrameBuffer("CoverageProgram::initializeFramebufAndTex");
  imageTex_ = imageReprojTex;
}

void CoverageProgram::init() {
  shaderManager_.init();
  shaderManager_.addShader(GL_VERTEX_SHADER, "shaders/coverage_vertex_shader.glsl");
  shaderManager_.addShader(GL_GEOMETRY_SHADER, "shaders/coverage_geometry_shader.glsl");
  shaderManager_.addFeedbackTransform("visible");
  shaderManager_.addShader(GL_FRAGMENT_SHADER, "shaders/coverage_fragment_shader.glsl");
  shaderManager_.finalize();
  std::cout << "CoverageProgram::init" << std::endl;
}

void CoverageProgram::createAttributes() {
  posAttribReprojId_ = shaderManager_.getAttribLocation("position");
}

void CoverageProgram::createTransformFeedback(int length) {
  glGenBuffers(1, &feedbackBuffer_);
  glBindBuffer(GL_ARRAY_BUFFER, feedbackBuffer_);
  glBufferData(GL_ARRAY_BUFFER, length * sizeof(float), nullptr, GL_DYNAMIC_READ);
}

void CoverageProgram::resetTransformFeedback(int length) {
  glBindBuffer(GL_ARRAY_BUFFER, feedbackBuffer_);
  glBufferData(GL_ARRAY_BUFFER, length * sizeof(float), nullptr, GL_DYNAMIC_READ);
}

void CoverageProgram::createUniforms() {
  mvp1IDReproj_ = shaderManager_.getUniformLocation("MVP1");
  mvp2IDReproj_ = shaderManager_.getUniformLocation("MVP2");
  camCenter1ID_ = shaderManager_.getUniformLocation("camCenter1");
  camCenter2ID_ = shaderManager_.getUniformLocation("camCenter2");

  shadowMap1IdReproj_ = shaderManager_.getUniformLocation("shadowMap1");
  shadowMap2IdReproj_ = shaderManager_.getUniformLocation("shadowMap2");
}

