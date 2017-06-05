
#include <CheckReprojAreaProgram.h>


CheckReprojAreaProgram::CheckReprojAreaProgram(int imageWidth, int imageHeight) :
    ShaderProgram(imageWidth, imageHeight) {
      area_=16.0;

}

CheckReprojAreaProgram::~CheckReprojAreaProgram() {
}

void CheckReprojAreaProgram::compute(bool show) {

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  shaderManager_.enable();

  glUniformMatrix4fv(mvp1CollID_, 1, GL_FALSE, &mvp[0][0]);
    glUniform1f(areaId_, area_);

  glActiveTexture(GL_TEXTURE11);
  glBindTexture(GL_TEXTURE_2D, depthTexture_);
  glUniform1i(shadowMapIdColl_, 11);

  glEnableVertexAttribArray(posAttribCollId_);
  glBindBuffer(GL_ARRAY_BUFFER, arrayBufferObj_);
  glVertexAttribPointer(posAttribCollId_, 4, GL_FLOAT, GL_FALSE, 0, 0);

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
  glDisableVertexAttribArray(posAttribCollId_);
  glFinish();

  if (!show)
    glDisable(GL_RASTERIZER_DISCARD);

  if (useElements_Indices) {
    feedbackTr.resize(numElements_, 0.0);
    feedbackLength_ = numElements_;

  } else {
    feedbackTr.resize(sizeArray_, 0.0);
    feedbackLength_ = sizeArray_;

  }

  glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, feedbackTr.size() * sizeof(float), &feedbackTr[0]);
}

void CheckReprojAreaProgram::createTransformFeedback(int length) {
  glGenBuffers(1, &feedbackBuffer_);
  glBindBuffer(GL_ARRAY_BUFFER, feedbackBuffer_);
  glBufferData(GL_ARRAY_BUFFER, length * sizeof(float), nullptr, GL_DYNAMIC_READ);
}

void CheckReprojAreaProgram::resetTransformFeedback(int length) {
  glBindBuffer(GL_ARRAY_BUFFER, feedbackBuffer_);
  glBufferData(GL_ARRAY_BUFFER, length * sizeof(float), nullptr, GL_DYNAMIC_READ);
}

void CheckReprojAreaProgram::init() {
  shaderManager_.init();
  shaderManager_.addShader(GL_VERTEX_SHADER, "shaders/check_repr_area_vertex_shader.glsl");
  shaderManager_.addShader(GL_GEOMETRY_SHADER, "shaders/check_repr_area_geometry_shader.glsl");
  shaderManager_.addFeedbackTransform("areaCheck");
  shaderManager_.addShader(GL_FRAGMENT_SHADER, "shaders/check_repr_area_fragment_shader.glsl");
  shaderManager_.finalize();
}

void CheckReprojAreaProgram::createAttributes() {
  posAttribCollId_ = shaderManager_.getAttribLocation("position");
}

void CheckReprojAreaProgram::createUniforms() {
  mvp1CollID_ = shaderManager_.getUniformLocation("MVP");
  shadowMapIdColl_ = shaderManager_.getUniformLocation("shadowMap");
  areaId_ = shaderManager_.getUniformLocation("maxArea");
}
