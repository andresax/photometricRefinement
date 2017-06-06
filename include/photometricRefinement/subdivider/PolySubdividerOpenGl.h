/*
 * PolySubdividerOpenGl.h
 *
 *  Created on: Jun 5, 2017
 *      Author: andrea
 */

#ifndef SRC_SUBDIVIDER_POLYSUBDIVIDEROPENGL_H_
#define SRC_SUBDIVIDER_POLYSUBDIVIDEROPENGL_H_

#include <ShaderProgram.h>
#include <OpenGLProgram.h>
#include <glm.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <Mesh.h>
#include <types.hpp>
class PolySubdividerOpenGl : public OpenGLProgram{
public:
  PolySubdividerOpenGl(int imageWidth, int imageHeight, GLFWwindow* window);
  PolySubdividerOpenGl(int imageWidth, int imageHeight, GLFWwindow* window, int maxArea, int numit);
  virtual ~PolySubdividerOpenGl();

  void setVertexBufferObj(GLuint vertexBufferObj) {
    vertexBufferObj_ = vertexBufferObj;
  }

  void setNumActiveVertices(int numActiveVertices) {
    numActiveVertices_ = numActiveVertices;
  }



  void subdivide(Mesh &p, std::vector<photometricGradient::CameraType> mvps, int numActiveVertices);

  void setCurActiveVertices(const std::vector<Vertex_handle>& curActiveVertices) {
    curActiveVertices_ = curActiveVertices;
  }

  void setAreaMax(float maxArea) {
    maxArea_ = maxArea;
  }

  void setNumIt(int numIt) {
    numIt_ = numIt;
  }

private:
  void initShaders();

  int numActiveVertices_;
  GLuint depthTexture_,framebufferDepth_;
  GLuint vertexBufferObj_;
  ShaderProgram *depthMapProgram_;
  ShaderProgram *reprojectionProgram_;
  float maxArea_;
  int numIt_;
  std::vector<Vertex_handle> curActiveVertices_;
};

#endif /* SRC_SUBDIVIDER_POLYSUBDIVIDEROPENGL_H_ */
