#ifndef BESTVIEWSTOBECOMPARED_SRC_COVERAGEPROGRAM_H_
#define BESTVIEWSTOBECOMPARED_SRC_COVERAGEPROGRAM_H_

#include <ShaderProgram.h>
#include <glm.hpp>


class CoverageProgram : public ShaderProgram{
public:
  CoverageProgram(int imageWidth, int imageHeight);
  virtual ~CoverageProgram() = default;

   void initTex();
   void createTransformFeedback(int length);
   void resetTransformFeedback(int length);

   void setFeedbackLength(GLuint feedbackLength) {
     feedbackLength_ = feedbackLength;
   }

   const std::vector<float>& getFeedbackTr() const {
     return feedbackTr;
   }

   void initializeFramebufAndTex(GLuint &imageReprojTex);

   void compute(bool renderFrameBuf = false);

   void setDepthTexture(GLuint depthTexture, GLuint depthTexture2) {
     depthTexture_ = depthTexture;
     depthTexture2_ = depthTexture2;
   }


   void setMvp(const glm::mat4& mvp1, const glm::mat4& mvp2) {
     mvp1_ = mvp1;
     mvp2_ = mvp2;
   }

   void setCamCenters(const glm::vec3& camCenter1,const glm::vec3& camCenter2) {
     camCenter1_ = camCenter1;
     camCenter2_ = camCenter2;
   }


 private:
   void init();

   GLuint feedbackBuffer_;
   GLuint feedbackLength_;
   std::vector<float> feedbackTr;
   void createAttributes();
   void createUniforms();

   GLuint framebufferReproj_;
   GLuint imageTex_;

   GLuint posAttribReprojId_, mvp1IDReproj_, mvp2IDReproj_, shadowMap1IdReproj_, shadowMap2IdReproj_;

   GLuint depthTexture_, depthTexture2_, camCenter1ID_,camCenter2ID_;
   glm::vec3 camCenter1_,camCenter2_;
   glm::mat4 mvp1_, mvp2_;
};

#endif /* BESTVIEWSTOBECOMPARED_SRC_COVERAGEPROGRAM_H_ */
