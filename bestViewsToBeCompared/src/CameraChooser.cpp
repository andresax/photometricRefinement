#include <CameraChooser.h>
#include <DepthMapProgram.h>
#include <CoverageProgram.h>
#include <AngleProgram.h>
#include <CollectAngleProgram.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

#define M_PI2 3.1415926535897932384626433832795


CameraChooser::CameraChooser() {
  angleProgram_ = depthMapProgram_ = collectAngleProgram_ = nullptr;
  overlap_ = 0;
  cur = -1;
  ref = -1;
  framebufferDepth_ = 0, framebufferDepth2_ = 0, depthTexture_ = 0, depthTexture2_ = 0, image2AngleTex_ = 0;
  vertexBufferObj_ = 0, numActiveVertices_ = 0, imageWidth_ = 0;
  imageHeight_ = 0;
  coverageProgram_ = nullptr;
  std::cout << "Using CameraChooser without gpu" << std::endl;
  mu_1=0.25;
  mu_2=0.25;
  mu_3=0.5;
}
CameraChooser::CameraChooser(int imageWidth, int imageHeight) {
  framebufferDepth_ = 0, framebufferDepth2_ = 0, depthTexture_ = 0, depthTexture2_ = 0, image2AngleTex_ = 0;
  vertexBufferObj_ = 0, numActiveVertices_ = 0, imageWidth_ = imageWidth;
  imageHeight_ = imageHeight;
  initOpenGLStuff();
  glGenBuffers(1, &vertexBufferObj_);
  mu_1=0.25;
  mu_2=0.25;
  mu_3=0.5;

}

CameraChooser::CameraChooser(int imageWidth, int imageHeight, GLuint vertexBufferObj) {
  framebufferDepth_ = 0, framebufferDepth2_ = 0, depthTexture_ = 0, depthTexture2_ = 0, image2AngleTex_ = 0;
  vertexBufferObj_ = 0, numActiveVertices_ = 0, imageWidth_ = imageWidth;
  imageHeight_ = imageHeight;
  vertexBufferObj_ = vertexBufferObj;
  initShaders();
  mu_1=0.25;
  mu_2=0.25;
  mu_3=0.5;
}
CameraChooser::~CameraChooser() {
}
void CameraChooser::initOpenGLStuff() {
  init();
  initShaders();
}

void CameraChooser::getKNear(const std::vector<photometricGradient::CameraType>& cameraList, int maxCam, std::vector<std::vector<int>> &out, std::vector<int> consideredIdx) {

  if (consideredIdx.size() == 0) {
    consideredIdx.resize(cameraList.size());
    std::iota(consideredIdx.begin(), consideredIdx.end(), 0);
  }

  std::vector<std::vector<bool>> done;
  std::vector<int> tmp(cameraList.size(), 0);
  std::vector<bool> tmp2(cameraList.size(), false);
  done.assign(cameraList.size(), tmp2);
  out.assign(cameraList.size(), tmp);

  for (int curIdx = 0; curIdx < cameraList.size(); curIdx++) {
    cams.clear();
    for (int curC = curIdx + 1; curC < cameraList.size(); curC++) {
      float goodness1;

      computeGoodness(cameraList[curIdx], cameraList[curC], goodness1);
      cams.push_back(std::pair<int, float>(curC, goodness1));
    }

    std::vector<int> indices(cams.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(std::begin(indices), std::end(indices), [&] (int i, int j) {
      return cams[i].second > cams[j].second;
    });

//    for(auto i:indices)
//    std::cout<<i<<std::endl;
//    for(auto c:cams)
//    std::cout<<c.first<<" "<<c.second<<std::endl;
    std::vector<int> tmp;
    int countCam = 0;
    int i = 0;
    while (countCam < maxCam && i < indices.size()) {

      if (done[curIdx][indices[i]] == false) {
        tmp.push_back(cams[indices[i]].first);
        done[curIdx][indices[i]] = true;
        done[indices[i]][curIdx] = true;
        countCam++;
      }
      ++i;
    }
    out[curIdx] = tmp;
  }
}

void CameraChooser::getKNearWithoutRep(const std::vector<photometricGradient::CameraType>& cameraList, int maxCam, std::vector<std::vector<int>> &out,
    std::vector<int> consideredIdx) {

  if (consideredIdx.size() == 0) {
    consideredIdx.resize(cameraList.size());
    std::iota(consideredIdx.begin(), consideredIdx.end(), 0);
  }

  std::vector<std::vector<bool>> done;
  std::vector<int> tmp(cameraList.size(), 0);
  std::vector<bool> tmp2(cameraList.size(), false);
  done.assign(cameraList.size(), tmp2);
  out.assign(cameraList.size(), std::vector<int>());

  std::vector<std::vector<float>> goodness;
  goodness.resize(cameraList.size());
  //compute goodness matrix

  for (int curIdx = 0; curIdx < cameraList.size(); curIdx++) {
    std::vector<float> tmp(cameraList.size(), 0.0);
    goodness[curIdx] = tmp;
  }

  for (int curIdx = 0; curIdx < cameraList.size(); curIdx++) {
    for (int curC = curIdx + 1; curC < cameraList.size(); curC++) {
      float g;
      computeGoodness(cameraList[curIdx], cameraList[curC], g);
      goodness[curIdx][curC] = g;
      goodness[curC][curIdx] = g;
    }
  }
  std::cout << "goodness computed" << std::endl;

  for (int curIdx = 0; curIdx < cameraList.size(); curIdx++) {

    bool endedFound = false;
    float w = 0.5;
    do {
      std::vector<float> newGoodness(cameraList.size(), 0.0f);
      for (int curCam = curIdx + 1; curCam < cameraList.size(); ++curCam) {
        float curGoodness = goodness[curIdx][curCam];
        for (auto id : out[curIdx]) {
          curGoodness = curGoodness + w * goodness[id][curCam];
        }
        newGoodness[curCam] = curGoodness / static_cast<float>(1 + w * static_cast<float>(out[curIdx].size()));
      }

      std::vector<int> indices(cameraList.size());
      std::iota(indices.begin(), indices.end(), 0);
      std::sort(std::begin(indices), std::end(indices), [&] (int i, int j) {
        return newGoodness[i] > newGoodness[j];
      });

//      std::cout << "newGoodness " << std::endl;
//      for (auto n : newGoodness)
//        std::cout << n << " " << std::flush;
//      std::cout << std::endl<<" indices" << std::endl;
//      for (auto n : indices)
//        std::cout << n << " " << std::flush;
//      std::cout << " " << std::endl;

      endedFound = true;
      for (int i = 0; i < indices.size() && endedFound; ++i) {
        if (done[curIdx][indices[i]] == false) {
          //std::cout << "curIdx "<<curIdx<<" indices[i] "<<indices[i]<<" i "<<i << std::endl;
          done[curIdx][indices[i]] = true;
          //done[indices[i]][curIdx] = true;
          out[curIdx].push_back(indices[i]);
          endedFound = false;
        }
      }

    } while (!endedFound && out[curIdx].size() < maxCam);

  }
}

void CameraChooser::getKNearWithCoverage(const std::vector<photometricGradient::CameraType>& cameraList, int maxCam, std::vector<std::vector<int>> &out,
    std::vector<int> consideredIdx) {

  if (consideredIdx.size() == 0) {
    consideredIdx.resize(cameraList.size());
    std::iota(consideredIdx.begin(), consideredIdx.end(), 0);
  }

  std::vector<int> visible(mesh_->p.size_of_facets(), 0);

  std::vector<std::vector<bool>> done;
  std::vector<int> tmp(cameraList.size(), 0);
  std::vector<bool> tmp2(cameraList.size(), false);
  done.assign(cameraList.size(), tmp2);
  out.assign(cameraList.size(), std::vector<int>());

  std::vector<std::vector<float>> goodness;
  goodness.resize(cameraList.size());

  std::vector<std::vector<int>> indicesOrder;
  indicesOrder.resize(cameraList.size());

  std::vector<int> lastIdx;
  lastIdx.resize(cameraList.size());
  //compute goodness matrix

  for (int curIdx = 0; curIdx < cameraList.size(); curIdx++) {
    std::vector<float> tmp(cameraList.size(), 0.0);
    std::vector<int> tmpint(cameraList.size(), 0);
    goodness[curIdx] = tmp;
    indicesOrder[curIdx] = tmpint;
  }

  for (int curIdx = 0; curIdx < cameraList.size(); curIdx++) {
    for (int curC = curIdx + 1; curC < cameraList.size(); curC++) {
      float g;
      computeGoodness(cameraList[curIdx], cameraList[curC], g);
      goodness[curIdx][curC] = g;
      goodness[curC][curIdx] = g;
    }
  }
  std::cout << "goodness computed" << std::endl;

  float totGoodness = 0.0;
  for (int curIdx = 0; curIdx < cameraList.size(); curIdx++) {
    std::iota(indicesOrder[curIdx].begin(), indicesOrder[curIdx].end(), 0);
    std::sort(std::begin(indicesOrder[curIdx]), std::end(indicesOrder[curIdx]), [&] (int i, int j) {
      return goodness[curIdx][i] > goodness[curIdx][j];
    });
    int curUnusedIdx = 0;
    out[curIdx].push_back(indicesOrder[curIdx][curUnusedIdx]);
    lastIdx[curIdx] = curUnusedIdx;
    totGoodness += goodness[curIdx][indicesOrder[curIdx][curUnusedIdx]];
    std::vector<bool> vis;
    computeCoverage(cameraList[curIdx], cameraList[indicesOrder[curIdx][curUnusedIdx]], vis);
    for (int curV = 0; curV < vis.size(); curV++) {
      if (vis[curV])
        visible[curV]++;
    }
  }
  std::cout<< "End INIT " <<std::endl;
  for (int curIdx = 0; curIdx < cameraList.size(); curIdx++) {
    std::cout<< curIdx <<"---> " <<out[curIdx][0]<<std::endl;
  }

  float referenceMean, referenceVar;
  meanAndVarVec(visible, referenceMean, referenceVar);

  bool ended = false;
  float newTotGoodness = totGoodness;

  while (!ended && newTotGoodness > 0.9 * totGoodness) {
    bool changed = false;
    for (int curIdx = 0; curIdx < cameraList.size() && changed == false; curIdx++) {
      if (lastIdx[curIdx] + 1 < cameraList.size()) {
        int curUnusedIdx = lastIdx[curIdx] + 1;

        std::vector<bool> vis, visOld;
        computeCoverage(cameraList[curIdx], cameraList[indicesOrder[curIdx][curUnusedIdx]], vis);
        computeCoverage(cameraList[curIdx], cameraList[indicesOrder[curIdx][curUnusedIdx - 1]], visOld);
        float tempGoodness = newTotGoodness - goodness[curIdx][indicesOrder[curIdx][curUnusedIdx - 1]] + goodness[curIdx][indicesOrder[curIdx][curUnusedIdx]];
        for (int curV = 0; curV < vis.size(); curV++) {
          if (vis[curV])
            visible[curV]++;
          if (visOld[curV])
            visible[curV]--;
        }

        float curMean, curVar;
        meanAndVarVec(visible, curMean, curVar);

        bool improved = (curVar < referenceVar && curMean >= referenceMean);

        if (improved) {
//          std::cout<< "Improvement for cam  "<< curIdx<<std::flush;
//          std::cout<< " from  "<< out[curIdx][0]<<std::flush;
//          std::cout<< " to  "<< indicesOrder[curIdx][curUnusedIdx]<<std::flush;
//          std::cout<< " goodness from  "<< newTotGoodness << " to " << tempGoodness<<std::flush;
//          std::cout<< " referenceMean from  "<< referenceMean << " to " << curMean<<std::flush;
//          std::cout<< " referenceVar from  "<< referenceVar << " to " << curVar<<std::endl;

          referenceMean = curMean;
          referenceVar = curVar;
          out[curIdx][0] = (indicesOrder[curIdx][curUnusedIdx]);
          lastIdx[curIdx] = curUnusedIdx;
          newTotGoodness = tempGoodness;
          changed = true;
        } else {
          for (int curV = 0; curV < vis.size(); curV++) {
            if (vis[curV])
              visible[curV]--;
            if (visOld[curV])
              visible[curV]++;
          }
        }

      }
    }
    if (!changed) {
      ended = true;
    }
//    std::cout<< " newTotGoodness =   "<< newTotGoodness << " 0.9 * totGoodness= " << 0.9 * totGoodness<<std::flush;
//    std::cout<< " ended =   "<< ended <<std::endl;
  }
}

void CameraChooser::meanAndVarVec(std::vector<int> vis, float &mean, float&stddev) {

  float sum = std::accumulate(vis.begin(), vis.end(), 0.0);

  std::vector<float> sqrVec;
  std::transform(vis.begin(), vis.end(), std::back_inserter(sqrVec), [](int n) {return std::pow(n,2);});
  float sumSqr = std::accumulate(sqrVec.begin(), sqrVec.end(), 0.0);

  mean = sum / vis.size();

  std::vector<double> diff(vis.size());
  std::transform(vis.begin(), vis.end(), diff.begin(), std::bind2nd(std::minus<double>(), mean));
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  stddev = std::sqrt(sq_sum / vis.size());
}
void CameraChooser::getKNear(const std::vector<photometricGradient::CameraType>& cameraList, int curIdx, int num, std::vector<int> &out, std::vector<int> consideredIdx) {

  if (consideredIdx.size() == 0) {
    consideredIdx.resize(cameraList.size());
    std::iota(consideredIdx.begin(), consideredIdx.end(), 0);
  }
//std::cout<<"--curIdx: "<<curIdx<<" -->";
  if (curIdx <= cameraList.size()) {
    std::vector<std::pair<int, float> > cameras;
    for (int curC : consideredIdx) {

      if (curIdx != curC) {
        float goodness1;
        cur = curC;
        ref = curIdx;
        computeGoodness(cameraList[curIdx], cameraList[curC], goodness1);
        cameras.push_back(std::pair<int, float>(curC, goodness1));
      }
    }
    std::sort(cameras.begin(), cameras.end(), [] (std::pair<int, float> i, std::pair<int, float> j) {
      return i.second > j.second;
    });

    for (int curC = 0; curC < num; curC++) {
      out.push_back(cameras[curC].first);
    }
  }

}

void CameraChooser::getNearByThreshold(const std::vector<photometricGradient::CameraType>& cameraList, int curIdx, float th, std::vector<int> &out) {

//  std::cout<<"--curIdx: "<<curIdx<<" -->";
  if (curIdx <= cameraList.size()) {
    std::vector<std::pair<int, float> > cameras;
    for (int curC = 0; curC < cameraList.size(); curC++) {

      if (curIdx != curC) {
        float goodness1;
        cur = curC;
        ref = curIdx;
        computeGoodness(cameraList[curIdx], cameraList[curC], goodness1);
        cameras.push_back(std::pair<int, float>(curC, goodness1));
      }
    }
    std::sort(cameras.begin(), cameras.end(), [] (std::pair<int, float> i, std::pair<int, float> j) {
      return i.second > j.second;
    });

    for (int curC = 0; curC < cameraList.size(); curC++) {
      if (cameras[curC].second > th) {
        out.push_back(cameras[curC].first);
        std::cout << " " << cameras[curC].first;
      }
    }

    std::cout << std::endl;
  }

}

void CameraChooser::getExaustive(const std::vector<photometricGradient::CameraType>& cameraList, std::vector<std::vector<int>>& out, std::vector<int> &consideredIdx) {
  if (consideredIdx.size() == 0) {
    consideredIdx.resize(cameraList.size());
    std::iota(consideredIdx.begin(), consideredIdx.end(), 0);
  }
  std::vector<int> tmp(0, 0);
  out.assign(cameraList.size(), tmp);
  for (int it = 0; it < cameraList.size(); ++it) {
    for (int it2 = it + 1; it2 < cameraList.size(); ++it2) {
      if (it != it2) {
        out[it].push_back(it2);
      }
    }
  }
}

void CameraChooser::getBestBySfmOutput(const std::shared_ptr<SfMData>& sfm, int curIdx, int minMatchesNum, int maxCam, std::vector<int>& out) {

  computeMutualMatches(sfm);

  std::vector<int> indices(mutualMatches[curIdx].size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(std::begin(indices), std::end(indices), [&](int i1, int i2) {return mutualMatches[curIdx][i1] > mutualMatches[curIdx][i2];});
//
 std::cout<<curIdx<<": " <<std::flush;
 for (auto i : mutualMatches[curIdx])
   std::cout << " " << i << std::flush;
 std::cout << std::endl;
  for (int i = 0; i < ((maxCam < indices.size()) ? maxCam : indices.size()); ++i) {
    if (mutualMatches[curIdx][indices[i]] > minMatchesNum)
      out.push_back(indices[i]);
  }
}

void CameraChooser::getBestBySfmOutput(const std::shared_ptr<SfMData>& sfm, int minMatchesNum, int maxCam, std::vector<std::vector<int>>& out) {

  computeMutualMatches(sfm);
  std::vector<std::vector<bool>> done;
  std::vector<int> tmp(sfm->camerasList_.size(), 0);
  std::vector<bool> tmp2(sfm->camerasList_.size(), false);
  done.assign(sfm->camerasList_.size(), tmp2);
  out.assign(sfm->camerasList_.size(), tmp);

  for (int curIdx = 0; curIdx < sfm->camerasList_.size(); ++curIdx) {

    std::vector<int> indices(mutualMatches[curIdx].size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(std::begin(indices), std::end(indices), [&](int i1, int i2) {return mutualMatches[curIdx][i1] > mutualMatches[curIdx][i2];});

    std::vector<int> tmp;
    int countCam = 0;
    int i = 0;
    while (countCam < maxCam && i < indices.size()) {

      if (mutualMatches[curIdx][indices[i]] > minMatchesNum && done[curIdx][indices[i]] == false) {
        tmp.push_back(indices[i]);
        done[curIdx][indices[i]] = true;
        done[indices[i]][curIdx] = true;
        countCam++;
      }
      ++i;
    }

    out[curIdx] = tmp;
  }
//


}
void CameraChooser::computeMutualMatches(const std::shared_ptr<SfMData>& sfm) {
  if (mutualMatches.size() == 0) {
    std::vector<int> tmp(sfm->camerasList_.size(), 0);
    mutualMatches.assign(sfm->camerasList_.size(), tmp);
    std::cout <<"CAM LIST SIZE: "<<sfm->camerasList_.size()<<std::endl;

    for (int cam1 = 0; cam1 < sfm->camerasList_.size(); cam1++) {
      std::sort(sfm->pointsVisibleFromCamN_[cam1].begin(), sfm->pointsVisibleFromCamN_[cam1].end());
    }

    for (int cam1 = 0; cam1 < sfm->camerasList_.size(); cam1++) {
      for (int cam2 = cam1 + 1; cam2 < sfm->camerasList_.size(); cam2++) {
        for (auto ptID : sfm->pointsVisibleFromCamN_[cam1]) {
          if (std::binary_search(sfm->pointsVisibleFromCamN_[cam2].begin(), sfm->pointsVisibleFromCamN_[cam2].end(), ptID)) {
            float angle = angleThreePoints(sfm->camerasList_[cam1].center, sfm->points_[ptID], sfm->camerasList_[cam2].center);

            if (angle > 0.0f / 180.0f * PI && angle < 180.0f / 180.0f * PI) {
              mutualMatches[cam1][cam2]++;
              mutualMatches[cam2][cam1]++;
            }
          }
        }
      }
    }
  }
}

float CameraChooser::angleThreePoints(glm::vec3 p1, glm::vec3 p, glm::vec3 p2) {

  glm::vec3 x1 = glm::normalize(p1 - p);
  glm::vec3 x2 = glm::normalize(p2 - p);

  return 2 * glm::atan(glm::length(x1 * glm::length(x2) - glm::length(x1) * x2), glm::length(x1 * glm::length(x2) + glm::length(x1) * x2));

}

void CameraChooser::computeGoodness(const photometricGradient::CameraType &cam1, const photometricGradient::CameraType &cam2, float &goodness1) {

  glm::mat4 mvp1 = cam1.mvp;
  glm::mat4 mvp2 = cam2.mvp;

  glm::vec3 center1 = cam1.center;
  glm::vec3 center2 = cam2.center;
//***c***************DEPTH MAP *******************************
  depthMapProgram_->setArrayBufferObj(vertexBufferObj_, numActiveVertices_);
  static_cast<photometricGradient::DepthMapProgram *>(depthMapProgram_)->computeDepthMap(framebufferDepth_, mvp1);
  static_cast<photometricGradient::DepthMapProgram *>(depthMapProgram_)->computeDepthMap(framebufferDepth2_, mvp2);
  glFinish();

//**********************AngleProgram*******************************
  angleProgram_->setArrayBufferObj(vertexBufferObj_, numActiveVertices_);
  static_cast<AngleProgram *>(angleProgram_)->setCamCenters(center1, center2);
  static_cast<AngleProgram *>(angleProgram_)->setDepthTexture(depthTexture_, depthTexture2_);
  static_cast<AngleProgram *>(angleProgram_)->setMvp(mvp1, mvp2);
  angleProgram_->compute(true);
  glFinish();

  GLfloat *pixels_;
  pixels_ = new GLfloat[imageWidth_ * imageHeight_ * 3];

  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, image2AngleTex_);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ROW_LENGTH, 0);
  glPixelStorei(GL_PACK_SKIP_ROWS, 0);
  glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
  glBindTexture(GL_TEXTURE_2D, image2AngleTex_);
  glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_FLOAT, pixels_);
  glDisable(GL_TEXTURE_2D);

  SwapBuffers();

  float accSin = 0.0, accZSin = 0.0, curX, curY, curZ;
  float accCos = 0.0, accZCos = 0.0;
  std::vector<float> accSinV,accCosV,accZSinV,accZCosV;
  long int countTotPixelInRefAndCur = 0, countTotPixelInRef = 0, countTotPixelsImage;
  for (int i = 0; i < imageHeight_; ++i) {
    for (int j = 0; j < imageWidth_; ++j) {
      curX = (float) (pixels_[(imageHeight_ - i - 1) * 3 * imageWidth_ + j * 3 + 0]);
      curY = (float) (pixels_[(imageHeight_ - i - 1) * 3 * imageWidth_ + j * 3 + 1]); //
      curZ = (float) (pixels_[(imageHeight_ - i - 1) * 3 * imageWidth_ + j * 3 + 2]);
      if (curX != 0.0) {
        countTotPixelInRef++;
        if (curY != 0.0) {
          accCos = accCos + std::cos(static_cast<float>(curX));
          accZCos = accZCos + std::cos(static_cast<float>(curZ));
          accSin = accSin +std::sin( static_cast<float>(curX));
          accZSin = accZSin + std::sin(static_cast<float>(curZ));

          accCosV.push_back(std::cos(static_cast<float>(curX)));
          accZCosV.push_back(std::cos(static_cast<float>(curZ)));
          accSinV.push_back(std::sin( static_cast<float>(curX)));
          accZSinV.push_back(std::sin(static_cast<float>(curZ)));


          countTotPixelInRefAndCur++;
        }
      }
    }
  }
  countTotPixelsImage = imageHeight_ * imageWidth_;

//  std::stringstream s;
//  s << ref << "_" << cur << "=" << static_cast<float>(countTotPixelInRefAndCur) / static_cast<float>(countTotPixelInRef);
//  cv::Mat tem = cv::Mat(imageHeight_, imageWidth_, CV_8UC3);
//  for (int i = 0; i < imageHeight_; ++i) {
//    for (int j = 0; j < imageWidth_; ++j) {
//      for (int curCh = 0; curCh < 3; ++curCh) {
//        tem.at<cv::Vec3b>(i, j)[2 - curCh] = (unsigned char) 100 * (pixels_[(imageHeight_ - i - 1) * 3 * imageWidth_ + j * 3 + curCh]);
//      }
//    }
//  }
//  cv::imwrite("result" + s.str() + ".png", tem);

  delete (pixels_);

  float mean = 50 * M_PI / 180;
  float stddev = M_PI / 4;
  float stddev2 = M_PI / 3;

  float meanAngleCos = accCos / static_cast<float>(countTotPixelInRefAndCur);
  float meanDiffSymAngleCos = accZCos / static_cast<float>(countTotPixelInRefAndCur);
  float meanAngleSin = accSin / static_cast<float>(countTotPixelInRefAndCur);
  float meanDiffSymAngleSin = accZSin / static_cast<float>(countTotPixelInRefAndCur);

  float meanAngle = std::atan2(meanAngleSin,meanAngleCos);
  float meanDiffSymAngle = std::atan2(meanDiffSymAngleSin,meanDiffSymAngleCos);

  std::sort(accCosV.begin(), accCosV.end());
  std::sort(accSinV.begin(), accSinV.end());
  std::sort(accZCosV.begin(), accZCosV.end());
  std::sort(accZSinV.begin(), accZSinV.end());

  float medianAngle= std::atan2(accSinV[accSinV.size()/2],accCosV[accCosV.size()/2]);
  float medianDiffSymAngle = std::atan2(accZSinV[accZSinV.size()/2],accZCosV[accZCosV.size()/2]);
  //float enoughBaselineLog = exp(-((log(mean) - log(meanAngle)) * (log(mean) - log(meanAngle))) / (2 * stddev * stddev));
//  float enoughBaseline = exp(-((mean - meanAngle) * (mean - meanAngle)) / (2 * stddev2 * stddev2));
  float enoughBaseline = exp(-((mean - medianAngle) * (mean - medianAngle)) / (2 * stddev2 * stddev2));

//  float angleSymmetry = exp(-((0 - meanDiffSymAngle) * (0 - meanDiffSymAngle)) / (2 * stddev * stddev));
  float angleSymmetry = exp(-((0 - medianDiffSymAngle) * (0 - medianDiffSymAngle)) / (2 * stddev * stddev));
  float overlap = abs(static_cast<float>(countTotPixelInRefAndCur) / static_cast<float>(countTotPixelInRef));

  goodness1 = mu_1 * enoughBaseline + mu_2 * angleSymmetry + mu_3 * overlap;

  overlap_ = overlap;

//  std::cout << "Cam " << cur << " ";
//  std::cout << "countTotPixelInRef% " << countTotPixelInRef << " ";
//  std::cout << "countTotPixelInRefAndCur " << countTotPixelInRefAndCur << " ";
//  std::cout << "meanAngle " << meanAngle << " ";
//  std::cout << "meanDiffSymAngle " << meanDiffSymAngle << " ";
//  std::cout << "angleSymmetry " << angleSymmetry << " ";
//  std::cout << "enoughBaseline " << enoughBaseline << " ";
//  std::cout << "overlap " << overlap << " ";
//  std::cout << "goodness1 " << goodness1 << std::endl;

}

void CameraChooser::computeCoverage(const photometricGradient::CameraType &cam1, const photometricGradient::CameraType &cam2, std::vector<bool> &coverage) {

  glm::mat4 mvp1 = cam1.mvp;
  glm::mat4 mvp2 = cam2.mvp;

  glm::vec3 center1 = cam1.center;
  glm::vec3 center2 = cam2.center;
//***c***************DEPTH MAP *******************************
  depthMapProgram_->setArrayBufferObj(vertexBufferObj_, numActiveVertices_);
  static_cast<photometricGradient::DepthMapProgram *>(depthMapProgram_)->computeDepthMap(framebufferDepth_, mvp1);
  static_cast<photometricGradient::DepthMapProgram *>(depthMapProgram_)->computeDepthMap(framebufferDepth2_, mvp2);
  glFinish();

//**********************CoverageProgram*******************************
  coverageProgram_->setArrayBufferObj(vertexBufferObj_, numActiveVertices_);
  static_cast<CoverageProgram *>(coverageProgram_)->resetTransformFeedback(numActiveVertices_);
  static_cast<CoverageProgram *>(coverageProgram_)->setCamCenters(center1, center2);
  static_cast<CoverageProgram *>(coverageProgram_)->setDepthTexture(depthTexture_, depthTexture2_);
  static_cast<CoverageProgram *>(coverageProgram_)->setMvp(mvp1, mvp2);
  static_cast<CoverageProgram *>(coverageProgram_)->compute(false);
  glFinish();

  SwapBuffers();

  coverage.assign(mesh_->p.size_of_facets(), 0.0);
  std::vector<float> out = static_cast<CoverageProgram *>(coverageProgram_)->getFeedbackTr();
  int curF = 0;
  for (int curV = 0; curV < out.size(); curV += 3) {
    //std::cout<<out[curV] << " " << out[curV+1]<< " "<<out[curV+2]<<std::endl;
    if (out[curV] + out[curV+1] + out[curV+2] > 1)
      coverage[curF] = true;
    else
      coverage[curF] = false;
    curF++;
  }
}

void CameraChooser::initShaders() {
//************************depth********************************
  std::cout << "photometricGradient::DepthMapProgram init...";
  depthMapProgram_ = new photometricGradient::DepthMapProgram(imageWidth_, imageHeight_);
  depthMapProgram_->initializeProgram();
  depthMapProgram_->setUseElementsIndices(false);
  static_cast<photometricGradient::DepthMapProgram *>(depthMapProgram_)->initializeFramebufAndTex(framebufferDepth_, depthTexture_);
  static_cast<photometricGradient::DepthMapProgram *>(depthMapProgram_)->initializeFramebufAndTex(framebufferDepth2_, depthTexture2_);
  std::cout << "DONE" << std::endl;

//************************AngleProgram**************************
  std::cout << "AngleProgram init...";
  angleProgram_ = new AngleProgram(imageWidth_, imageHeight_);
  angleProgram_->initializeProgram();
  angleProgram_->setUseElementsIndices(false);
  static_cast<AngleProgram *>(angleProgram_)->initializeFramebufAndTex(image2AngleTex_);
  std::cout << "DONE" << std::endl;

//**********************CoverageProgram*************************************
  std::cout << "CoverageProgram init...";
  coverageProgram_ = new CoverageProgram(imageWidth_, imageHeight_);
  coverageProgram_->initializeProgram();
  coverageProgram_->setUseElementsIndices(false);
  static_cast<CoverageProgram *>(coverageProgram_)->createTransformFeedback(0);
  std::cout << "DONE" << std::endl;
}

void CameraChooser::resetMeshInfo(bool checkVisibility) {
  glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObj_);
  std::vector<glm::vec3> verticesUnwrapped;

  for (Facet_iterator itFac = mesh_->p.facets_begin(); itFac != mesh_->p.facets_end(); itFac++) {

    Halfedge_handle h0 = itFac->halfedge();
    Halfedge_handle h1 = h0->next();
    Halfedge_handle h2 = h1->next();

    if (!checkVisibility || (h0->vertex()->getVisibility() || h1->vertex()->getVisibility() || h2->vertex()->getVisibility())) {
      verticesUnwrapped.push_back(glm::vec3(h0->vertex()->point().x(), h0->vertex()->point().y(), h0->vertex()->point().z()));
      verticesUnwrapped.push_back(glm::vec3(h1->vertex()->point().x(), h1->vertex()->point().y(), h1->vertex()->point().z()));
      verticesUnwrapped.push_back(glm::vec3(h2->vertex()->point().x(), h2->vertex()->point().y(), h2->vertex()->point().z()));
    }
  }
  numActiveVertices_ = verticesUnwrapped.size();

  glBufferData(GL_ARRAY_BUFFER, verticesUnwrapped.size() * sizeof(glm::vec3), &verticesUnwrapped[0], GL_DYNAMIC_DRAW);
  verticesUnwrapped.clear();
}
