#include <utilities.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

//#define DEBUG_OPTIMIZATION_VERBOSE
//#define DEBUG_OPTIMIZATION

namespace utilities {

glm::vec2 projectPoint(const glm::mat4 &cameraMatrix, const glm::vec3 &pt) {
  glm::vec4 ptH = glm::vec4(pt, 1.0);
//  glm::vec4 ptimH = glm::transpose(cameraMatrix) * ptH;
  glm::vec4 ptimH = ptH * cameraMatrix;
  return glm::vec2(ptimH.x / ptimH.z, ptimH.y / ptimH.z);
}

std::string getFrameNumber(int curFrame, int digitIdxLength) {
  std::ostringstream curNumber;
  if (digitIdxLength > 0) {
    int n = curFrame;
    int curNumOfDigit = curFrame == 0 ? 1 : 0;
    while (n > 0) {
      n /= 10;
      ++curNumOfDigit;
    }
    while (curNumOfDigit < digitIdxLength) {
      curNumber << "0";
      curNumOfDigit++;
    }
  }
  curNumber << curFrame;
  return curNumber.str();
}

void writeObj(std::vector<glm::vec3> &vertices, std::string path) {
  std::ofstream fileOut(path);
  for (auto v : vertices) {
    fileOut << "v " << v.x << " " << v.y << " " << v.z << " " << std::endl;
  }

  for (int curF = 0; curF < vertices.size(); curF += 3) {
    fileOut << "f " << curF + 1 << " " << curF + 2 << " " << curF + 3 << " " << std::endl;
  }

  fileOut.close();
}

glm::mat3 rotX(float alpha) {
  glm::mat3 rot; //set identity matrix
  rot[1][1] = cos(alpha);
  rot[1][2] = -sin(alpha);
  rot[2][1] = sin(alpha);
  rot[2][2] = cos(alpha);
  return rot;
}

glm::mat3 rotY(float alpha) {
  glm::mat3 rot; //set identity matrix
  rot[0][0] = cos(alpha);
  rot[0][2] = sin(alpha);
  rot[2][0] = -sin(alpha);
  rot[2][2] = cos(alpha);
  return rot;
}

void printMatrix(glm::mat4 matrix) {
  for (int curRow = 0; curRow < 4; ++curRow) {
    for (int curCol = 0; curCol < 4; ++curCol) {
      std::cout << matrix[curRow][curCol] << " ";
    }
    std::cout << std::endl;
  }
}
void printMatrix(const std::string message, glm::mat4 matrix) {
  std::cout << message << std::endl;
  for (int curRow = 0; curRow < 4; ++curRow) {
    for (int curCol = 0; curCol < 4; ++curCol) {
      std::cout << matrix[curRow][curCol] << " ";
    }
    std::cout << std::endl;
  }
}
void printMatrix(glm::mat3 matrix) {
  for (int curRow = 0; curRow < 3; ++curRow) {
    for (int curCol = 0; curCol < 3; ++curCol) {
      std::cout << matrix[curRow][curCol] << " ";
    }
    std::cout << std::endl;
  }
}

void printMatrix(std::string message, glm::mat3 matrix) {
  std::cout << message << std::endl;
  for (int curRow = 0; curRow < 3; ++curRow) {
    for (int curCol = 0; curCol < 3; ++curCol) {
      std::cout << matrix[curRow][curCol] << " ";
    }
    std::cout << std::endl;
  }
}

void printMatrix(glm::vec3 vector) {
  for (int curIdx = 0; curIdx < 3; ++curIdx) {
    std::cout << vector[curIdx] << " ";
  }
  std::cout << std::endl;
}

void printMatrix(std::string message, glm::vec4 vector) {
  std::cout << message << std::endl;
  printMatrix(vector);
}
void printMatrix(glm::vec4 vector) {
  for (int curIdx = 0; curIdx < 4; ++curIdx) {
    std::cout << vector[curIdx] << " ";
  }
  std::cout << std::endl;
}

void printMatrix(std::string message, glm::vec3 vector) {
  std::cout << message << std::endl;
  for (int curIdx = 0; curIdx < 3; ++curIdx) {
    std::cout << vector[curIdx] << " ";
  }
  std::cout << std::endl;
}

glm::mat3 rotZ(float alpha) {
  glm::mat3 rot; //set identity matrix
  rot[0][0] = cos(alpha);
  rot[0][1] = -sin(alpha);
  rot[1][0] = sin(alpha);
  rot[1][1] = cos(alpha);
  return rot;
}

void readLineAndStore(std::ifstream &configFile, bool &value) {
  std::string line;
  int valueRead;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> valueRead;
  if (valueRead == 0) {
    value = false;
  } else {
    value = true;
  }
}

void readLineAndStore(std::ifstream &configFile, int &value) {
  std::string line;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> value;
}

void readLineAndStore(std::ifstream &configFile, double &value) {
  std::string line;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> value;
}
void readLineAndStore(std::ifstream &configFile, float &value) {
  std::string line;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> value;
}

void readLineAndStore(std::ifstream &configFile, std::string &value) {
  std::string line;
  std::getline(configFile, line);

  std::istringstream iss(line);
  iss >> value;
  std::cout << iss.str() << std::endl;
  if (value.at(0) == '#') {
    value = std::string("");
  }
}

void saveVisibilityPly(const SfMData& sfm_data, const std::string &name) {

  std::vector<glm::vec3> camCenters;
  std::vector<glm::vec3> points;
  for (auto c : sfm_data.camerasList_) {
    camCenters.push_back(c.center);
  }

  for (auto p : sfm_data.points_) {
    points.push_back(p);
  }

  utilities::saveVisibilityPly(camCenters, points, sfm_data.pointsVisibleFromCamN_, name);
}

void saveVisibilityPly(const std::vector<glm::vec3> &camCenters, const std::vector<glm::vec3> & points, const std::vector<std::vector<int> >& visibility,
    const std::string &name, bool pointsVisibleFromCamN) {

  std::ofstream visFile(name + ".ply");
//
  int numVisRays = 0;
  for (int curIdx = 0; curIdx < visibility.size(); ++curIdx) {
    numVisRays += visibility[curIdx].size();
  }
  std::cout << "Writing initVis_" << std::endl;
  std::cout << "numVisRays_: " << numVisRays << std::endl;
  std::cout << "camCenters_.size(): " << camCenters.size() << std::endl;

  visFile << "ply" << std::endl << "format ascii 1.0" << std::endl;
  visFile << "element vertex " << points.size() + camCenters.size() << std::endl;
  visFile << "property float x " << std::endl << "property float y " << std::endl << "property float z " << std::endl;
  visFile << "element edge " << numVisRays << std::endl;
  visFile << "property int vertex1 " << std::endl << "property int vertex2" << std::endl;
  visFile << "end_header" << std::endl;

  for (auto curCam : camCenters) {
    visFile << curCam.x << " " << curCam.y << " " << curCam.z << std::endl;
  }

  for (auto curPt : points) {
    visFile << curPt.x << " " << curPt.y << " " << curPt.z << std::endl;
  }

  if (pointsVisibleFromCamN) {
    for (int curCamIdx = 0; curCamIdx < camCenters.size(); ++curCamIdx) {

      for (auto curPtIdx : visibility[curCamIdx]) {

        visFile << curCamIdx << " " << camCenters.size() + curPtIdx << std::endl;
      }
      //  std::cout << "Cam: " << curCamIdx << "DONE" << std::endl;

    }
  } else {
    for (int curPtIdx = 0; curPtIdx < points.size(); ++curPtIdx) {

      for (auto curCamIdx : visibility[curPtIdx]) {

        visFile << curCamIdx << " " << camCenters.size() + curPtIdx << std::endl;
      }
      //std::cout << "Point: " << curPtIdx << "DONE" << std::endl;

    }
  }
  visFile.close();
}
void saveVisibilityPly(const glm::vec3 &camCenters, const std::vector<glm::vec3> & points, const std::vector<int> & visibility, const std::string &name) {

  std::ofstream visFile(name + ".ply");
//
  int numVisRays = visibility.size();

  std::cout << "Writing initVis_" << std::endl;
  std::cout << "numVisRays_: " << numVisRays << std::endl;

  visFile << "ply" << std::endl << "format ascii 1.0" << std::endl;
  visFile << "element vertex " << points.size() + 1 << std::endl;
  visFile << "property float x " << std::endl << "property float y " << std::endl << "property float z " << std::endl;
  visFile << "element edge " << numVisRays << std::endl;
  visFile << "property int vertex1 " << std::endl << "property int vertex2" << std::endl;
  visFile << "end_header" << std::endl;

  visFile << camCenters.x << " " << camCenters.y << " " << camCenters.z << std::endl;

  for (auto curPt : points) {
    visFile << curPt.x << " " << curPt.y << " " << curPt.z << std::endl;
  }

  for (auto curPtIdx : visibility) {

    visFile << 0 << " " << 1 + curPtIdx << std::endl;
  }
  //  std::cout << "Cam: " << curCamIdx << "DONE" << std::endl;

  visFile.close();
}


int GaussNewton(const std::vector<cv::Mat> &cameras, const std::vector<cv::Point2f> &points, cv::Point3f init3Dpoint, cv::Point3f &optimizedPoint,
    int numIterations, float maxMse) {
  int numMeasures = points.size();
  cv::Mat r = cv::Mat(numMeasures * 2, 1, CV_32F);

  cv::Mat curEstimate3DPoint = cv::Mat(3, 1, CV_32F);
  cv::Mat curEstimate3DPointH = cv::Mat(4, 1, CV_32F);
  curEstimate3DPoint.at<float>(0, 0) = init3Dpoint.x;
  curEstimate3DPoint.at<float>(1, 0) = init3Dpoint.y;
  curEstimate3DPoint.at<float>(2, 0) = init3Dpoint.z;

  cv::Mat J, H;
  float last_mse = 0;
  int i;
  for (i = 0; i < numIterations; i++) {

    float mse = 0;
    //compute residuals
    for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
      curEstimate3DPointH.at<float>(0, 0) = curEstimate3DPoint.at<float>(0, 0);
      curEstimate3DPointH.at<float>(1, 0) = curEstimate3DPoint.at<float>(1, 0);
      curEstimate3DPointH.at<float>(2, 0) = curEstimate3DPoint.at<float>(2, 0);
      curEstimate3DPointH.at<float>(3, 0) = 1.0;
      cv::Mat cur2DpositionH = cameras[curMeas] * curEstimate3DPointH;

      r.at<float>(2 * curMeas, 0) = ((points[curMeas].x - cur2DpositionH.at<float>(0, 0) / cur2DpositionH.at<float>(2, 0)));
      mse += r.at<float>(2 * curMeas, 0) * r.at<float>(2 * curMeas, 0);

      r.at<float>(2 * curMeas + 1, 0) = ((points[curMeas].y - cur2DpositionH.at<float>(1, 0) / cur2DpositionH.at<float>(2, 0)));
      mse += r.at<float>(2 * curMeas + 1, 0) * r.at<float>(2 * curMeas + 1, 0);
#ifdef DEBUG_OPTIMIZATION
      if(i==0) {
        std::cout<<"CurMeas: "<<curMeas<<std::endl<<"curEstimate3DPointH="<< curEstimate3DPointH.t()<<std::endl;
        std::cout<<"CurCam"<<cameras[curMeas]<<std::endl;
        std::cout<<"cur2DpositionH: "<<cur2DpositionH.at<float>(0, 0)/cur2DpositionH.at<float>(2, 0)<<", "<<cur2DpositionH.at<float>(1, 0)/cur2DpositionH.at<float>(2, 0)<<std::endl;
        std::cout<<"points[curMeas]: "<<points[curMeas]<<std::endl;
        std::cout<<"residual on x: "<<r.at<float>(2 * curMeas, 0)<<std::endl;
        std::cout<<"residual on y: "<<r.at<float>(2 * curMeas + 1 , 0)<<std::endl;
        std::cout<<std::endl;}
#endif
    }
    //mse = sqrt(mse)/(numMeasures*2);

    if (abs(mse / (numMeasures * 2) - last_mse) < 0.0000000005) {
      break;
    }
    last_mse = mse / (numMeasures * 2);

    if (point2D3DJacobian(cameras, curEstimate3DPoint, J, H) == -1) {
      //std::cout<<"It= "<<" point2D3DJacobian(cameras, curEstimate3DPoint, J, H) == -1 "<<std::endl;
      return -1;
    }
#ifdef DEBUG_OPTIMIZATION_VERBOSE
    std::cout<<"J: "<<J<<std::endl;
    std::cout<<"H: "<<H<<std::endl;
#endif

    curEstimate3DPoint += H.inv() * J.t() * r;

#ifdef DEBUG_OPTIMIZATION
    std::cout << "It= " << i << " last_mse " << last_mse << std::endl;
#endif
  }

  if (last_mse < maxMse) {
    optimizedPoint.x = curEstimate3DPoint.at<float>(0, 0);
    optimizedPoint.y = curEstimate3DPoint.at<float>(1, 0);
    optimizedPoint.z = curEstimate3DPoint.at<float>(2, 0);
    return 1;
  } else {
    return -1;
  }
}

int point2D3DJacobian(const std::vector<cv::Mat> &cameras, const cv::Mat &cur3Dpoint, cv::Mat &J, cv::Mat &hessian) {

  int numMeasures = cameras.size();
  cv::Mat cur3DPointHomog = cv::Mat(4, 1, CV_32F);
  ;
  cur3DPointHomog.at<float>(0, 0) = cur3Dpoint.at<float>(0, 0);
  cur3DPointHomog.at<float>(1, 0) = cur3Dpoint.at<float>(1, 0);
  cur3DPointHomog.at<float>(2, 0) = cur3Dpoint.at<float>(2, 0);
  cur3DPointHomog.at<float>(3, 0) = 1.0;

  J = cv::Mat(2 * numMeasures, 3, CV_32FC1);  //2 rows for each point: one for x, the other for y
  hessian = cv::Mat(3, 3, CV_32FC1);

  /*std::cout << "gdevre" <<std::endl;
   std::cout << cameras[0] <<std::endl;*/
  for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
    cv::Mat curReproj = cameras[curMeas] * cur3DPointHomog;
    float xH = curReproj.at<float>(0, 0);
    float yH = curReproj.at<float>(1, 0);
    float zH = curReproj.at<float>(2, 0);
    float p00 = cameras[curMeas].at<float>(0, 0);
    float p01 = cameras[curMeas].at<float>(0, 1);
    float p02 = cameras[curMeas].at<float>(0, 2);
    float p10 = cameras[curMeas].at<float>(1, 0);
    float p11 = cameras[curMeas].at<float>(1, 1);
    float p12 = cameras[curMeas].at<float>(1, 2);
    float p20 = cameras[curMeas].at<float>(2, 0);
    float p21 = cameras[curMeas].at<float>(2, 1);
    float p22 = cameras[curMeas].at<float>(2, 2);

    //d(P*X3D)/dX
    J.at<float>(2 * curMeas, 0) = (p00 * zH - p20 * xH) / (zH * zH);
    J.at<float>(2 * curMeas + 1, 0) = (p10 * zH - p20 * yH) / (zH * zH);

    //d(P*X3D)/dY
    J.at<float>(2 * curMeas, 1) = (p01 * zH - p21 * xH) / (zH * zH);
    J.at<float>(2 * curMeas + 1, 1) = (p11 * zH - p21 * yH) / (zH * zH);

    //d(P*X3D)/dZ
    J.at<float>(2 * curMeas, 2) = (p02 * zH - p22 * xH) / (zH * zH);
    J.at<float>(2 * curMeas + 1, 2) = (p12 * zH - p22 * yH) / (zH * zH);
  }

  hessian = J.t() * J;
  float d;
  d = cv::determinant(hessian);
  if (d < 0.0000000001) {
    //printf("doh");
    return -1;
  } else {
    return 1;
  }
}

glm::vec3 toGlm(const Point & p){
  return glm::vec3(p.x(), p.y(), p.z());
}
glm::vec3 toGlm(const Vertex_handle & p){
  return glm::vec3(p->point().x(), p->point().y(), p->point().z());
}


void convertToMvp(photometricGradient::CameraType &cam, glm::mat4 &mvpOut) {

  glm::mat4 modelViewMatrix, projectionMatrix;
  computeModelViewMatrix(cam.rotation, cam.translation, modelViewMatrix);
  computeProjectionMatrix(cam.intrinsics, cam.imageHeight, cam.imageWidth, projectionMatrix);
  mvpOut = projectionMatrix * modelViewMatrix;
}

void computeProjectionMatrix(glm::mat3 &intrinsics, int h, int w, glm::mat4 &projectionMatrixOut) {

  float N = 0.10;
  float F = 1000.0;

  glm::mat4 persp = glm::mat4(0.0);
  persp[0][0] = intrinsics[0][0];
  persp[0][1] = 0.0;
  persp[0][2] = intrinsics[0][2];
  persp[1][1] = intrinsics[1][1];
  persp[1][2] = intrinsics[1][2];
  persp[2][2] = -(N + F);
  persp[2][3] = N * F;
  persp[3][2] = 1.0;

  double L = 0;
  double R = w;
  double B = 0;
  double T = h;
  glm::mat4 ortho = glm::mat4(0.0);

  int dino = 1;/*to change the y sign if dino file format is used*/
  ortho[0][0] = 2.0 / (R - L);
  ortho[0][3] = -(R + L) / (R - L);
  ortho[1][1] = (1 - 2 * dino) * 2.0 / (T - B);
  ortho[1][3] = -(1 - 2 * dino) * (T + B) / (T - B);
  ortho[2][2] = (-2.0 / (F - N));
  ortho[2][3] = (-(F + N) / (F - N));
  ortho[3][3] = 1.0;
  projectionMatrixOut = glm::transpose(persp * ortho);
}

void computeModelViewMatrix(glm::mat3 &rotation, glm::vec3 &translation, glm::mat4 &modelViewMatrixOut) {

  glm::mat4 modelMatrix = glm::mat4();
  glm::mat4 viewMatrix = glm::mat4();

  viewMatrix[0][0] = rotation[0][0];
  viewMatrix[0][1] = rotation[1][0];
  viewMatrix[0][2] = rotation[2][0];
  viewMatrix[1][0] = rotation[0][1];
  viewMatrix[1][1] = rotation[1][1];
  viewMatrix[1][2] = rotation[2][1];
  viewMatrix[2][0] = rotation[0][2];
  viewMatrix[2][1] = rotation[1][2];
  viewMatrix[2][2] = rotation[2][2];
  viewMatrix[3][0] = translation.x;
  viewMatrix[3][1] = translation.y;
  viewMatrix[3][2] = translation.z;
  viewMatrix[0][3] = 0.0;
  viewMatrix[1][3] = 0.0;
  viewMatrix[2][3] = 0.0;
  viewMatrix[3][3] = 1.0;

  modelViewMatrixOut = viewMatrix * modelMatrix;
}
void outlierFilteringAndInlierOptimize(SfMData &sfm_data, std::vector<bool>& inliers, int numIterations, float maxMse) {

  outlierFilteringAndInlierOptimize(sfm_data.points_, sfm_data.camViewingPointN_, sfm_data.camerasList_, sfm_data.point2DoncamViewingPoint_, inliers,
      numIterations, maxMse);
}

void outlierFilteringAndInlierOptimize(std::shared_ptr<SfMData>  &sfm_data, std::vector<bool>& inliers, int numIterations, float maxMse) {

  outlierFilteringAndInlierOptimize(sfm_data->points_, sfm_data->camViewingPointN_, sfm_data->camerasList_, sfm_data->point2DoncamViewingPoint_, inliers,
      numIterations, maxMse);
}

void outlierFilteringAndInlierOptimize(std::vector<glm::vec3>& points_, const std::vector<std::vector<int> > &camViewingPointN_,
    const std::vector<photometricGradient::CameraType> &camerasList_, const std::vector<std::vector<glm::vec2> > &point2DoncamViewingPoint_, std::vector<bool>& inliers,
    int numIterations, float maxMse) {

  inliers.assign(points_.size(), false);
  std::vector<cv::Mat> cameras;
  std::vector<cv::Point2f> measures;
  cv::Point3f init3Dpoint;
  cv::Point3f optimizedPoint;

  for (int curPt3D = 0; curPt3D < points_.size(); curPt3D++) {
    cameras.clear();
    cameras.assign(camViewingPointN_[curPt3D].size(), cv::Mat());
    for (int curC = 0; curC < camViewingPointN_[curPt3D].size(); curC++) {
      cameras[curC] = cv::Mat(4, 4, CV_32F);
      for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
          cameras[curC].at<float>(row, col) = camerasList_[camViewingPointN_[curPt3D][curC]].cameraMatrix[row][col];
        }
      }

    }

    measures.clear();
    measures.assign(point2DoncamViewingPoint_[curPt3D].size(), cv::Point2f());
    for (int curMeas = 0; curMeas < point2DoncamViewingPoint_[curPt3D].size(); curMeas++) {
      measures[curMeas].x = point2DoncamViewingPoint_[curPt3D][curMeas].x;
      measures[curMeas].y = point2DoncamViewingPoint_[curPt3D][curMeas].y;
    }

    init3Dpoint.x = points_[curPt3D].x;
    init3Dpoint.y = points_[curPt3D].y;
    init3Dpoint.z = points_[curPt3D].z;

    if (utilities::GaussNewton(cameras, measures, init3Dpoint, optimizedPoint, numIterations, maxMse) != -1 && point2DoncamViewingPoint_[curPt3D].size() > 2) {

      points_[curPt3D].x = optimizedPoint.x;
      points_[curPt3D].y = optimizedPoint.y;
      points_[curPt3D].z = optimizedPoint.z;
      inliers[curPt3D] = true;
    }
  }

}

void convertToMvp2(photometricGradient::CameraType &cam, glm::mat4 &mvpOut) {

  glm::mat4 modelViewMatrix, projectionMatrix;
  computeModelViewMatrix2(cam.rotation, cam.translation, modelViewMatrix);
  computeProjectionMatrix2(cam.intrinsics, cam.imageHeight, cam.imageWidth, projectionMatrix);
  mvpOut = projectionMatrix * modelViewMatrix;
}

void computeProjectionMatrix2(glm::mat3 &intrinsics, int h, int w, glm::mat4 &projectionMatrixOut) {

  float N = 0.001;
  float F = 100.0;

  glm::mat4 persp = glm::mat4(0.0);
  persp[0][0] = intrinsics[0][0];
  persp[0][1] = 0.0;
  persp[0][2] = intrinsics[0][2];
  persp[1][1] = intrinsics[1][1];
  persp[1][2] = intrinsics[1][2];
  persp[2][2] = -(N + F);
  persp[2][3] = N * F;
  persp[3][2] = 1.0;

  double L = 0;
  double R = w;
  double B = 0;
  double T = h;
  glm::mat4 ortho = glm::mat4(0.0);

  int dino = 1;/*to change the y sign if dino file format is used*/
  ortho[0][0] = 2.0 / (R - L);
  ortho[0][3] = -(R + L) / (R - L);
  ortho[1][1] = (1 - 2 * dino) * 2.0 / (T - B);
  ortho[1][3] = -(1 - 2 * dino) * (T + B) / (T - B);
  ortho[2][2] = (-2.0 / (F - N));
  ortho[2][3] = (-(F + N) / (F - N));
  ortho[3][3] = 1.0;
  projectionMatrixOut = glm::transpose(persp * ortho);
}

void computeModelViewMatrix2(glm::mat3 &rotation, glm::vec3 &translation, glm::mat4 &modelViewMatrixOut) {

  glm::mat4 modelMatrix = glm::mat4();
  glm::mat4 viewMatrix = glm::mat4();

  viewMatrix[0][0] = rotation[0][0];
  viewMatrix[0][1] = rotation[1][0];
  viewMatrix[0][2] = rotation[2][0];
  viewMatrix[1][0] = rotation[0][1];
  viewMatrix[1][1] = rotation[1][1];
  viewMatrix[1][2] = rotation[2][1];
  viewMatrix[2][0] = rotation[0][2];
  viewMatrix[2][1] = rotation[1][2];
  viewMatrix[2][2] = rotation[2][2];
  viewMatrix[3][0] = translation.x;
  viewMatrix[3][1] = translation.y;
  viewMatrix[3][2] = translation.z;
  viewMatrix[0][3] = 0.0;
  viewMatrix[1][3] = 0.0;
  viewMatrix[2][3] = 0.0;
  viewMatrix[3][3] = 1.0;

  modelViewMatrixOut = viewMatrix * modelMatrix;
}

void computeExtrinsicsHZ(const glm::mat3& rotation, const glm::vec3& traslation, glm::mat4& extrinsics) {
  extrinsics[0][0] = rotation[0][0];
  extrinsics[0][1] = rotation[0][1];
  extrinsics[0][2] = rotation[0][2];
  extrinsics[1][0] = rotation[1][0];
  extrinsics[1][1] = rotation[1][1];
  extrinsics[1][2] = rotation[1][2];
  extrinsics[2][0] = rotation[2][0];
  extrinsics[2][1] = rotation[2][1];
  extrinsics[2][2] = rotation[2][2];
  extrinsics[0][3] = traslation[0];
  extrinsics[1][3] = traslation[1];
  extrinsics[2][3] = traslation[2];
  extrinsics[3][3] = 1.0;
}
}


