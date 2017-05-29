#ifndef UTILITIESPHOTOMETRICGRADIENT_HPP_
#define UTILITIESPHOTOMETRICGRADIENT_HPP_

#include <opencv2/core/core.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <glm.hpp>
#include <string>
#include <typesRefinement.hpp>
#include <types.hpp>
#include <Mesh.h>

namespace utilities {


void computeExtrinsicsHZ(const glm::mat3 &rotation, const glm::vec3 &traslation, glm::mat4 &extrinsics);

void convertToMvp2(photometricGradient::CameraType &cam, glm::mat4 &mvpOut);
void computeProjectionMatrix2(glm::mat3 &intrinsics, int h, int w, glm::mat4 &projectionMatrixOut);
void computeModelViewMatrix2(glm::mat3 &rotation, glm::vec3 &translation, glm::mat4 &modelViewMatrixOut);
glm::vec2 projectPoint(const glm::mat4 &cameraMatrix, const glm::vec3 &pt);

//compute the string representing the frame number with a specified padding
std::string getFrameNumber(int curFrame, int digitIdxLength);

//write an OB files with only vertices
void writeObj(std::vector<glm::vec3> &vertices, std::string path);

/**
 * Print on standard output the matrix specified
 */
void printMatrix(glm::mat4 matrix);
void printMatrix(std::string message, glm::mat4 matrix);
void printMatrix(glm::mat3 matrix);
void printMatrix(std::string message, glm::mat3 matrix);
void printMatrix(glm::vec3 vector);
void printMatrix(std::string message, glm::vec3 vector);
void printMatrix(glm::vec4 vector);
void printMatrix(std::string message, glm::vec4 vector);

/*Stores the visibility rays in a ply file named initVisT.ply
 *
 *
 * param camCenters list of camera centers in world coordinate
 * param points list of points centers in camera coordinates
 * pointsVisibleFromCamN visibility information: vector of i int vectors; each i-th int vectors is the list
 *                       of points indices  visible from th ei-th cam
 */
void saveVisibilityPly(const std::vector<glm::vec3> &camCenters, const std::vector<glm::vec3> & points, const std::vector<std::vector<int> >& visibility,
    const std::string &name = "visibility", bool pointsVisibleFromCamN = true);

void saveVisibilityPly(const glm::vec3 &camCenters, const std::vector<glm::vec3> & points, const std::vector<int>& visibility, const std::string &name =
    "visibilityCam");

void saveVisibilityPly(const SfMData &sfm_data, const std::string &name = "visibility");

/*computes rotation around X axis*/
glm::mat3 rotX(float alpha);

/*computes rotation around Y axis*/
glm::mat3 rotY(float alpha);

/*computes rotation around Z axis*/
glm::mat3 rotZ(float alpha);


glm::vec3 toGlm(const Point & p);
glm::vec3 toGlm(const Vertex_handle & p);

void convertToMvp(photometricGradient::CameraType &cam, glm::mat4 &mvpOut);
void computeProjectionMatrix(glm::mat3 &intrinsics, int h, int w, glm::mat4 &projectionMatrixOut);
void computeModelViewMatrix(glm::mat3 &rotation, glm::vec3 &translation, glm::mat4 &modelViewMatrixOut);

/**
 * Read one line in the configuration file and stores the parameter in the value variable
 */
void readLineAndStore(std::ifstream &configFile, bool &value);
void readLineAndStore(std::ifstream &configFile, int &value);
void readLineAndStore(std::ifstream &configFile, double &value);
void readLineAndStore(std::ifstream &configFile, float &value);
void readLineAndStore(std::ifstream &configFile, std::string &value);


/*
 * Gauss Newton to optimize the 3D point position of the 3D point optimizedPoint
 * */
int GaussNewton(const std::vector<cv::Mat> &cameras, const std::vector<cv::Point2f> &points, cv::Point3f init3Dpoint, cv::Point3f &optimizedPoint,
    int numIterations = 30, float maxMse = 4.0);

int point2D3DJacobian(const std::vector<cv::Mat> &cameras, const cv::Mat &cur3Dpoint, cv::Mat &jacobian, cv::Mat &hessian);

void outlierFilteringAndInlierOptimize(SfMData &sfm_data, std::vector<bool>& inliers, int numIterations = 30, float maxMse = 4.0);
void outlierFilteringAndInlierOptimize(std::shared_ptr<SfMData>  &sfm_data, std::vector<bool>& inliers, int numIterations = 30, float maxMse = 4.0);
void outlierFilteringAndInlierOptimize(std::vector<glm::vec3>& points_, const std::vector<std::vector<int> > &camViewingPointN_,
    const std::vector<photometricGradient::CameraType> &camerasList_, const std::vector<std::vector<glm::vec2> > &point2DoncamViewingPoint_, std::vector<bool>& inliers,
    int numIterations = 30, float maxMse = 4.0);

}  // namespace utils

#endif /* UTILITIES_HPP_ */
