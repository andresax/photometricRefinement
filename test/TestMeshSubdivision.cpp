#include <TestMeshSubdivision.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <glm.hpp>
#include <utilities.hpp>
#include <MeshSubdivider.h>

TestMeshSubdivision::TestMeshSubdivision() {

}

TestMeshSubdivision::~TestMeshSubdivision() {
}

void TestMeshSubdivision::testMeshSubdivision() {
  createMesh();
  createCamera();

  MeshSubdivider ms;
  for (int i = 0; i < 10; i++) {
    showMeshOnCamera();
    ms.subdivide(mesh_, camera_.cameraMatrix);
  }
  showMeshOnCamera();
}

void TestMeshSubdivision::createMesh() {
  //mesh_.loadFormat("/home/andrea/workspaceC/plane.off", false);

  std::ifstream input("/home/andrea/workspaceC/plane.off");
  Mesh mesh;
  if (!input || !(input >> mesh_)) {
    std::cerr << "Not a valid off file." << std::endl;
  }
}

void TestMeshSubdivision::createCamera() {
  std::string camParam = "400 0 510 0 400 510 0 0 1 -0.939693 0 0.34202 0 1 0 -0.34202 0 -0.939693 0.170492 -0 1.28586";
  //std::cout<<"tempCamera.cameraMatrix "<<iss.str()<<std::endl;
  std::stringstream s(camParam);

  s >> camera_.intrinsics[0][0] >> camera_.intrinsics[0][1] >> camera_.intrinsics[0][2] >> camera_.intrinsics[1][0] >> camera_.intrinsics[1][1]
      >> camera_.intrinsics[1][2] >> camera_.intrinsics[2][0] >> camera_.intrinsics[2][1] >> camera_.intrinsics[2][2] >> camera_.rotation[0][0]
      >> camera_.rotation[0][1] >> camera_.rotation[0][2] >> camera_.rotation[1][0] >> camera_.rotation[1][1] >> camera_.rotation[1][2]
      >> camera_.rotation[2][0] >> camera_.rotation[2][1] >> camera_.rotation[2][2] >> camera_.translation[0] >> camera_.translation[1]
      >> camera_.translation[2];

  glm::mat4 tempCameraExtrinsic(0.0);
  tempCameraExtrinsic[0][0] = camera_.rotation[0][0];
  tempCameraExtrinsic[0][1] = camera_.rotation[0][1];
  tempCameraExtrinsic[0][2] = camera_.rotation[0][2];
  tempCameraExtrinsic[1][0] = camera_.rotation[1][0];
  tempCameraExtrinsic[1][1] = camera_.rotation[1][1];
  tempCameraExtrinsic[1][2] = camera_.rotation[1][2];
  tempCameraExtrinsic[2][0] = camera_.rotation[2][0];
  tempCameraExtrinsic[2][1] = camera_.rotation[2][1];
  tempCameraExtrinsic[2][2] = camera_.rotation[2][2];
  tempCameraExtrinsic[0][3] = camera_.translation[0];
  tempCameraExtrinsic[1][3] = camera_.translation[1];
  tempCameraExtrinsic[2][3] = camera_.translation[2];

  glm::mat4 tempCameraIntrinsicH(0.0);
  tempCameraIntrinsicH[0][0] = camera_.intrinsics[0][0];
  tempCameraIntrinsicH[0][1] = camera_.intrinsics[0][1];
  tempCameraIntrinsicH[0][2] = camera_.intrinsics[0][2];
  tempCameraIntrinsicH[1][0] = camera_.intrinsics[1][0];
  tempCameraIntrinsicH[1][1] = camera_.intrinsics[1][1];
  tempCameraIntrinsicH[1][2] = camera_.intrinsics[1][2];
  tempCameraIntrinsicH[2][0] = camera_.intrinsics[2][0];
  tempCameraIntrinsicH[2][1] = camera_.intrinsics[2][1];
  tempCameraIntrinsicH[2][2] = camera_.intrinsics[2][2];

  camera_.cameraMatrix = glm::transpose(glm::transpose(tempCameraIntrinsicH) * glm::transpose(tempCameraExtrinsic));
  camera_.imageHeight = 1020;
  camera_.imageWidth = 1020;
  camera_.center = -camera_.translation * glm::transpose(camera_.rotation);
  utilities::convertToMvp2(camera_, camera_.mvp);

}

void TestMeshSubdivision::showMeshOnCamera() {
  cv::Mat rendering = cv::Mat::zeros(camera_.imageWidth, camera_.imageHeight, CV_8UC3);

  typedef MeshSurface::Vertex_index vertex_descriptor;
  //MeshSurface::Property_map<he_descriptor, Kernel::Point_3> location = mesh_.ha;
  MeshSurface::Property_map<vertex_descriptor, Ker::Point_3> location = mesh_.points();


  for (MeshSurface::Edge_index heIt : mesh_.edges()) {

    vertex_descriptor td = CGAL::target(heIt, mesh_);
    vertex_descriptor sd = CGAL::source(heIt, mesh_);

    glm::vec2 pt1_2d = utilities::projectPoint(camera_.cameraMatrix, glm::vec3(location[td].x(), location[td].y(), location[td].z()));
    glm::vec2 pt2_2d = utilities::projectPoint(camera_.cameraMatrix, glm::vec3(location[sd].x(), location[sd].y(), location[sd].z()));

    cv::Point2i p1 = cv::Point2i(static_cast<int>(pt1_2d.x), static_cast<int>(pt1_2d.y));
    cv::Point2i p2 = cv::Point2i(static_cast<int>(pt2_2d.x), static_cast<int>(pt2_2d.y));

    cv::line(rendering, p1, p2, cv::Scalar(255, 255, 255), 1);

    cv::imshow("Rendering", rendering);
    cv::waitKey();
  }
  cv::imshow("Rendering", rendering);
  cv::waitKey();
  rendering.release();

}
