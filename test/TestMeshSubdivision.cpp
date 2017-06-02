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
  for(int i=0;i<10;i++){
  ms.subdivide(mesh_, camera_.cameraMatrix);
  showMeshOnCamera();
  }
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
  cv::Mat rendering = cv::Mat(camera_.imageWidth, camera_.imageHeight, CV_8UC3);

  typedef MeshSurface::Halfedge_index he_descriptor;
  typedef MeshSurface::Vertex_index vertex_descriptor;
  //MeshSurface::Property_map<he_descriptor, Kernel::Point_3> location = mesh_.ha;
  MeshSurface::Property_map<vertex_descriptor, Ker::Point_3> location = mesh_.points();
  MeshSurface::Property_map<he_descriptor, Ker::Point_3> heValMap;

  for (vertex_descriptor heIt : mesh_.vertices()) {
//  for (Halfedge_iterator heIt = mesh_.p.halfedges_begin(); heIt != mesh_.p.halfedges_end(); heIt++) {
//    glm::vec4 p1_3d = glm::vec4(heIt->vertex()->point().x(), heIt->vertex()->point().y(),heIt->vertex()->point().z(), 1.0);
//        glm::vec4 p2_3d = glm::vec4(heIt->opposite()->vertex()->point().x(), heIt->opposite()->vertex()->point().y(),heIt->opposite()->vertex()->point().z(), 1.0);
    glm::vec4 p1_3d = glm::vec4(location[heIt].x(), location[heIt].y(), location[heIt].z(), 1.0);
    glm::vec4 p2_3d = glm::vec4(location[heIt].x(), location[heIt].y(), location[heIt].z(), 1.0);

    glm::vec4 pt1_2dH = p1_3d * camera_.cameraMatrix;
    glm::vec4 pt2_2dH = p2_3d * camera_.cameraMatrix;

    glm::vec2 pt1_2d = glm::vec2(pt1_2dH.x / pt1_2dH.z, pt1_2dH.y / pt1_2dH.z);
    glm::vec2 pt2_2d = glm::vec2(pt2_2dH.x / pt2_2dH.z, pt2_2dH.y / pt2_2dH.z);

    cv::Point2d p1 = cv::Point2d(pt1_2d.x, pt1_2d.y);
    cv::Point2d p2 = cv::Point2d(pt2_2d.x, pt2_2d.y);

    cv::line(rendering, p1, p2, cv::Scalar(255, 255, 255, 0), 1);
    //std::cout << p1 << std::endl;

  }
  cv::imshow("Rendering", rendering);
  cv::waitKey();

}
