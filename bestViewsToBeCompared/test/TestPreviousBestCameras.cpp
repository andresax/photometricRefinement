#include <TestPreviousBestCameras.h>
#include <CameraChooser.h>
#include <OpenMvgParser.h>
#include <Mesh.h>

TestPreviousBestCameras::TestPreviousBestCameras() {

}

void TestPreviousBestCameras::run() {
  std::shared_ptr<Mesh> m = std::make_shared<Mesh>();

  m->loadFormat("/home/andrea/workspaceC/incrementalMVS/Mesh_Semantic_nocolor.off", false);
  
  OpenMvgParser op("data/castle/castle.off");
  op.parse();
  sfm_data_ = std::make_shared<SfMData>(op.getSfmData());
  std::vector<int> out;
  CameraChooser camCh(sfm_data_->imageWidth_, sfm_data_->imageHeight_);
  camCh.setMesh(m);
  camCh.getKNear(sfm_data_->camerasList_, 5, 2, out);

}
