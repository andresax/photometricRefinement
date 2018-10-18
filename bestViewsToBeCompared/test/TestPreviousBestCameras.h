
#ifndef TEST_PREVBESTCAM_H_
#define TEST_PREVBESTCAM_H_

#include <memory>
#include <typesRefinement.hpp>

class TestPreviousBestCameras {
public:
  TestPreviousBestCameras();
  virtual ~TestPreviousBestCameras()=default;
  void run();
private:
  std::shared_ptr<SfMData>  sfm_data_;
};

#endif