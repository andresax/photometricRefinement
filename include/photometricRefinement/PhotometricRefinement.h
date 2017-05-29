#ifndef PhotometricRefinement_H_
#define PhotometricRefinement_H_

#include <types.hpp>
#include <glm.hpp>
#include <SurfaceEvolver.h>
#include <string>

class PhotometricRefinement {
public:
  PhotometricRefinement(std::string pathConfigFile);
  virtual ~PhotometricRefinement();

  void run();
private:
  std::shared_ptr<SfMData>  sfm_data_; 

  std::vector<int> numFrame;
  SurfaceEvolver surfaceEvolver_;
  ConfigurationSemanticReconstruction conf_;

};

#endif /* PhotometricRefinement_H_ */
