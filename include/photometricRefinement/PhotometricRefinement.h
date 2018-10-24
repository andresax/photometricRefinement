#ifndef PhotometricRefinement_H_
#define PhotometricRefinement_H_

#include <string>
#include <glm/glm.hpp>
#include <photometricGradient/types.hpp>
#include <misc/typesRefinement.hpp>
#include <misc/OpenMvgParser.h>
#include <SurfaceEvolver.h>
#include <Configurator.h>

class PhotometricRefinement {
public:
  PhotometricRefinement(std::string pathConfigFile);
  virtual ~PhotometricRefinement();

  void run();
private:
  std::shared_ptr<SfMData>  sfm_data_; 

  std::vector<int> numFrame;
  SurfaceEvolver surfaceEvolver_;
  PhotometricRefinementConfiguration conf_;

};

#endif /* PhotometricRefinement_H_ */
