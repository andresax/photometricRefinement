#include <PhotometricRefinement.h>
#include <OpenMvgParser.h>
#include <Configurator.h>
#include <types.hpp>

PhotometricRefinement::PhotometricRefinement(std::string path) {
  Configurator con(path);
  conf_ = con.parseConfigFile();
  OpenMvgParser op(conf_.fileOpenMVG_);
  op.parse();
  sfm_data_ = std::make_shared<SfMData>(op.getSfmData());
  surfaceEvolver_.initSurfaceEvolver(conf_.refinementConfig_, sfm_data_);
}

PhotometricRefinement::~PhotometricRefinement() {
}

void PhotometricRefinement::run() {
  surfaceEvolver_.beginEvolver();
  surfaceEvolver_.refine();
}

