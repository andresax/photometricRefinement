#include <Configurator.h>
#include <utilities.hpp>
#include <boost/filesystem.hpp>

Configurator::Configurator(const std::string &path) {
  file_.open(path.c_str());
  if (file_.is_open())
    std::cout << path << " opened" << std::endl;
  else
    std::cout << "Error reading " << path << std::endl;
}

Configurator::~Configurator() {
}

PhotometricRefinementConfiguration Configurator::parseConfigFile() {
  std::cout << "Configurator::parseConfigFile ... " << std::flush;
  utilities::readLineAndStore(file_, c.fileOpenMVG_);
  utilities::readLineAndStore(file_, c.pathInitMesh_);
  //utilities::readLineAndStore(file_, c.pathOutputDir_);
  utilities::readLineAndStore(file_, c.lambdaPhoto_);
  utilities::readLineAndStore(file_, c.lambdaSmooth_);
  utilities::readLineAndStore(file_, c.maxDistanceCamFeatureRef);
  utilities::readLineAndStore(file_, c.numIt_);
  utilities::readLineAndStore(file_, c.window_NCC_);
  utilities::readLineAndStore(file_, c.ensureedgemax_);
  utilities::readLineAndStore(file_, c.ensureedgeit_);
  utilities::readLineAndStore(file_, c.optionalNameDir_);

  createDirRes();
  std::cout << "DONE" << std::endl;
  return c;
}

void Configurator::createDirRes() {

  std::cout << "Configurator::createDirRes  ..." << std::flush;
  std::stringstream s;
  std::string base = "Results/";
  s << base << "Incremental_" << c.optionalNameDir_ << "_" << "_" << c.lambdaPhoto_ << "_"
      << c.lambdaSmooth_ << "_" << c.maxDistanceCamFeatureRef << "_" << c.numIt_ << "_"
      << c.ensureedgemax_ << "_" << c.ensureedgeit_<<"/";

  boost::filesystem::path dirbase(base.c_str());
  boost::filesystem::create_directories(dirbase);

  boost::filesystem::path dir(s.str().c_str());
  boost::filesystem::create_directories(dir);

  c.pathOutputDir_ = s.str();

  std::cout << "DONE" << std::endl;
}
