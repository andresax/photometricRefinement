#ifndef CONFIGURATOR_H_
#define CONFIGURATOR_H_


#include <string>
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <photometricGradient/types.hpp>
#include <misc/typesRefinement.hpp>
#include <misc/utilities.hpp>

class Configurator {
  public:
    Configurator(const std::string &path);
    virtual ~Configurator();
    PhotometricRefinementConfiguration parseConfigFile();

  private:
    void createDirRes();

    PhotometricRefinementConfiguration c;
    std::ifstream file_;
};

#endif /* CONFIGURATOR_H_ */
