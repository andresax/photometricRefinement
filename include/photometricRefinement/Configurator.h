#ifndef CONFIGURATOR_H_
#define CONFIGURATOR_H_

#include <types.hpp>

#include <typesRefinement.hpp>
#include <string>
#include <fstream>
#include <iostream>

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
