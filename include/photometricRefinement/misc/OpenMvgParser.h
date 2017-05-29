#ifndef CAM_PARSERS_OPENMVGPARSER_H_
#define CAM_PARSERS_OPENMVGPARSER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <typesRefinement.hpp>
#include <types.hpp>
#include <rapidjson/document.h>


class OpenMvgParser {
public:
  OpenMvgParser(std::string path);
  OpenMvgParser();
  virtual ~OpenMvgParser();
  virtual void parse();

  const SfMData& getSfmData() const {
    return sfm_data_;
  }

  void setFileName(const std::string& fileName) {
    fileName_ = fileName;
    fileStream_.open(fileName_.c_str());
  }

private:
  void parseViews(const std::map<int,glm::mat3> & intrinsics, const std::map<int, glm::vec3> &distortion, const std::map<int,photometricGradient::CameraType> & extrinsics);
  void parseIntrinsics(std::map<int,glm::mat3> & intrinsics, std::map<int, glm::vec3> &distortion);
  void parseExtrinsics(std::map<int,photometricGradient::CameraType> & extrinsics);
  void parsePoints();


  rapidjson::Document document_;
  std::string fileName_;
  std::ifstream fileStream_;
  SfMData sfm_data_;

};

#endif /* CAM_PARSERS_OPENMVGPARSER_H_ */
