#ifndef INCLUDE_SUBDIVIDER_H_
#define INCLUDE_SUBDIVIDER_H_


#include <glm.hpp>


class Subdivider {
public:
  Subdivider();
  Subdivider(int areaMax, int numIt);

  virtual ~Subdivider();
  void setAreaMax(int areaMax) {
    areaMax_ = areaMax;
  }

  void setNumIt(int numIt) {
    numIt_ = numIt;
  }
private:
  float orientPoint(const glm::vec2 &v0, const glm::vec2 &v1, const glm::vec2 &p);
  int areaMax_, numIt_;
};

#endif /* INCLUDE_SUBDIVIDER_H_ */
