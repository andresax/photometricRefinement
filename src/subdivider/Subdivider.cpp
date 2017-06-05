
#include <Subdivider.h>

Subdivider::Subdivider() {
  areaMax_ = 16;
  numIt_ = 1;
}
Subdivider::Subdivider(int areaMax, int numIt){
  areaMax_ = areaMax;
  numIt_ = numIt;

}



float Subdivider::orientPoint(const glm::vec2& v0, const glm::vec2& v1, const glm::vec2& p) {
  glm::mat2 m;
  m[0][0] = (v1.x - v0.x);
  m[0][1] = (p.x - v0.x);
  m[1][0] = (v1.y - v0.y);
  m[1][1] = (p.y - v0.y);

  return m[0][0] * m[1][1] - m[0][1] * m[1][0];
}
