#include "linearalg.hpp"

float cross(const la::Vector &a, const la::Vector &b) {
  return a.x * b.y - a.y * b.x;
}
float dot(const la::Vector &a, const la::Vector &b) {
  return a.x * b.x + a.y * b.y;
}
float absDot(const la::Vector &a, const la::Vector &b) {
  return std::abs(dot(a, b));
}

la::Vector perpendicular(const la::Vector &a) { return {-a.y, a.x}; }

la::Vector elementViseMultipication(const la::Vector &vectorA,
                                    const la::Vector &vectorB) {
  return {vectorA.x * vectorB.x, vectorA.y * vectorB.y};
}
float magnitude(const la::Vector &a) { return std::sqrt(dot(a, a)); }
la::Vector normalise(la::Vector a) {
  float mag = magnitude(a);
  if (mag == 0.F) {
    return {0.F, 0.F};
  }
  a.x /= mag;
  a.y /= mag;
  return a;
}
la::Vector rotate(const la::Vector &vectorToRotate, float angle) {
  la::Vector rotatedVector;
  rotatedVector.x =
      vectorToRotate.x * std::cos(angle) - vectorToRotate.y * std::sin(angle);
  rotatedVector.y =
      vectorToRotate.x * std::sin(angle) + vectorToRotate.y * std::cos(angle);
  return rotatedVector;
}
std::array<la::Vector, 2> getBaseCoordinateSystem(float orientationOfOrigin) {
  std::array<la::Vector, 2> baseVector;
  baseVector[0] = {std::cos(orientationOfOrigin),
                   std::sin(orientationOfOrigin)};
  baseVector[1] = {-std::sin(orientationOfOrigin),
                   std::cos(orientationOfOrigin)};

  return baseVector;
}
la::Vector transformToCordinateSystem(const la::Vector &vectorToTransformed,
                                      const la::Vector &positionOfOrigin,
                                      float orientationOfOrigin) {
  std::array<la::Vector, 2> baseVector;
  baseVector[0] = {std::cos(orientationOfOrigin),
                   std::sin(orientationOfOrigin)};
  baseVector[1] = {-std::sin(orientationOfOrigin),
                   std::cos(orientationOfOrigin)};
  la::Vector transformedVector;
  transformedVector.x =
      dot(vectorToTransformed - positionOfOrigin, baseVector[0]);
  transformedVector.y =
      dot(vectorToTransformed - positionOfOrigin, baseVector[1]);
  return transformedVector;
}

la::Vector
inverseTransformToCordinateSystem(const la::Vector &vectorToTransformed,
                                  const la::Vector &positionOfOrigin,
                                  float orientationOfOrigin) {
  std::array<la::Vector, 2> baseVector;
  baseVector[0] = {std::cos(-orientationOfOrigin),
                   std::sin(-orientationOfOrigin)};
  baseVector[1] = {-std::sin(-orientationOfOrigin),
                   std::cos(-orientationOfOrigin)};
  la::Vector transformedVector;
  transformedVector.x =
      dot(vectorToTransformed, baseVector[0]) + positionOfOrigin.x;
  transformedVector.y =
      dot(vectorToTransformed, baseVector[1]) + positionOfOrigin.y;
  return transformedVector;
}
la::Vector tripleproduct(const la::Vector &a, const la::Vector &b,
                         const la::Vector &c) {

  return b * dot(c, a) - a * dot(b, c);
}

la::Vector computeIntersection(const std::array<la::Vector, 2> &a,
                               const std::array<la::Vector, 2> &b) {

  // Line AB represented as a1x + b1y = c1
  double a1 = a[1].y - a[0].y;
  double b1 = a[0].x - a[1].x;
  double c1 = a1 * (a[0].x) + b1 * (a[0].y);

  // Line CD represented as a2x + b2y = c2
  double a2 = b[1].y - b[0].y;
  double b2 = b[0].x - b[1].x;
  double c2 = a2 * (b[0].x) + b2 * (b[0].y);

  double determinant = a1 * b2 - a2 * b1;

  if (determinant == 0) {
    // The lines are parallel. This is simplified
    // by returning a pair of FLT_MAX
    return {std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max()};
  } else {
    float x = (b2 * c1 - b1 * c2) / determinant;
    float y = (a1 * c2 - a2 * c1) / determinant;
    return {x, y};
  }
}
la::Vector computeIntersection(const std::vector<la::Vector> &a,
                               const std::array<la::Vector, 2> &b) {

  // Line AB represented as a1x + b1y = c1
  float a1 = a[1].y - a[0].y;
  float b1 = a[0].x - a[1].x;
  float c1 = a1 * (a[0].x) + b1 * (a[0].y);

  // Line CD represented as a2x + b2y = c2
  float a2 = b[1].y - b[0].y;
  float b2 = b[0].x - b[1].x;
  float c2 = a2 * (b[0].x) + b2 * (b[0].y);

  float determinant = a1 * b2 - a2 * b1;

  if (determinant == 0) {
    // The lines are parallel. This is simplified
    // by returning a pair of FLT_MAX
    return {std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max()};
  } else {
    float x = (b2 * c1 - b1 * c2) / determinant;
    float y = (a1 * c2 - a2 * c1) / determinant;
    return {x, y};
  }
}
