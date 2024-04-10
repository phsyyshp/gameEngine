#include "utils.hpp"
#include <SFML/System/Vector2.hpp>
float cross(const sf::Vector2f &a, const sf::Vector2f &b) {
  return a.x * b.y - a.y * b.x;
}
float dot(const sf::Vector2f &a, const sf::Vector2f &b) {
  return a.x * b.x + a.y * b.y;
}
float absDot(const sf::Vector2f &a, const sf::Vector2f &b) {
  return std::abs(dot(a, b));
}

sf::Vector2f perpendicular(const sf::Vector2f &a) { return {-a.y, a.x}; }

sf::Vector2f elementViseMultipication(const sf::Vector2f &vectorA,
                                      const sf::Vector2f &vectorB) {
  return {vectorA.x * vectorB.x, vectorA.y * vectorB.y};
}
float magnitude(const sf::Vector2f &a) { return std::sqrt(dot(a, a)); }
sf::Vector2f normalise(sf::Vector2f a) {
  float mag = magnitude(a);
  a.x /= mag;
  a.y /= mag;
  return a;
}
sf::Vector2f rotate(const sf::Vector2f &vectorToRotate, float angle) {
  sf::Vector2f rotatedVector;
  rotatedVector.x =
      vectorToRotate.x * std::cos(angle) - vectorToRotate.y * std::sin(angle);
  rotatedVector.y =
      vectorToRotate.x * std::sin(angle) + vectorToRotate.y * std::cos(angle);
  return rotatedVector;
}
std::array<sf::Vector2f, 2> getBaseCoordinateSystem(float orientationOfOrigin) {
  std::array<sf::Vector2f, 2> baseVector;
  baseVector[0] = {std::cos(orientationOfOrigin),
                   std::sin(orientationOfOrigin)};
  baseVector[1] = {-std::sin(orientationOfOrigin),
                   std::cos(orientationOfOrigin)};

  return baseVector;
}
sf::Vector2f transformToCordinateSystem(const sf::Vector2f &vectorToTransformed,
                                        const sf::Vector2f &positionOfOrigin,
                                        float orientationOfOrigin) {
  std::array<sf::Vector2f, 2> baseVector;
  baseVector[0] = {std::cos(orientationOfOrigin),
                   std::sin(orientationOfOrigin)};
  baseVector[1] = {-std::sin(orientationOfOrigin),
                   std::cos(orientationOfOrigin)};
  sf::Vector2f transformedVector;
  transformedVector.x =
      dot(vectorToTransformed - positionOfOrigin, baseVector[0]);
  transformedVector.y =
      dot(vectorToTransformed - positionOfOrigin, baseVector[1]);
  return transformedVector;
}

sf::Vector2f
inverseTransformToCordinateSystem(const sf::Vector2f &vectorToTransformed,
                                  const sf::Vector2f &positionOfOrigin,
                                  float orientationOfOrigin) {
  std::array<sf::Vector2f, 2> baseVector;
  baseVector[0] = {std::cos(-orientationOfOrigin),
                   std::sin(-orientationOfOrigin)};
  baseVector[1] = {-std::sin(-orientationOfOrigin),
                   std::cos(-orientationOfOrigin)};
  sf::Vector2f transformedVector;
  transformedVector.x =
      dot(vectorToTransformed, baseVector[0]) + positionOfOrigin.x;
  transformedVector.y =
      dot(vectorToTransformed, baseVector[1]) + positionOfOrigin.y;
  return transformedVector;
}