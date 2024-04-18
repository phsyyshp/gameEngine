#include "utils.hpp"
#include <SFML/Graphics/Vertex.hpp>
#include <SFML/System/Vector2.hpp>
#include <limits>
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
  if (mag == 0.F) {
    return {0.F, 0.F};
  }
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
sf::Vector2f tripleproduct(const sf::Vector2f &a, const sf::Vector2f &b,
                           const sf::Vector2f &c) {

  return b * dot(c, a) - a * dot(b, c);
}

void showPoints(sf::RenderWindow &window, const std::vector<sf::Vector2f> &p,
                sf::Color color) {
  for (auto point : p) {
    sf::CircleShape ax(2.f);
    ax.setFillColor(color);
    ax.setPosition(point);
    ax.setOrigin(2, 2);
    window.draw(ax);
  }
}
sf::Vector2f computeIntersection(const std::array<sf::Vector2f, 2> &a,
                                 const std::array<sf::Vector2f, 2> &b) {

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
sf::Vector2f computeIntersection(const std::vector<sf::Vector2f> &a,
                                 const std::array<sf::Vector2f, 2> &b) {

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

sf::Vector2f plotLine(const std::vector<sf::Vector2f> &points,
                      sf::RenderWindow &window, sf::Color color) {
  std::array<sf::Vertex, 2> line = {sf::Vertex(points[0], color),
                                    sf::Vertex(points[1], color)};
  window.draw(line.data(), 2, sf::Lines);
}

sf::Vector2f plotLine(const std ::array<sf::Vector2f, 2> &points,
                      sf::RenderWindow &window, sf::Color color) {
  std::array<sf::Vertex, 2> line = {sf::Vertex(points[0], color),
                                    sf::Vertex(points[1], color)};
  window.draw(line.data(), 2, sf::Lines);
}