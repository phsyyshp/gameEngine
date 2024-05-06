#pragma once
#include <array>
#include <cmath>
#include <vector>

namespace la {
class Vector {
public:
  Vector() : x(), y() {}
  Vector(float x_, float y_) : x(x_), y(y_) {}
  float x = 0.F;
  float y = 0.F;
  bool operator==(const Vector &other) const {
    return x == other.x && y == other.y;
  }
  Vector operator+(const Vector &other) const {
    return {x + other.x, y + other.y};
  }
  Vector operator-(const Vector &other) const {
    return {x - other.x, y - other.y};
  }
  Vector operator-() const { return {-x, -y}; }
  Vector operator/(float a) const { return {x / a, y / a}; }
  void operator*=(float a) {
    x *= a;
    y *= a;
  }
  void operator+=(const Vector &other) {
    x += other.x;
    y += other.y;
  }
};
inline Vector operator*(const Vector &vector, float a) {
  return {vector.x * a, vector.y * a};
};
inline Vector operator*(float a, const Vector &vector) {
  return {vector.x * a, vector.y * a};
};

} // namespace la
float cross(const la::Vector &, const la::Vector &);
float dot(const la::Vector &, const la::Vector &);
float absDot(const la::Vector &, const la::Vector &);

float magnitude(const la::Vector &);
la::Vector normalise(la::Vector);
la::Vector perpendicular(const la::Vector &);
la::Vector transformToCordinateSystem(const la::Vector &vectorToTransformed,
                                      const la::Vector &positionOfOrigin,
                                      float orientationOfOrigin);

la::Vector inverseTransformToCordinateSystem(const la::Vector &,
                                             const la::Vector &, float);
la::Vector rotate(const la::Vector &, float);
std::array<la::Vector, 2> getBaseCoordinateSystem(float);
la::Vector elementViseMultipication(const la::Vector &, const la::Vector &);
la::Vector tripleproduct(const la::Vector &, const la::Vector &,
                         const la::Vector &);
la::Vector computeIntersection(const std::array<la::Vector, 2> &,
                               const std::array<la::Vector, 2> &);

la::Vector computeIntersection(const std::vector<la::Vector> &,
                               const std::array<la::Vector, 2> &);
