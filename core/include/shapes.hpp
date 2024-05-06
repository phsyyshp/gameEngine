#pragma once
#include "linearalg.hpp"
#include "rigidBody2D.hpp"
// #include "visuals.hpp"
// #include <SFML/Graphics.hpp>
// #include <SFML/Graphics/ConvexShape.hpp>
// #include <SFML/Graphics/RenderWindow.hpp>
// #include <SFML/System/Vector2.hpp>
#include <cmath>
#include <limits>
class Circle : public RigidBody2D {
public:
  Circle(float x, float y, float r) : RigidBody2D(x, y), radius(r) {
    setInverseInertia(20.F / std::pow(r, 4) / M_PIf);
  }
  virtual ~Circle(){};
  // getters
  float getRadius() const { return radius; }
  la::Vector getSupport(const la::Vector &direction) {
    return position + normalise(direction) * radius;
  }
  RigidBody2DType type() override { return type_; }

private:
  float radius;
  RigidBody2DType type_ = RigidBody2DType::CIRCLE;
};

class Box : public RigidBody2D {
public:
  Box(float x, float y, float w, float h)
      : RigidBody2D(x, y), width(w), height(h), halfSize(w / 2, h / 2) {
    setInverseInertia(5 * 12 / (w * h * h * h));
  }
  virtual ~Box(){};
  la::Vector getHalfSize() const { return halfSize; }
  la::Vector getClosestPoint(const la::Vector &point) {
    la::Vector localPoint = point - RigidBody2D::getPosition();
    la::Vector closestPoint = localPoint;
    closestPoint.x = std::clamp(closestPoint.x, -width / 2, width / 2);
    closestPoint.y = std::clamp(closestPoint.y, -height / 2, height / 2);
    return closestPoint + RigidBody2D::getPosition();
  }
  std::array<la::Vector, 4> getVertices() const {

    std::array<la::Vector, 2> axes = getBaseCoordinateSystem(getOrientation());
    std::array<la::Vector, 4> vertices = {
        position + (axes[0] * halfSize.x + axes[1] * halfSize.y),
        position + (axes[0] * halfSize.x - axes[1] * halfSize.y),
        position + (-axes[0] * halfSize.x - axes[1] * halfSize.y),
        position + (-axes[0] * halfSize.x + axes[1] * halfSize.y)};
    return vertices;
  }

  la::Vector getSupport(const la::Vector &direction) const {
    float maxDot = -std::numeric_limits<float>::max();
    float projection;
    la::Vector supportPoint;
    std::array<la::Vector, 4> vertices = getVertices();
    for (auto &vertex : vertices) {
      projection = dot(vertex, direction);
      if (maxDot < projection) {
        maxDot = projection;
        supportPoint = vertex;
      }
    }
    return supportPoint;
  }
  void getIncidentReferencePolygon(
      std::vector<la::Vector> &polygon, const la::Vector &normal,
      std::array<std::array<la::Vector, 2>, 2> &adjacentEdges) {
    // findClosest vertex along normal
    std::array<la::Vector, 4> vertices = getVertices();
    la::Vector furthestVertexAlongNormal = getSupport(normal);
    auto idxIt =
        std::find(vertices.begin(), vertices.end(), furthestVertexAlongNormal);
    int idx = idxIt - vertices.begin();
    // find edges that includes that vertex
    std::array<la::Vector, 2> edge1 = {vertices[(idx + 4 - 1) % 4],
                                       furthestVertexAlongNormal};
    std::array<la::Vector, 2> edge2 = {furthestVertexAlongNormal,
                                       vertices[(idx + 1) % 4]};
    std::array<std::array<la::Vector, 2>, 2> containingAxes = {edge1, edge2};
    // find the edge that its normal have the highest projection on collision
    // normal
    float maxDot = -std::numeric_limits<float>::max();
    for (auto &edge : containingAxes) {
      float projection =
          std::abs(dot(normalise(perpendicular(edge[1] - edge[0])), normal));
      if (projection > maxDot) {
        maxDot = projection;
        polygon = {edge[0], edge[1]};
      }
      // if (Visual::isDebug) {

      //   showPoints(Visual::window, std::vector<la::Vector>{edge[0], edge[1]},
      //              sf::Color::Red);
      // }
    }
    auto it = std::find(vertices.begin(), vertices.end(), polygon[0]);
    adjacentEdges[0] = {vertices[(it - vertices.begin() + 4 - 1) % 4],
                        polygon[0]};
    it = std::find(vertices.begin(), vertices.end(), polygon[1]);
    adjacentEdges[1] = {polygon[1], vertices[(it - vertices.begin() + 1) % 4]};
    // if (Visual::isDebug) {

    //   showPoints(
    //       Visual::window,
    //       std::vector<la::Vector>{adjacentEdges[0][0], adjacentEdges[0][1]},
    //       sf::Color::Blue);
    //   showPoints(
    //       Visual::window,
    //       std::vector<la::Vector>{adjacentEdges[1][0], adjacentEdges[1][1]},
    //       sf::Color::Blue);
    // }
  }
  RigidBody2DType type() override { return type_; }

private:
  la::Vector halfSize;
  float width;
  float height;
  RigidBody2DType type_ = RigidBody2DType::BOX;
};

class Polygon : public RigidBody2D {
public:
  explicit Polygon(const std::vector<la::Vector> &v)
      : vertices(v), RigidBody2D(0, 0) {
    la::Vector center = {0, 0};
    int i = 0;
    center = center * 1.f / static_cast<float>(vertices.size());
    RigidBody2D::position = {center.x, center.y};
  }
  RigidBody2DType type() override { return type_; }

private:
  std::vector<la::Vector> vertices;
  RigidBody2DType type_ = RigidBody2DType::POLYGON;
};