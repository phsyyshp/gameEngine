#pragma once
#include "rigidBody2D.hpp"
#include "utils.hpp"
#include "visuals.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/ConvexShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Vector2.hpp>
#include <cmath>
#include <limits>
class Circle : public RigidBody2D, public sf::CircleShape {
public:
  Circle(float x, float y, float r)
      : sf::CircleShape(r), RigidBody2D(x, y), radius(r) {
    setPosition(x, y);
    setOrigin(r, r);
    setRotation(orientation / 3.14159f * 180);
    setInverseInertia(20.F / std::pow(r, 4) / M_PIf);
  }
  virtual ~Circle(){};
  // getters
  float getRadius() const { return radius; }

  void update() {
    setPosition(position.x, position.y);
    setRotation(orientation / M_PIf * 180.f);
  }
  sf::Vector2f getSupport(const sf::Vector2f &direction) {
    return position + normalise(direction) * radius;
  }

  RigidBody2DType type() override { return type_; }

private:
  float radius;
  RigidBody2DType type_ = RigidBody2DType::CIRCLE;
};

class Box : public RigidBody2D, public sf::RectangleShape {
public:
  Box(float x, float y, float w, float h)
      : sf::RectangleShape(sf::Vector2f(w, h)), RigidBody2D(x, y), width(w),
        height(h), halfSize(w / 2, h / 2) {
    setPosition(x, y);
    setOrigin(w / 2, h / 2);
    setRotation(orientation / 3.14159f * 180);
    setInverseInertia(5 * 12 / (w * h * h * h));
  }
  virtual ~Box(){};
  sf::Vector2f getHalfSize() const { return halfSize; }
  sf::Vector2f getClosestPoint(const sf::Vector2f &point) {
    sf::Vector2f localPoint = point - RigidBody2D::getPosition();
    sf::Vector2f closestPoint = localPoint;
    closestPoint.x = std::clamp(closestPoint.x, -width / 2, width / 2);
    closestPoint.y = std::clamp(closestPoint.y, -height / 2, height / 2);
    return closestPoint + RigidBody2D::getPosition();
  }
  void update() {

    setPosition(position.x, position.y);
    setRotation(orientation / M_PIf * 180.f);
  };
  std::array<sf::Vector2f, 4> getVertices() const {

    // position = RigidBody2D::getPosition();
    std::array<sf::Vector2f, 2> axes =
        getBaseCoordinateSystem(getOrientation());
    std::array<sf::Vector2f, 4> vertices = {
        position + (axes[0] * halfSize.x + axes[1] * halfSize.y),
        position + (axes[0] * halfSize.x - axes[1] * halfSize.y),
        position + (-axes[0] * halfSize.x - axes[1] * halfSize.y),
        position + (-axes[0] * halfSize.x + axes[1] * halfSize.y)};
    return vertices;
  }

  sf::Vector2f getSupport(const sf::Vector2f &direction) const {
    float maxDot = -std::numeric_limits<float>::max();
    float projection;
    sf::Vector2f supportPoint;
    std::array<sf::Vector2f, 4> vertices = getVertices();
    for (auto &vertex : vertices) {
      projection = dot(vertex, direction);
      if (maxDot < projection) {
        maxDot = projection;
        supportPoint = vertex;
      }
    }
    return supportPoint;
  }
  bool isPointIn(const sf::Vector2f &point) {
    sf::Vector2f relativePoint =
        transformToCordinateSystem(point, position, getOrientation());
    return (std::abs(relativePoint.x) <= std::abs(halfSize.x)) &&
           (std::abs(relativePoint.y) <= std::abs(halfSize.y));
  }
  void getIncidentReferencePolygon(
      std::vector<sf::Vector2f> &polygon, const sf::Vector2f &normal,
      std::array<std::array<sf::Vector2f, 2>, 2> &adjacentEdges) {
    // findClosest vertex along normal
    std::array<sf::Vector2f, 4> vertices = getVertices();
    sf::Vector2f furthestVertexAlongNormal = getSupport(normal);
    auto idxIt =
        std::find(vertices.begin(), vertices.end(), furthestVertexAlongNormal);
    int idx = idxIt - vertices.begin();
    // find edges that includes that vertex
    std::array<sf::Vector2f, 2> edge1 = {vertices[(idx + 4 - 1) % 4],
                                         furthestVertexAlongNormal};
    std::array<sf::Vector2f, 2> edge2 = {furthestVertexAlongNormal,
                                         vertices[(idx + 1) % 4]};
    std::array<std::array<sf::Vector2f, 2>, 2> containingAxes = {edge1, edge2};
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
      if (Visual::isDebug) {

        showPoints(Visual::window, std::vector<sf::Vector2f>{edge[0], edge[1]},
                   sf::Color::Red);
      }
    }
    auto it = std::find(vertices.begin(), vertices.end(), polygon[0]);
    adjacentEdges[0] = {vertices[(it - vertices.begin() + 4 - 1) % 4],
                        polygon[0]};
    it = std::find(vertices.begin(), vertices.end(), polygon[1]);
    adjacentEdges[1] = {polygon[1], vertices[(it - vertices.begin() + 1) % 4]};
    if (Visual::isDebug) {

      showPoints(
          Visual::window,
          std::vector<sf::Vector2f>{adjacentEdges[0][0], adjacentEdges[0][1]},
          sf::Color::Blue);
      showPoints(
          Visual::window,
          std::vector<sf::Vector2f>{adjacentEdges[1][0], adjacentEdges[1][1]},
          sf::Color::Blue);
    }
  }
  RigidBody2DType type() override { return type_; }

private:
  sf::Vector2f halfSize;
  float width;
  float height;
  RigidBody2DType type_ = RigidBody2DType::BOX;
};

class Polygon : public RigidBody2D, public sf::ConvexShape {
public:
  explicit Polygon(const std::vector<sf::Vector2f> &v)
      : vertices(v), RigidBody2D(0, 0) {
    setPointCount(vertices.size());
    sf::Vector2f center = {0, 0};
    int i = 0;
    for (auto &vertex : vertices) {
      center += vertex;
      setPoint(i, vertex);
      i++;
    }
    center = center * 1.f / static_cast<float>(vertices.size());

    RigidBody2D::position = {center.x, center.y};
    setPosition(center);
    setOrigin(center - vertices[0]);
    setRotation(getOrientation() / 3.14159f * 180);
  }
  void update() {
    setPosition(position.x, position.y);
    setRotation(orientation / M_PIf * 180.f);
  }
  RigidBody2DType type() override { return type_; }

private:
  std::vector<sf::Vector2f> vertices;
  RigidBody2DType type_ = RigidBody2DType::POLYGON;
};