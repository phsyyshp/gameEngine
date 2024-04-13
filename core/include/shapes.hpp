#pragma once
#include "rigidBody2D.hpp"
#include "utils.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/System/Vector2.hpp>
#include <cmath>
#include <limits>
class Circle : public RigidBody2D, public sf::CircleShape {
public:
  Circle(float x, float y, float r)
      : sf::CircleShape(r), RigidBody2D(x, y), radius(r) {
    setPosition(x, y);
    setOrigin(r, r);
  }
  // getters
  float getRadius() const { return radius; }

  void update() { setPosition(position.x, position.y); }

private:
  float radius;
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

private:
  sf::Vector2f halfSize;
  float width;
  float height;
};