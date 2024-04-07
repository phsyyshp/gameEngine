#pragma once
#include "rigidBody2D.hpp"
#include <SFML/Graphics.hpp>
#include <cmath>
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

private:
  sf::Vector2f halfSize;
  float width;
  float height;
};