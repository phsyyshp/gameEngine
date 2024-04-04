#include "collidible.hpp"
#include <SFML/Graphics.hpp>
class Circle : public RigidBody2D, public sf::CircleShape {
public:
  Circle(float x, float y, float r)
      : sf::CircleShape(r), RigidBody2D(x, y), radius(r) {
    setPosition(x, y);
  }
  // getters
  float getRadius() const { return radius; }

private:
  float radius;
};

class Box : public RigidBody2D, public sf::RectangleShape {
public:
  Box(float x, float y, float w, float h)
      : sf::RectangleShape(sf::Vector2f(w, h)), RigidBody2D(x, y), width(w),
        height(h) {
    setPosition(x, y);
    setOrigin(w / 2, h / 2);
    setRotation(orientation / 3.14159f * 180);
  }

  void update() {

    setPosition(position.x, position.y);
    setRotation(orientation / 3.14159f * 180);
  };

private:
  float width;
  float height;
};