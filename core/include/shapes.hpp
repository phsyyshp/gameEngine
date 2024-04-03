#include "collidible.hpp"
#include <SFML/Graphics.hpp>
class Circle : public Collidible, public sf::CircleShape {
public:
  Circle(float x, float y, float r)
      : sf::CircleShape(r), Collidible(x, y), radius(r) {
    setPosition(x, y);
  }
  // getters
  float getRadius() const { return radius; }

private:
  float radius;
};

class Box : public Collidible, public sf::RectangleShape {
public:
  Box(float x, float y, float w, float h)
      : sf::RectangleShape(sf::Vector2f(w, h)), Collidible(x, y), width(w),
        height(h) {
    setPosition(x, y);
    setOrigin(w / 2, h / 2);
    setRotation(orientation);
  }

  void update() {

    setPosition(position.x, position.y);
    setRotation(orientation);
  };

private:
  float width;
  float height;
};