#include "shapes.hpp"
class Entity : public Box {
public:
  Entity(float x, float y, float _width, float _height)
      : Box(x, y, _width, _height) {}

  void updatePosition(float deltaTime);
  void updateVelocity(float deltaTime);
  void update(float deltaTime);

private:
  sf::Vector2f velocity;
};