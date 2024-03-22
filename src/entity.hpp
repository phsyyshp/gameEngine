#include <SFML/Graphics.hpp>
class Entity {
public:
  Entity(float x, float y, float _width, float _height)
      : position(x, y), velocity(0, 0), width(_width), height(_height) {}

  void updatePosition(float deltaTime);
  void updateVelocity(float deltaTime);
  void update(float deltaTime);

private:
  sf::Vector2f position;
  sf::Vector2f velocity;
  float width;
  float height;
};