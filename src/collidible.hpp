#include <SFML/Graphics.hpp>
class Collidible {
public:
  Collidible(float x, float y) : position(x, y) {}
  // getters
  sf::Vector2f getPosition() const;

protected:
  sf::Vector2f position;
};