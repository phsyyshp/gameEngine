#include "utils.hpp"
float cross(const sf::Vector2f &a, const sf::Vector2f &b) {
  return a.x * b.y - a.y * b.x;
}
float dot(const sf::Vector2f &a, const sf::Vector2f &b) {
  return a.x * b.x + a.y * b.y;
}
sf::Vector2f perpendicular(const sf::Vector2f &a) {
  return sf::Vector2f(-a.y, a.x);
}
