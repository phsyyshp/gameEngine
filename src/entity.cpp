#include "entity.hpp"
void Entity::updatePosition(float deltaTime) {
  position.x += deltaTime * velocity.x;
  position.y += deltaTime * velocity.y;
}
void Entity::updateVelocity(float deltaTime) {
  velocity.x = 0.f;
  velocity.y = 0.f;
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
    velocity.x = 1.f;
  }
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
    velocity.x = -1.f;
  }
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) {
    velocity.y = -1.f;
  }
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
    velocity.y = 1.f;
  }
}
void Entity::update(float deltaTime) {
  updatePosition(deltaTime);
  updateVelocity(deltaTime);
}