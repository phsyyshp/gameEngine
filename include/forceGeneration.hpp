#include "rigidBody2D.hpp"
#include <SFML/Graphics.hpp>
class ForceGenerator {
public:
  virtual void updateForce(RigidBody2D &body, float duration) = 0;
};
class Gravity : public ForceGenerator {
public:
  Gravity(const sf::Vector2f &gravity_) : gravity(gravity_) {}
  void updateForce(RigidBody2D &body, float duration) override;

private:
  sf::Vector2f gravity;
};