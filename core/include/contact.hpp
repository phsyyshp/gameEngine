

#pragma once
#include "rigidBody2D.hpp"
#include "utils.hpp"
#include <SFML/System/Vector2.hpp>

class Contact {
  static RigidBody2D emptyBody;

public:
  Contact(RigidBody2D &a = emptyBody, RigidBody2D &b = emptyBody)
      : bodies{a, b} {}
  sf::Vector2f calculateFrictionlessImpulse();

  void applyImpulse(float);
  // getters declare:
  sf::Vector2f getContactPoint() const;
  sf::Vector2f getContactNormal() const;
  float getPenetrationDepth() const;
  std::array<std::reference_wrapper<RigidBody2D>, 2> &getBodies();
  float getFriction() const;
  float getResitution() const;
  // setters declare:
  void setContactPoint(const sf::Vector2f &);
  void setContactNormal(const sf::Vector2f &);
  void setPenetrationDepth(float);
  void setFriction(float);
  void setResitution(float);

  void applyVelocityChange(float lagrangianMultiplier);
  void applyPositionChange(std::array<sf::Vector2f, 2> &);
  void applyVelocityChangeSphereSphere();
  sf::Vector2f calculateFrictionlessImpulseSphereSphere();
  float solveContactConstraints(float deltaTime);
  void setTotalImpulseNormal(float totalImpulseNormal);
  float getTotalImpulseNormal();

private:
  sf::Vector2f contactPoint;
  sf::Vector2f contactNormal;
  float normalImpulseSum = 0;
  float penetrationDepth = 0;
  std::array<std::reference_wrapper<RigidBody2D>, 2> bodies;
  float friction;
  float resitution = 0.5f;
};