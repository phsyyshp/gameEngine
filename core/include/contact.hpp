

#pragma once
#include "rigidBody2D.hpp"
#include "utils.hpp"

class Contact {
  static RigidBody2D emptyBody;

public:
  Contact(RigidBody2D &a = emptyBody, RigidBody2D &b = emptyBody)
      : bodies{a, b} {}
  sf::Vector2f calculateFrictionlessImpulse() const;

  void applyImpulse();
  // getters declare:
  sf::Vector2f getContactPoint() const;
  sf::Vector2f getContactNormal() const;
  float getPenetrationDepth() const;
  std::array<std::reference_wrapper<RigidBody2D>, 2> getBodies() const;
  float getFriction() const;
  float getResitution() const;
  float getDesiredDeltaVelocity() const;
  // setters declare:
  void setContactPoint(const sf::Vector2f &);
  void setContactNormal(const sf::Vector2f &);
  void setPenetrationDepth(float);
  void setFriction(float);
  void setResitution(float);
  void setDesiredDeltaVelocity(float);

private:
  sf::Vector2f contactPoint;
  sf::Vector2f contactNormal;
  float penetrationDepth;
  std::array<std::reference_wrapper<RigidBody2D>, 2> bodies;
  float friction;
  float resitution;
  float desiredDeltaVelocity;
};