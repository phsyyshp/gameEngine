#pragma once
#include "rigidBody2D.hpp"
#include <SFML/System/Vector2.hpp>
#include <array>
#include <memory>

class Contact {
public:
  Contact() = default;

  // getters declare:
  std::array<sf::Vector2f, 2> getContactPosition() const;
  sf::Vector2f getContactNormal() const;
  float getPenetrationDepth() const;
  float getFriction() const;
  float getResitution() const;
  float getTotalImpulseNormal();
  std::array<sf::Vector2f, 2> getLocalContactPosition() const;
  // setters declare:
  void setContactPosition(const std::array<sf::Vector2f, 2> &rp,
                          const RigidBody2D &bodyA, const RigidBody2D &bodyB);
  void setContactNormal(const sf::Vector2f &);
  void setPenetrationDepth(float);
  void setFriction(float);
  void setResitution(float);
  void setTotalImpulseNormal(float totalImpulseNormal);

  void makePersistent();
  bool isPersistent() const;

private:
  std::array<sf::Vector2f, 2> contactPosition = {};      // {A,B}
  std::array<sf::Vector2f, 2> contactPositionLocal = {}; // {A,B}
  sf::Vector2f contactNormal;                            // Always from B to A;
  float normalImpulseSum = 0;
  float penetrationDepth = 0;
  float friction;
  float resitution = 0.5F;
  bool persistent = false;
};