#pragma once
#include <SFML/System/Vector2.hpp>
#include <array>

class Contact {
public:
  Contact() = default;

  // getters declare:
  sf::Vector2f getContactPosition() const;
  sf::Vector2f getContactNormal() const;
  float getPenetrationDepth() const;
  float getFriction() const;
  float getResitution() const;
  float getTotalImpulseNormal();
  float getBias();
  std::array<sf::Vector2f, 2> getLocalContactPosition() const;
  // setters declare:
  void setContactPosition(const sf::Vector2f &contactPosition_);
  void setContactNormal(const sf::Vector2f &);
  void setPenetrationDepth(float);
  void setFriction(float);
  void setResitution(float);
  void setTotalImpulseNormal(float totalImpulseNormal);
  void setBias(float);

  void makePersistent();
  bool isPersistent() const;

private:
  sf::Vector2f contactPosition = {};
  sf::Vector2f contactNormal; // Always from B to A;
  float normalImpulseSum = 0.F;
  float penetrationDepth = 0.F;
  float friction;
  float resitution = 0.5F;
  bool persistent = false;
  float bias = 0.F;
};