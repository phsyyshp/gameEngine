#pragma once
#include <SFML/Graphics.hpp>
#include <iostream>
class RigidBody2D {

public:
  RigidBody2D(float x, float y) : position(x, y) {}

  void addForce(const sf::Vector2f &);
  // getters
  float getInverseMass() const;
  void setInverseMass(float);
  void integrate(float deltaTime);
  void clearAccumulators();

  void addForceAtPoint(const sf::Vector2f &force, const sf::Vector2f &point);
  void addForceOnBody(const sf::Vector2f &force, const sf::Vector2f &point);

private:
  float inverseMass;
  float angularVelocity = 0;
  sf::Vector2f forceAccum = {0.f, 0.f};
  float torqueAccum = 0.f;
  float angularDamping = 0.f;
  float linearDamping = 0.f;
  sf::Vector2f lastFrameAcceleration;
  float inverseInertia = 1.f;

protected:
  float orientation = 0;
  sf::Vector2f velocity = {0.f, 0.f};
  sf::Vector2f position;
};