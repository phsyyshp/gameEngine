#pragma once
#include "utils.hpp"
#include <SFML/Graphics.hpp>

#include <iostream>
class RigidBody2D {

public:
  RigidBody2D(float x, float y) : position(x, y) {}
  virtual ~RigidBody2D(){};

  void addForce(const sf::Vector2f &);
  // getters
  float getInverseMass() const;
  float getInverseInertia() const;
  void setInverseMass(float);
  void setInverseInertia(float);
  void setOrientation(float);
  void addDisplacement(const sf::Vector2f &);
  void integrate(float deltaTime);
  void clearAccumulators();

  void addForceAtPoint(const sf::Vector2f &force, const sf::Vector2f &point);
  void addForceOnBody(const sf::Vector2f &force, const sf::Vector2f &point);
  void addVelocity(const sf::Vector2f &velocity);
  void addAngularVelocity(float rotation);
  sf::Vector2f getPosition() const;
  sf::Vector2f getVelocity() const;
  float getAngularVelocity() const;
  float getOrientation() const;
  bool isAwake() const;
  int getAwakeTimer() const;
  void setAwakeTimer(int);
  void wakeUp();
  void sleep();

  sf::Vector2f localToGlobal(const sf::Vector2f &localPoint) const;

private:
  float inverseMass = 0.F;
  float angularVelocity = 0.F;
  sf::Vector2f forceAccum = {0.F, 0.F};
  float torqueAccum = 0.F;
  float angularDamping = 0.F;
  float linearDamping = 0.F;
  sf::Vector2f lastFrameAcceleration = {0.f, 0.f};
  ;
  float inverseInertia = 0.F;

  bool isAwake_ = true;

  int isAwakeTimer = 0;

protected:
  float orientation = 0.F;
  sf::Vector2f velocity = {0.F, 0.F};
  sf::Vector2f position;
};