#pragma once
#include "utils.hpp"
#include <SFML/Graphics.hpp>

#include <iostream>
enum RigidBody2DType { CIRCLE, BOX, POLYGON, rb };
class RigidBody2D {

public:
  RigidBody2D(float x, float y) : position(x, y) {}
  RigidBody2D(RigidBody2D &) = delete;
  virtual ~RigidBody2D(){};
  virtual RigidBody2DType type() { return RigidBody2DType::rb; }
  void addForce(const sf::Vector2f &);
  // operators
  bool operator==(const RigidBody2D &other) const;
  // getters
  float getInverseMass() const;
  float getInverseInertia() const;
  void setInverseMass(float);
  void setInverseInertia(float);
  void setOrientation(float);
  void addDisplacement(const sf::Vector2f &);
  void integrate(float deltaTime);
  void integrateForces(float deltaTime);
  void integrateVelocities(float deltaTime);

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
  bool isMarked() const;
  void mark();
  void clearMark();
  bool isDynamic() const;

  sf::Vector2f localToGlobal(const sf::Vector2f &localPoint) const;
  float sleepTime = 0.F;

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
  bool isMarked_ = false;

protected:
  float orientation = 0.F;
  sf::Vector2f velocity = {0.F, 0.F};
  sf::Vector2f position;
};