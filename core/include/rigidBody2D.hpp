#pragma once
#include "linearalg.hpp"

enum RigidBody2DType { CIRCLE, BOX, POLYGON, rb };
class RigidBody2D {

public:
  RigidBody2D(float x, float y) : position(x, y) {}
  RigidBody2D(RigidBody2D &) = delete;
  virtual ~RigidBody2D(){};
  virtual RigidBody2DType type() { return RigidBody2DType::rb; }
  void addForce(const la::Vector &);
  // operators
  bool operator==(const RigidBody2D &other) const;
  // getters
  float getInverseMass() const;
  float getInverseInertia() const;
  void setInverseMass(float);
  void setInverseInertia(float);
  void setOrientation(float);
  void addDisplacement(const la::Vector &);
  void integrate(float deltaTime);
  void integrateForces(float deltaTime);
  void integrateVelocities(float deltaTime);

  void clearAccumulators();

  void addForceAtPoint(const la::Vector &force, const la::Vector &point);
  void addForceOnBody(const la::Vector &force, const la::Vector &point);
  void addVelocity(const la::Vector &velocity);
  void addAngularVelocity(float rotation);
  la::Vector getPosition() const;
  la::Vector getVelocity() const;
  float getAngularSpeed() const;
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

  la::Vector localToGlobal(const la::Vector &localPoint) const;
  float sleepTime = 0.F;

private:
  float inverseMass = 0.F;
  float angularVelocity = 0.F;
  la::Vector forceAccum = {0.F, 0.F};
  float torqueAccum = 0.F;
  float angularDamping = 0.F;
  float linearDamping = 0.F;
  la::Vector lastFrameAcceleration = {0.f, 0.f};
  ;
  float inverseInertia = 0.F;
  bool isAwake_ = true;
  bool isMarked_ = false;

protected:
  float orientation = 0.F;
  la::Vector velocity = {0.F, 0.F};
  la::Vector position;
};