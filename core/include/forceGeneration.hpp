#pragma once
#include "rigidBody2D.hpp"
class ForceGenerator {
public:
  virtual void updateForce(RigidBody2D &body, float duration) = 0;
};
class Gravity : public ForceGenerator {
public:
  Gravity(const la::Vector &gravity_) : gravity(gravity_) {}
  void updateForce(RigidBody2D &body, float duration) override;

private:
  la::Vector gravity;
};