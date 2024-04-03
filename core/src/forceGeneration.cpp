#include "forceGeneration.hpp"
void Gravity::updateForce(RigidBody2D &body, float duration) {

  if (body.getInverseMass() == 0.f)
    return;
  body.addForce(gravity / body.getInverseMass());
}