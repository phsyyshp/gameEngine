#include "rigidBody2D.hpp"
#include <cmath>
// getters

float RigidBody2D::getInverseMass() const { return inverseMass; }
float RigidBody2D::getInverseInertia() const { return inverseInertia; }
sf::Vector2f RigidBody2D::getPosition() const { return position; }
sf::Vector2f RigidBody2D::getVelocity() const { return velocity; }
float RigidBody2D::getAngularVelocity() const { return angularVelocity; }
void RigidBody2D::addForce(const sf::Vector2f &force) { forceAccum += force; }
void RigidBody2D::addVelocity(const sf::Vector2f &velocity) {
  this->velocity += velocity;
}
void RigidBody2D::addAngularVelocity(float rotation) {
  angularVelocity += rotation;
}
void RigidBody2D::addForceAtPoint(const sf::Vector2f &force,
                                  const sf::Vector2f &point) {
  forceAccum += force;
  torqueAccum +=
      (point.x - position.x) * force.y - (point.y - position.y) * force.x;
}
float RigidBody2D::getOrientation() const { return orientation; }
void RigidBody2D::addDisplacement(const sf::Vector2f &addDisplacement) {
  position += addDisplacement;
}

void RigidBody2D::addForceOnBody(const sf::Vector2f &force,
                                 const sf::Vector2f &localPoint) {
  sf::Vector2f worldPoint = localPoint + position;
  addForceAtPoint(force, worldPoint);
  // forceAccum += force;
  // torqueAccum += localPoint.x * force.y - localPoint.y * force.x;
}
void RigidBody2D::integrate(float deltaTime) {
  lastFrameAcceleration = forceAccum * inverseMass;
  float angularAcceleration = torqueAccum * inverseInertia;

  velocity += lastFrameAcceleration * deltaTime;
  angularVelocity += angularAcceleration * deltaTime;

  // velocity *= std::pow(linearDamping, deltaTime);
  // angularVelocity *= std::pow(angularDamping, deltaTime);

  position += velocity * deltaTime;
  orientation += angularVelocity * deltaTime;
  orientation = std::fmod(orientation, 2 * M_PI);
  clearAccumulators();
}
void RigidBody2D::integrateForces(float deltaTime) {
  lastFrameAcceleration = forceAccum * inverseMass;
  float angularAcceleration = torqueAccum * inverseInertia;

  velocity += lastFrameAcceleration * deltaTime;
  angularVelocity += angularAcceleration * deltaTime;
}
void RigidBody2D::integrateVelocities(float deltaTime) {

  position += velocity * deltaTime;
  orientation += angularVelocity * deltaTime;
  orientation = std::fmod(orientation, 2 * M_PI);
  clearAccumulators();
}

void RigidBody2D::setInverseInertia(float inverseInertia_) {
  inverseInertia = inverseInertia_;
}
void RigidBody2D::setInverseMass(float inverseMass_) {
  inverseMass = inverseMass_;
}
void RigidBody2D::clearAccumulators() {
  forceAccum = {0.F, 0.F};
  torqueAccum = 0.F;
}
void RigidBody2D::setOrientation(float orientation_) {
  orientation = orientation_;
}
bool RigidBody2D::isAwake() const { return isAwake_; }
void RigidBody2D::wakeUp() {
  sleepTime = 0;
  isAwake_ = true;
}
void RigidBody2D::sleep() { isAwake_ = false; }
sf::Vector2f RigidBody2D::localToGlobal(const sf::Vector2f &localPoint) const {
  return inverseTransformToCordinateSystem(localPoint, position, orientation);
}
bool RigidBody2D::isMarked() const { return isMarked_; }
void RigidBody2D::mark() { isMarked_ = true; }
void RigidBody2D::clearMark() { isMarked_ = false; }

bool RigidBody2D::isDynamic() const { return inverseMass > 0.F; }
bool RigidBody2D::operator==(const RigidBody2D &other) const {
  return this == &other;
}
