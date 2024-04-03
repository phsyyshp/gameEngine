#include "rigidBody2D.hpp"
#include <cmath>
// getters

float RigidBody2D::getInverseMass() const { return inverseMass; }
void RigidBody2D::addForce(const sf::Vector2f &force) { forceAccum += force; }

void RigidBody2D::addForceAtPoint(const sf::Vector2f &force,
                                  const sf::Vector2f &point) {
  forceAccum += force;
  torqueAccum +=
      (point.x - position.x) * force.y - (point.y - position.y) * force.x;
}
void RigidBody2D::addForceOnBody(const sf::Vector2f &force,
                                 const sf::Vector2f &localPoint) {
  sf::Vector2f worldPoint;
  worldPoint.x = std::cos(orientation) * localPoint.x -
                 std::sin(orientation) * localPoint.y + position.x;
  worldPoint.y = std::sin(orientation) * localPoint.x +
                 std::cos(orientation) * localPoint.y + position.y;
  forceAccum += force;
  torqueAccum += worldPoint.x * force.y - worldPoint.y * force.x;
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

void RigidBody2D::setInverseMass(float inverseMass_) {
  inverseMass = inverseMass_;
}
void RigidBody2D::clearAccumulators() {
  forceAccum = {0.f, 0.f};
  torqueAccum = 0.f;
}
