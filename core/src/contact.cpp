#include "contact.hpp"
#include <SFML/System/Vector2.hpp>
RigidBody2D Contact::emptyBody(-80, 80);
// getters
sf::Vector2f Contact::getContactPoint() const { return contactPoint; }
sf::Vector2f Contact::getContactNormal() const { return contactNormal; }
float Contact::getPenetrationDepth() const { return penetrationDepth; }
std::array<std::reference_wrapper<RigidBody2D>, 2> &Contact::getBodies() {
  return bodies;
}
float Contact::getFriction() const { return friction; }
float Contact::getResitution() const { return resitution; }

// setters
void Contact::setContactPoint(const sf::Vector2f &contactPoint_) {
  contactPoint = contactPoint_;
}
void Contact::setContactNormal(const sf::Vector2f &contactNormal_) {
  contactNormal = contactNormal_;
}
void Contact::setPenetrationDepth(float penetrationDepth_) {
  penetrationDepth = penetrationDepth_;
}
void Contact::setFriction(float friction_) { friction = friction_; }
void Contact::setResitution(float resitution_) { resitution = resitution_; }
// helpers

sf::Vector2f Contact::calculateFrictionlessImpulse() {
  sf::Vector2f impulse;
  std::array<sf::Vector2f, 2> linearVelocityAtContacRelativeToCenter;
  std::array<sf::Vector2f, 2> linearVelocityAtContact;
  std::array<float, 2> angularSpeed{};
  std::array<sf::Vector2f, 2> relativeContactPosition;
  relativeContactPosition[0] = contactPoint - bodies[0].get().getPosition();
  relativeContactPosition[1] = contactPoint - bodies[1].get().getPosition();

  float totalInverseMass =
      bodies[0].get().getInverseMass() + bodies[1].get().getInverseMass();
  for (int i = 0; i < 2; i++) {
    linearVelocityAtContacRelativeToCenter[i] =
        bodies[i].get().getAngularVelocity() *
        perpendicular(relativeContactPosition[i]);
    linearVelocityAtContact[i] = bodies[i].get().getVelocity() +
                                 linearVelocityAtContacRelativeToCenter[i];
  }
  sf::Vector2f contactVelocity =
      linearVelocityAtContact[0] - linearVelocityAtContact[1];
  float impulseForce = dot(contactVelocity, contactNormal);
  angularSpeed[0] = cross(relativeContactPosition[0], contactNormal) *
                    bodies[0].get().getInverseInertia();
  angularSpeed[1] = cross(relativeContactPosition[1], contactNormal) *
                    bodies[1].get().getInverseInertia();
  float angularEffect =
      dot(perpendicular(relativeContactPosition[0]) * angularSpeed[0],
          contactNormal) +
      dot(perpendicular(relativeContactPosition[1]) * angularSpeed[1],
          contactNormal);
  impulseForce =
      impulseForce * (-1.0f - resitution) / (totalInverseMass + angularEffect);
  return impulseForce * contactNormal;
}
sf::Vector2f Contact::calculateFrictionlessImpulseSphereSphere() {
  sf::Vector2f impulse;
  std::array<sf::Vector2f, 2> linearVelocityAtContact;
  std::array<sf::Vector2f, 2> relativeContactPosition;
  relativeContactPosition[0] = contactPoint - bodies[0].get().getPosition();
  relativeContactPosition[1] = contactPoint - bodies[1].get().getPosition();

  float totalInverseMass =
      bodies[0].get().getInverseMass() + bodies[1].get().getInverseMass();
  for (int i = 0; i < 2; i++) {
    linearVelocityAtContact[i] = bodies[i].get().getVelocity();
  }
  sf::Vector2f contactVelocity =
      linearVelocityAtContact[0] - linearVelocityAtContact[1];
  float impulseForce = dot(contactVelocity, contactNormal);
  impulseForce = impulseForce * (-1.0f - resitution) / (totalInverseMass);
  return impulseForce * contactNormal;
}
void Contact::applyVelocityChange() {
  std::array<sf::Vector2f, 2> velocityChange;
  std::array<float, 2> angularVelocityChange;
  sf::Vector2f impulse = calculateFrictionlessImpulse();
  float impulsiveTorque =
      cross(contactPoint - bodies[0].get().getPosition(), impulse);
  angularVelocityChange[0] =
      bodies[0].get().getInverseInertia() * impulsiveTorque;
  velocityChange[0] = impulse * bodies[0].get().getInverseMass();
  bodies[0].get().addVelocity(velocityChange[0]);
  bodies[0].get().addAngularVelocity(angularVelocityChange[0]);

  float impulsiveTorque2 =
      cross(contactPoint - bodies[1].get().getPosition(), -impulse);
  angularVelocityChange[1] =
      bodies[1].get().getInverseInertia() * impulsiveTorque2;
  velocityChange[1] = -impulse * bodies[1].get().getInverseMass();
  bodies[1].get().addVelocity(velocityChange[1]);
  bodies[1].get().addAngularVelocity(angularVelocityChange[1]);
}
void Contact::applyVelocityChangeSphereSphere() {
  std::array<sf::Vector2f, 2> velocityChange;
  sf::Vector2f impulse = calculateFrictionlessImpulseSphereSphere();
  velocityChange[0] = impulse * bodies[0].get().getInverseMass();
  bodies[0].get().addVelocity(velocityChange[0]);
  velocityChange[1] = -impulse * bodies[1].get().getInverseMass();
  bodies[1].get().addVelocity(velocityChange[1]);
}

void Contact::applyPositionChange(std::array<sf::Vector2f, 2> &displacement) {
  float totalInverseMass =
      bodies[0].get().getInverseMass() + bodies[1].get().getInverseMass();
  displacement[0] = contactNormal * penetrationDepth *
                    bodies[0].get().getInverseMass() / totalInverseMass;
  displacement[1] = -contactNormal * penetrationDepth *
                    bodies[1].get().getInverseMass() / totalInverseMass;
  bodies[0].get().addDisplacement(displacement[0]);
  bodies[1].get().addDisplacement(displacement[1]);
}