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

void Contact::setTotalImpulseNormal(float totalImpulseNormal_) {
  normalImpulseSum = totalImpulseNormal_;
}
float Contact::getTotalImpulseNormal() { return normalImpulseSum; }
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
void Contact::applyVelocityChange(float lagrangianMultiplier) {
  RigidBody2D &bodyA = bodies[0].get();
  RigidBody2D &bodyB = bodies[1].get();
  std::array<sf::Vector2f, 2> relativeContactPosition;
  relativeContactPosition[0] = contactPoint - bodies[0].get().getPosition();
  relativeContactPosition[1] = contactPoint - bodies[1].get().getPosition();
  bodies[0].get().addVelocity(contactNormal * bodyA.getInverseMass() *
                              lagrangianMultiplier);
  bodies[1].get().addVelocity(-contactNormal * bodyB.getInverseMass() *
                              lagrangianMultiplier);
  bodies[0].get().addAngularVelocity(
      cross(relativeContactPosition[0], contactNormal) *
      bodyA.getInverseInertia() * lagrangianMultiplier);
  bodies[1].get().addAngularVelocity(
      cross(-relativeContactPosition[1], contactNormal) *
      bodyB.getInverseInertia() * lagrangianMultiplier);
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

float Contact::solveContactConstraints(float deltaTime) {
  RigidBody2D &bodyA = bodies[0].get();
  RigidBody2D &bodyB = bodies[1].get();
  std::array<sf::Vector2f, 2> relativeContactPosition;
  std::array<float, 2> angularComponent;

  relativeContactPosition[0] = contactPoint - bodies[0].get().getPosition();
  relativeContactPosition[1] = contactPoint - bodies[1].get().getPosition();
  float totalInverseMass =
      bodies[0].get().getInverseMass() + bodies[1].get().getInverseMass();
  std::array<sf::Vector2f, 2> velocity = {bodyA.getVelocity(),
                                          bodyB.getVelocity()};
  std::array<float, 2> angularSpeed = {bodyA.getAngularVelocity(),
                                       bodyB.getAngularVelocity()};
  std::array<sf::Vector2f, 2> angularVelocity = {
      perpendicular(relativeContactPosition[0]) * angularSpeed[0],
      perpendicular(relativeContactPosition[1]) * angularSpeed[1]};
  angularComponent[0] = cross(relativeContactPosition[0], contactNormal) *
                        cross(relativeContactPosition[0], contactNormal) *
                        bodies[0].get().getInverseInertia();
  angularComponent[1] = cross(relativeContactPosition[1], contactNormal) *
                        cross(relativeContactPosition[1], contactNormal) *
                        bodies[1].get().getInverseInertia();
  float deminator =
      angularComponent[0] + angularComponent[1] + totalInverseMass;

  float bias = 0;
  float beta = 0.016F;
  bias = -beta / deltaTime * penetrationDepth;

  float lagrangianMultiplier =
      -(dot(velocity[0] - velocity[1] + angularVelocity[0] - angularVelocity[1],
            contactNormal) +
        bias) /
      deminator;
  return lagrangianMultiplier;
}