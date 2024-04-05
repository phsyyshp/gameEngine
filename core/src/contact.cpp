#include "contact.hpp"
RigidBody2D Contact::emptyBody(-80, 80);
// getters
sf::Vector2f Contact::getContactPoint() const { return contactPoint; }
sf::Vector2f Contact::getContactNormal() const { return contactNormal; }
float Contact::getPenetrationDepth() const { return penetrationDepth; }
std::array<std::reference_wrapper<RigidBody2D>, 2> Contact::getBodies() const {
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
sf::Vector2f Contact::calculateFrictionlessImpulse() const {
  sf::Vector2f impulseContact;
  std::array<sf::Vector2f, 2> relativeContactPosition;
  relativeContactPosition[0] = contactPoint - bodies[0].get().getPosition();
  relativeContactPosition[1] = contactPoint - bodies[1].get().getPosition();

  // Build a scalar that shows the change in velocity in
  // world space for a unit impulse in the direction of the contact
  // normal.
  float impulsiveTorque = cross(relativeContactPosition[0], contactNormal);
  float angularVelocity = bodies[0].get().getInverseInertia() * impulsiveTorque;
  sf::Vector2f deltaVelWorld = {-relativeContactPosition[0].y,
                                relativeContactPosition[0].x};
  deltaVelWorld *= angularVelocity;

  // Work out the change in velocity in contact coordinates.
  float deltaVelocity = dot(deltaVelWorld, contactNormal);
  // Add the linear component of velocity change.
  deltaVelocity += bodies[0].get().getInverseMass();
  // Check whether we need to consider the second body’s data.
  if (bodies[1].get().getPosition() != emptyBody.getPosition()) {
    // Go through the same transformation sequence again.
    float impulsiveTorque = cross(relativeContactPosition[1], contactNormal);
    float angularVelocity =
        bodies[1].get().getInverseInertia() * impulsiveTorque;
    sf::Vector2f deltaVelWorld = {-relativeContactPosition[1].y,
                                  relativeContactPosition[1].x};
    deltaVelWorld *= angularVelocity;
    deltaVelocity += dot(deltaVelWorld, contactNormal);
    // Add the linear component of velocity change.
    deltaVelocity += bodies[1].get().getInverseMass();
  }
  impulseContact.x = desiredDeltaVelocity / deltaVelocity;
  impulseContact.y = 0;
  return impulseContact;
}
void Contact::applyImpulse() {
  sf::Vector2f impulseContact = calculateFrictionlessImpulse();
  sf::Vector2f impulse = contactNormal * impulseContact.x;
  float impulsiveTorque =
      cross(contactPoint - bodies[0].get().getPosition(), impulse);
  bodies[0].get().addForceAtPoint(impulse, contactPoint);
  if (bodies[1].get().getPosition() != emptyBody.getPosition()) {
    bodies[1].get().addForceAtPoint(-impulse, contactPoint);
  }
}