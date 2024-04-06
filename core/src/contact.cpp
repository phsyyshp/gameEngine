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

void Contact::calculateDesiredVelocty() {
  std::array<sf::Vector2f, 2> velocity;
  std::array<sf::Vector2f, 2> relativeContactPosition;
  relativeContactPosition[0] = contactPoint - bodies[0].get().getPosition();
  relativeContactPosition[1] = contactPoint - bodies[1].get().getPosition();
  velocity[0] = bodies[0].get().getVelocity() +
                bodies[0].get().getAngularVelocity() *
                    perpendicular(relativeContactPosition[0]);
  sf::Vector2f contactVelocity;
  contactVelocity.x = dot(velocity[0], contactNormal);
  contactVelocity.y = dot(velocity[0], perpendicular(contactNormal));

  if (bodies[1].get().getPosition() != emptyBody.getPosition()) {
    velocity[1] = bodies[1].get().getVelocity() +
                  bodies[1].get().getAngularVelocity() *
                      perpendicular(relativeContactPosition[1]);

    contactVelocity.x -= dot(velocity[1], contactNormal);
    contactVelocity.y -= dot(velocity[1], perpendicular(contactNormal));
  }
  desiredDeltaVelocity = -contactVelocity.x * (1 + resitution);
}
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
sf::Vector2f Contact::connectToWorld(const sf::Vector2f &connect) const {
  sf::Vector2f world;
  world.x =
      connect.x * contactNormal.x + connect.y * perpendicular(contactNormal).x;
  world.y =
      connect.x * contactNormal.y + connect.y * perpendicular(contactNormal).y;
  return world;
}

sf::Vector2f Contact::calculateFrictionlessImpulse() {
  sf::Vector2f impulseContact;
  std::array<sf::Vector2f, 2> velocity;
  std::array<sf::Vector2f, 2> relativeContactPosition;
  relativeContactPosition[0] = contactPoint - bodies[0].get().getPosition();
  relativeContactPosition[1] = contactPoint - bodies[1].get().getPosition();

  // Build a scalar that shows the change in velocity in
  // world space for a unit impulse in the direction of the contact
  // normal.
  float impulsiveTorque = cross(relativeContactPosition[0], contactNormal);
  float angularVelocity = bodies[0].get().getInverseInertia() * impulsiveTorque;
  sf::Vector2f deltaVelWorld = perpendicular(relativeContactPosition[0]);
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
    velocity[1] = bodies[1].get().getVelocity() +
                  bodies[1].get().getAngularVelocity() *
                      perpendicular(relativeContactPosition[1]);
    deltaVelocity += dot(deltaVelWorld, contactNormal);
    // Add the linear component of velocity change.
    deltaVelocity += bodies[1].get().getInverseMass();
  }
  impulseContact.x = desiredDeltaVelocity / deltaVelocity;
  impulseContact.y = 0;
  return impulseContact;
}
void Contact::applyVelocityChange() {
  std::array<sf::Vector2f, 2> velocityChange;
  std::array<float, 2> rotationChange;
  calculateDesiredVelocty();
  sf::Vector2f impulseContact = calculateFrictionlessImpulse();
  sf::Vector2f impulse = connectToWorld(impulseContact);
  float impulsiveTorque =
      cross(contactPoint - bodies[0].get().getPosition(), impulse);
  rotationChange[0] = bodies[0].get().getInverseInertia() * impulsiveTorque;
  velocityChange[0] = impulse * bodies[0].get().getInverseMass();
  bodies[0].get().addVelocity(velocityChange[0]);
  bodies[0].get().addAngularVelocity(rotationChange[0]);
  if (bodies[1].get().getPosition() != emptyBody.getPosition()) {

    float impulsiveTorque2 =
        cross(contactPoint - bodies[1].get().getPosition(), -impulse);
    rotationChange[1] = bodies[1].get().getInverseInertia() * impulsiveTorque2;
    velocityChange[1] = -impulse * bodies[1].get().getInverseMass();
    bodies[1].get().addVelocity(velocityChange[1]);
    bodies[1].get().addAngularVelocity(rotationChange[1]);
  }
}
void Contact::applyImpulse(float deltaTime) {
  sf::Vector2f impulseContact = calculateFrictionlessImpulse();
  sf::Vector2f impulse = connectToWorld(impulseContact);
  float impulsiveTorque =
      cross(contactPoint - bodies[0].get().getPosition(), impulse);
  bodies[0].get().addForceAtPoint(impulse / deltaTime, contactPoint);
  if (bodies[1].get().getPosition() != emptyBody.getPosition()) {
    bodies[1].get().addForceAtPoint(-impulse / deltaTime, contactPoint);
  }
}