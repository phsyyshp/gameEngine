#include "contact.hpp"
#include "rigidBody2D.hpp"
// getters
sf::Vector2f Contact::getContactNormal() const { return contactNormal; }
float Contact::getPenetrationDepth() const { return penetrationDepth; }
float Contact::getFriction() const { return friction; }
float Contact::getResitution() const { return resitution; }
std::array<sf::Vector2f, 2> Contact::getContactPosition() const {
  return contactPosition;
}
float Contact::getTotalImpulseNormal() { return normalImpulseSum; }
std::array<sf::Vector2f, 2> Contact::getLocalContactPosition() const {
  return contactPositionLocal;
}
void Contact::setContactPosition(const std::array<sf::Vector2f, 2> &rp,
                                 const RigidBody2D &bodyA,
                                 const RigidBody2D &bodyB) {
  contactPosition = rp;
  contactPositionLocal = {transformToCordinateSystem(rp[0], bodyA.getPosition(),
                                                     bodyA.getOrientation()),
                          transformToCordinateSystem(rp[1], bodyB.getPosition(),
                                                     bodyB.getOrientation())};
}
// setters
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
void Contact::makePersistent() { persistent = true; }
bool Contact::isPersistent() const { return persistent; }
