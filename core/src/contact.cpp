#include "contact.hpp"
// getters
sf::Vector2f Contact::getContactNormal() const { return contactNormal; }
float Contact::getPenetrationDepth() const { return penetrationDepth; }
float Contact::getFriction() const { return friction; }
float Contact::getResitution() const { return resitution; }
sf::Vector2f Contact::getContactPosition() const { return contactPosition; }
float Contact::getTotalImpulseNormal() { return normalImpulseSum; }
float Contact::getBias() { return bias; }
// setters
void Contact::setContactNormal(const sf::Vector2f &contactNormal_) {
  contactNormal = contactNormal_;
}
void Contact::setContactPosition(const sf::Vector2f &contactPosition_) {
  this->contactPosition = contactPosition_;
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
void Contact::setBias(float bias_) { bias = bias_; }
