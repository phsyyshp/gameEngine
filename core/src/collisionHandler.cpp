#include "collisionHandler.hpp"
bool Collider::sphereAndSphere(Circle &a, Circle &b,
                               CollisionData &collisionData) {

  sf::Vector2f positionA = a.RigidBody2D::getPosition();
  sf::Vector2f positionB = b.RigidBody2D::getPosition();
  sf::Vector2f midLine = positionA - positionB;
  float distance = std::sqrt(midLine.x * midLine.x + midLine.y * midLine.y);
  if (distance <= 0.0f || distance >= a.getRadius() + b.getRadius()) {
    return false;
  }
  sf::Vector2f normal = midLine / distance;
  Contact contact(a, b);

  contact.setContactNormal(normal);
  contact.setContactPoint(positionA + midLine * 0.5f);
  contact.setPenetrationDepth(a.getRadius() + b.getRadius() - distance);
  collisionData.contacts.push_back(contact);
  return true;
}
