#include "collider.hpp"
bool sphereAndSphere(Circle &a, Circle &b, CollisionData &collisionData) {

  sf::Vector2f positionA = a.getPosition();
  sf::Vector2f positionB = b.getPosition();
  sf::Vector2f midLine = positionA - positionB;
  float distance = std::sqrt(midLine.x * midLine.x + midLine.y * midLine.y);
  if (distance <= 0.0f || distance >= a.getRadius() + b.getRadius()) {
    return false;
  }
  sf::Vector2f normal = midLine / distance;
  Contact contact(a, b);

  contact.contactNormal = normal;
  contact.contactPoint = positionA + midLine * 0.5f;
  contact.penetrationDepth = a.getRadius() + b.getRadius() - distance;
  collisionData.contacts.push_back(contact);
  return true;
}
