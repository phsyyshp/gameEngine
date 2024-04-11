#include "collisionHandler.hpp"
#include "contact.hpp"
#include <SFML/System/Vector2.hpp>
bool Collider::sphereAndSphere(Circle &a, Circle &b,
                               CollisionData &collisionData) {

  if (a.getInverseMass() == 0 && b.getInverseMass() == 0) {
    return false;
  }
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
bool Collider::sphereAndRectangle(Circle &circle, Box &box,
                                  CollisionData &collisionData) {
  // Transform the centre of the sphere into box coordinates
  sf::Vector2f circleCenter = circle.RigidBody2D::getPosition();
  sf::Vector2f boxCenter = box.RigidBody2D::getPosition();
  sf::Vector2f relCenter = transformToCordinateSystem(
      circleCenter, boxCenter, box.RigidBody2D::getOrientation());
  // relCenter = circleCenter - boxCenter;

  // Early out check to see if we can exclude the contact
  if (std::abs(relCenter.x) - circle.getRadius() > box.getHalfSize().x ||
      std::abs(relCenter.y) - circle.getRadius() > box.getHalfSize().y) {
    return false;
  }
  sf::Vector2f closestPoint(0, 0);
  // Clamp each coordinate to the box.
  closestPoint.x =
      std::clamp(relCenter.x, -box.getHalfSize().x, box.getHalfSize().x);
  closestPoint.y =
      std::clamp(relCenter.y, -box.getHalfSize().y, box.getHalfSize().y);
  // Check we're in contact
  sf::Vector2f cp2center = closestPoint - relCenter;
  float distance = cp2center.x * cp2center.x + cp2center.y * cp2center.y;
  if (distance > circle.getRadius() * circle.getRadius()) {
    return false;
  }

  // Compile the contact
  sf::Vector2f closestPointWorld = inverseTransformToCordinateSystem(
      closestPoint, boxCenter, box.RigidBody2D::getOrientation());
  // closestPointWorld = closestPoint + boxCenter;
  Contact contact(circle, box);
  sf::Vector2f contactNormal = normalise(-closestPointWorld + circleCenter);
  contact.setContactNormal(contactNormal);
  contact.setContactPoint(closestPointWorld);
  contact.setPenetrationDepth(circle.getRadius() - std::sqrt(distance));
  collisionData.contacts.push_back(contact);
  return true;
}
bool Collider::rectangleAndRectangle(Box &boxA, Box &boxB,
                                     CollisionData &collisionData) {
  if (boxA.getInverseMass() == 0 && boxB.getInverseMass() == 0) {
    return false;
  }
  // get axes of Rectangle
  sf::Vector2f positionA = boxA.RigidBody2D::getPosition();
  sf::Vector2f halfSize = boxA.getHalfSize();
  std::array<sf::Vector2f, 2> axesA =
      getBaseCoordinateSystem(boxA.getOrientation());
  sf::Vector2f positionB = boxB.RigidBody2D::getPosition();
  sf::Vector2f halfSizeB = boxB.getHalfSize();
  std::array<sf::Vector2f, 2> axesB =
      getBaseCoordinateSystem(boxB.getOrientation());
  sf::Vector2f distanceVector = positionA - positionB;
  float minOverlap = 10'000;
  sf::Vector2f smallestAxis;
  bool overLapCondition;
  for (int i = 0; i < 2; i++) {
    sf::Vector2f axis = axesA[i];
    overLapCondition =
        absDot(axis, distanceVector) > (absDot(axesA[0] * halfSize.x, axis) +
                                        absDot(axesA[1] * halfSize.y, axis) +
                                        absDot(axesB[0] * halfSizeB.x, axis) +
                                        absDot(axesB[1] * halfSizeB.y, axis));

    if (overLapCondition) {
      return false;
    } else {
      float overlap = std::abs(absDot(axis, distanceVector) -
                               (absDot(axesA[0] * halfSize.x, axis) +
                                absDot(axesA[1] * halfSize.y, axis) +
                                absDot(axesB[0] * halfSizeB.x, axis) +
                                absDot(axesB[1] * halfSizeB.y, axis)));
      if (overlap < minOverlap) {
        minOverlap = overlap;
        smallestAxis = axis;
      }
    }
    axis = axesB[i];
    if (overLapCondition) {
      return false;
    } else {
      float overlap = std::abs(absDot(axis, distanceVector) -
                               (absDot(axesA[0] * halfSize.x, axis) +
                                absDot(axesA[1] * halfSize.y, axis) +
                                absDot(axesB[0] * halfSizeB.x, axis) +
                                absDot(axesB[1] * halfSizeB.y, axis)));
      if (overlap < minOverlap) {
        minOverlap = overlap;
        smallestAxis = axis;
      }
    }
  }
  Contact contact(boxA, boxB);
  sf::Vector2f normal = normalise(smallestAxis);
  contact.setPenetrationDepth(minOverlap);
  sf::Vector2f closestPoint;
  std::array<sf::Vector2f, 4> verticesB = boxB.getVertices();
  std::array<sf::Vector2f, 4> verticesA = boxA.getVertices();
  bool isTouch = false;
  for (auto vertex : verticesB) {
    if (boxA.isPointIn(vertex)) {
      contact.setContactPoint(vertex);
      isTouch = true;
      if (!boxA.isPointIn(normal * (1.001f) + vertex)) {
        normal = -normal;
      }
      contact.setContactNormal(normal);
      collisionData.contacts.push_back(contact);
    }
  }
  for (auto vertex : verticesA) {
    if (boxB.isPointIn(vertex)) {
      isTouch = true;
      Contact contact2(boxA, boxB);
      contact2.setContactPoint(vertex);
      contact2.setPenetrationDepth(minOverlap);
      if (boxB.isPointIn(normal * (1.001f) + vertex)) {
        normal = -normal;
      }
      contact2.setContactNormal(normal);
      collisionData.contacts.push_back(contact2);
    }
  }

  return isTouch;
}
