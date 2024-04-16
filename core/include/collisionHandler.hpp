#pragma once
#include "contact.hpp"
#include "rigidBody2D.hpp"
#include "shapes.hpp"
#include <SFML/System/Vector2.hpp>
#include <cmath>
class CollisionData {
public:
  CollisionData() { contacts.reserve(256); }
  std::vector<Contact> contacts;
  void clear() { contacts.clear(); }
  [[nodiscard]] size_t size() const { return contacts.size(); }
};
class Collider {
public:
  static bool sphereAndSphere(const Circle &, const Circle &, CollisionData &);
  static bool sphereAndRectangle(const Circle &, const Box &, CollisionData &);
  static bool rectangleAndRectangle(const Box &, const Box &, CollisionData &);
  static bool genericCollision(const RigidBody2D &, const RigidBody2D &,
                               CollisionData &);
  static sf::Vector2f getSupportP(const std::vector<sf::Vector2f> &vertices,
                                  const sf::Vector2f &direction);
  static sf::Vector2f getSupportS(const Circle &circle,
                                  const sf::Vector2f &direction);

  static bool GJKintersectionPP(Box &shapeA, Box &shapeB, CollisionData &cd);
  static bool GJKintersectionSP(const Circle &circle,
                                const std::vector<sf::Vector2f> &verticesB);
  static bool nearestSimplex(std::vector<sf::Vector2f> &simplex,
                             sf::Vector2f &direction, const Box &shapeA,
                             const Box &shapeB);
};
