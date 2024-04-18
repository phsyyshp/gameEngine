#pragma once
#include "contact.hpp"
#include "rigidBody2D.hpp"
#include "shapes.hpp"
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Vector2.hpp>
#include <cmath>
class ContactManifold {
public:
  ContactManifold() {}
  void push_back(const Contact &contact) { contacts.push_back(contact); }
  std::vector<Contact> &getContacts() { return contacts; }

private:
  std::vector<Contact> contacts;
};
class CollisionData {
public:
  CollisionData() { contactManifolds.reserve(256); }

  // Getters;
  std::vector<ContactManifold> &getContactManifolds() {
    return contactManifolds;
  }
  [[nodiscard]] size_t size() const { return contactManifolds.size(); }

  // Modifiers;
  void push_back(const ContactManifold &contactManifold) {
    contactManifolds.push_back(contactManifold);
  }
  void clear() { contactManifolds.clear(); }
  bool isEmpty() { return contactManifolds.empty(); }
  void
  setContactManifolds(const std::vector<ContactManifold> &contactManifolds) {
    this->contactManifolds = contactManifolds;
  }

private:
  std::vector<ContactManifold> contactManifolds;
};

class Collider {
public:
  static bool sphereAndSphere(Circle &, Circle &, CollisionData &);
  static bool sphereAndRectangle(Circle &, Box &, CollisionData &);
  static bool rectangleAndRectangle(Box &, Box &, CollisionData &);
  static bool genericCollision(RigidBody2D &, RigidBody2D &, CollisionData &);
  static sf::Vector2f getSupportP(const std::vector<sf::Vector2f> &vertices,
                                  const sf::Vector2f &direction);
  static sf::Vector2f getSupportS(const Circle &circle,
                                  const sf::Vector2f &direction);

  static bool GJKintersectionPP(Box &shapeA, Box &shapeB,
                                std::vector<sf::Vector2f> &simplex);
  static float findContactNormalPenetration(std::vector<sf::Vector2f> &simplex,
                                            Box &shapeA, Box &shapeB,
                                            sf::Vector2f &normal_);
  static bool polygonPolygon(Box &shapeA, Box &shapeB,
                             CollisionData &collisionData);
  static bool GJKintersectionSP(const Circle &circle,
                                const std::vector<sf::Vector2f> &verticesB);
  static bool nearestSimplex(std::vector<sf::Vector2f> &simplex,
                             sf::Vector2f &direction, const Box &shapeA,
                             const Box &shapeB);
  static bool findContactManifold(Box &shapeA, Box &shapeB,
                                  CollisionData &collisionData,
                                  sf::RenderWindow *window);
  static void clip(std::vector<sf::Vector2f> &polygonToClip,
                   const std::array<sf::Vector2f, 2> &edge,
                   sf::RenderWindow *window, bool createNewPoint = true);
};