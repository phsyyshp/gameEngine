#pragma once
#include "contact.hpp"
#include "rigidBody2D.hpp"
#include "shapes.hpp"
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Vector2.hpp>
#include <cmath>

struct ManifoldKey {
public:
  ManifoldKey(RigidBody2D &a, RigidBody2D &b)
      : bodyA((&a < &b) ? a : b), bodyB((&a < &b) ? b : a) {}

  RigidBody2D &bodyA;
  RigidBody2D &bodyB;
};

inline bool operator<(const ManifoldKey &a1, const ManifoldKey &a2) {
  if (&(a1.bodyA) < &(a2.bodyA))
    return true;

  if (&a1.bodyA == &a2.bodyA && &a1.bodyB < &a2.bodyB)
    return true;

  return false;
}
class Collider {
public:
  static bool sphereAndSphere(Circle &, Circle &, std::vector<Contact> &);
  static bool sphereAndRectangle(Circle &, Box &, std::vector<Contact> &);
  static bool rectangleAndRectangle(Box &, Box &, std::vector<Contact> &);
  static bool genericCollision(Box &bodyA, Box &bodyB,
                               std::vector<Contact> &contacts);
  static bool genericCollision(Circle &bodyA, Box &bodyB,
                               std::vector<Contact> &contacts);
  static bool genericCollision(Box &bodyA, Circle &bodyB,
                               std::vector<Contact> &contacts);
  static bool genericCollision(Circle &bodyA, Circle &bodyB,
                               std::vector<Contact> &contacts);
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
                             std::vector<Contact> &collisionData,
                             sf::RenderWindow *window);
  static bool GJKintersectionSP(const Circle &circle,
                                const std::vector<sf::Vector2f> &verticesB);
  static bool nearestSimplex(std::vector<sf::Vector2f> &simplex,
                             sf::Vector2f &direction, const Box &shapeA,
                             const Box &shapeB);
  static bool findContactManifold(Box &shapeA, Box &shapeB,
                                  std::vector<Contact> &collisionData);
  static void clip(std::vector<sf::Vector2f> &polygonToClip,
                   const std::array<sf::Vector2f, 2> &edge,
                   bool createNewPoint = true);
  static bool collide(RigidBody2D &bodyA, RigidBody2D &bodyB,
                      std::vector<Contact> &contacts);
};
class ContactManifold {
public:
  // Constructor now takes pointers instead of references.
  ContactManifold(RigidBody2D *a, RigidBody2D *b) {
    if (a == nullptr || b == nullptr) {
      throw std::invalid_argument(
          "Null pointer passed to ContactManifold constructor");
    }
    bodyA = (a < b) ? a : b;
    bodyB = (a < b) ? b : a;
    Collider::collide(*bodyA, *bodyB, contacts);
  }
  // Custom copy constructor
  ContactManifold(const ContactManifold &other)
      : bodyA(other.bodyA), bodyB(other.bodyB), contacts(other.contacts) {}
  // Custom assignment operator
  ContactManifold &operator=(const ContactManifold &other) {
    if (this != &other) {
      bodyA = other.bodyA;
      bodyB = other.bodyB;
      contacts = other.contacts;
    }
    return *this;
  }
  RigidBody2D *getBodyA() const { return bodyA; }
  RigidBody2D *getBodyB() const { return bodyB; }
  std::vector<Contact> &getContacts() { return contacts; }
  size_t size() const { return contacts.size(); }

  void push_back(const Contact &contact) { contacts.push_back(contact); }

  void update(std::vector<Contact> &newContacts) {
    float threshold = 100.f;
    for (int i = 0; i < contacts.size(); i++) {
      for (int j = 0; j < newContacts.size(); j++) {
        if (std::abs(magnitude(contacts[i].getContactPosition()[0] -
                               newContacts[j].getContactPosition()[0])) <
                threshold &&
            std::abs(magnitude(contacts[i].getContactPosition()[1] -
                               newContacts[j].getContactPosition()[1])) <
                threshold) {
          newContacts[j].setTotalImpulseNormal(
              contacts[i].getTotalImpulseNormal());
          newContacts[j].makePersistent();
        }
      }
    }
    contacts = newContacts;
  }

private:
  std::vector<Contact> contacts;
  RigidBody2D *bodyA;
  RigidBody2D *bodyB;
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
