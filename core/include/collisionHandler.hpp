#include "contact.hpp"
#include "shapes.hpp"
#include <cmath>
class CollisionData {
public:
  CollisionData() { contacts.reserve(256); }
  std::vector<Contact> contacts;
  void clear() { contacts.clear(); }
};
class Collider {
public:
  static bool sphereAndSphere(Circle &, Circle &, CollisionData &);
  static bool sphereAndRectangle(Circle &, Box &, CollisionData &);
};
