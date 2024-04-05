#include "contact.hpp"
#include "shapes.hpp"
#include <cmath>
class CollisionData {
public:
  CollisionData() { contacts.reserve(256); }
  std::vector<Contact> contacts;
};
class Collider {
public:
  static bool sphereAndSphere(Circle &, Circle &, CollisionData &);
};
