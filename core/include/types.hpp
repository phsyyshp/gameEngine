#include <SFML/Graphics.hpp>
struct Contact {
  Contact(RigidBody2D &a, RigidBody2D &b) : bodies{a, b} {}
  sf::Vector2f contactPoint;
  sf::Vector2f contactNormal;
  float penetrationDepth;
  std::array<std::reference_wrapper<RigidBody2D>, 2> bodies;
  float friction;
  float resitution;
};
class CollisionData {
public:
  CollisionData() { contacts.reserve(256); }
  std::vector<Contact> contacts;
};