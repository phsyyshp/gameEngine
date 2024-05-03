#include "collisionHandler.hpp"
struct ManifoldKey {
public:
  ManifoldKey(RigidBody2D &a, RigidBody2D &b)
      //   : bodyA((&a < &b) ? a : b), bodyB((&a < &b) ? b : a) {}
      : bodyA(a), bodyB(b) {}
  RigidBody2D &bodyA;
  RigidBody2D &bodyB;
};

inline bool operator<(const ManifoldKey &a1, const ManifoldKey &a2) {
  if (&(a1.bodyA) < &(a2.bodyA)) {
    return true;
  }

  if (&a1.bodyA == &a2.bodyA && &a1.bodyB < &a2.bodyB) {
    return true;
  }

  return false;
}
class Manifold {
public:
  // Constructor now takes pointers instead of references.
  Manifold(RigidBody2D &a, RigidBody2D &b) : bodyA(a), bodyB(b) {
    Collider::collide(bodyA, bodyB, contacts);
  }
  RigidBody2D &getBodyA() const;
  RigidBody2D &getBodyB() const;
  std::vector<Contact> &getContacts();
  size_t size() const;

  void push_back(const Contact &contact);
  void update(std::vector<Contact> &newContacts);
  void preStep(Contact &contact, float deltaTime);
  void applyVelocityChange(float lagrangianMultiplier, Contact &contact);
  float solveContactConstraints(Contact &contact, float deltaTime);

private:
  std::vector<Contact> contacts;
  RigidBody2D &bodyA;
  RigidBody2D &bodyB;
};
class Island {

private:
  std::vector<std::unique_ptr<RigidBody2D>> bodies;
  std::vector<std::unique_ptr<Contact>> contacts;
};