#include "collisionHandler.hpp"
#include "rigidBody2D.hpp"
struct ManifoldKey {
public:
  ManifoldKey(RigidBody2D *a, RigidBody2D *b)
      //   : bodyA((&a < &b) ? a : b), bodyB((&a < &b) ? b : a) {}
      : bodyA(a), bodyB(b) {}
  RigidBody2D *bodyA;
  RigidBody2D *bodyB;
};

// FIX IT: there may be nasty bugs to due arbitrary ordering Manifold.
inline bool operator<(const ManifoldKey &a1, const ManifoldKey &a2) {
  if ((a1.bodyA) < (a2.bodyA)) {
    return true;
  }

  if (a1.bodyA == a2.bodyA && a1.bodyB < a2.bodyB) {
    return true;
  }

  return false;
}
class Manifold {
public:
  // Constructor now takes pointers instead of references.
  Manifold(RigidBody2D *a, RigidBody2D *b) : bodyA(a), bodyB(b) {
    Collider::collide(*bodyA, *bodyB, contacts);
  }
  // Manifold(Manifold &) = delete;

  RigidBody2D &getBodyA() const;
  RigidBody2D &getBodyB() const;
  RigidBody2D &getOtherBody(RigidBody2D *body) const;
  std::vector<Contact> &getContacts();
  size_t size() const;

  void push_back(const Contact &contact);
  void update(std::vector<Contact> &newContacts);
  void preStep(Contact &contact, float deltaTime);
  void applyVelocityChange(float lagrangianMultiplier, Contact &contact);
  float solveContactConstraints(Contact &contact, float deltaTime);

  bool isMarked() const;
  void mark();
  void clearMark();

private:
  std::vector<Contact> contacts;
  RigidBody2D *bodyA;
  RigidBody2D *bodyB;
  bool isMarked_;
};
class Island {
public:
  void clear() {
    bodies.clear();
    manifolds.clear();
  }
  void add(RigidBody2D *body) {
    // body->wakeUp();
    bodies.push_back(body);
  }
  void add(Manifold *manifold) { manifolds.push_back(manifold); }
  void sleep(float deltaTime) {
    std::cout << "size of island: " << bodies.size() << "\n";
    std::cout << "size of manifolds" << manifolds.size() << "\n";
    float velocityThreshold = 55.5f;
    float timeToSleep = deltaTime * 10;
    float minSleepTime = std::numeric_limits<float>::max();
    for (auto &body : bodies) {
      if (!body->isDynamic()) {
        body->sleep();
        continue;
      };
      if (magnitude(body->getVelocity()) > velocityThreshold) {
        body->sleepTime = 0.F;
        minSleepTime = 0.F;
      } else {
        // std::cout << "checking sleep\n";

        body->sleepTime += deltaTime;
        minSleepTime = std::min(minSleepTime, body->sleepTime);
      }
    }
    if (minSleepTime > timeToSleep) {
      for (auto &body : bodies) {
        body->sleep();
        // std::cout << "sleepin\n";
      }
    }
  }
  void simulate(float deltaTime) { sleep(deltaTime); }

private:
  std::vector<RigidBody2D *> bodies;
  std::vector<Manifold *> manifolds;
};