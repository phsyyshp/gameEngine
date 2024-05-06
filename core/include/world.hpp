#include "forceGeneration.hpp"
#include "manifold.hpp"
#include "rigidBody2D.hpp"
#include <map>
#include <memory>
class World {
public:
  World(int bodyCount = 200, bool isDebug_ = false) : isDebug(isDebug_) {
    bodies.reserve(bodyCount);
  }

  void startFrame();
  void runPhysics(float deltaTime, int subStep = 1);
  void registerBody(std::unique_ptr<RigidBody2D> body);
  void registerGravity(Gravity &forceGenerator);
  std::vector<std::unique_ptr<RigidBody2D>> &getBodies();
  size_t getBodySize() const;
  std::map<ManifoldKey, Manifold> &getManifolds();

  std::vector<Manifold *> getAllManifolds(const RigidBody2D &body);

  void findContacts();
  void solveIslands(float deltaTime);
  void setSleepers();

private:
  std::vector<std::unique_ptr<RigidBody2D>> bodies;
  std::vector<Gravity> gravity;
  std::map<ManifoldKey, Manifold> manifolds;
  bool isDebug = false;
};