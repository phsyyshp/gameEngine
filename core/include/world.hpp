#include "rigidBody2D.hpp"
class World {
  void startFrame();
  void runPhysics(float deltaTime);

private:
  std::vector<RigidBody2D> bodies;
};