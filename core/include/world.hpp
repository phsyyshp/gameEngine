#include "forceGeneration.hpp"
#include "rigidBody2D.hpp"
class World {
public:
  World(int bodyCount = 200) { bodies.reserve(bodyCount); }
  void startFrame();
  void runPhysics(float deltaTime);
  void registerBody(RigidBody2D &body);
  // void regiterForceGenerator(ForceGenerator &forceGenerator);

private:
  std::vector<RigidBody2D> bodies;
  // std::vector<ForceGenerator> forceGenerators;
};