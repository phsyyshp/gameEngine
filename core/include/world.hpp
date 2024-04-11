#include "collisionHandler.hpp"
#include "contactResolver.hpp"
#include "forceGeneration.hpp"
#include "rigidBody2D.hpp"
class World {
public:
  World(int bodyCount = 200) {
    boxes.reserve(bodyCount);
    circles.reserve(bodyCount);
  }

  void startFrame();
  void runPhysics(float deltaTime, int subStep = 1);
  void registerBody(Box &body);
  void registerBody(Circle &body);
  void registerForce(ForceGenerator &forceGenerator);

  void findContacts();
  void resolveCollisions();
  // void regiterForceGenerator(ForceGenerator &forceGenerator);

private:
  std::vector<Box> boxes;
  std::vector<Circle> circles;
  CollisionData collisionData;
  ContactResolver contactResolver;
  std::vector<ForceGenerator> forceGenerators;
};