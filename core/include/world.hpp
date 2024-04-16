#include "collisionHandler.hpp"
#include "contactResolver.hpp"
#include "forceGeneration.hpp"
#include "rigidBody2D.hpp"
#include <SFML/Graphics/RenderWindow.hpp>
class World {
public:
  World(int bodyCount = 200, bool isDebug_ = false) : isDebug(isDebug_) {
    boxes.reserve(bodyCount);
    circles.reserve(bodyCount);
  }

  void startFrame();
  void runPhysics(float deltaTime, int subStep = 1);
  void registerBody(Box &body);
  void registerBody(Circle &body);
  void registerGravity(Gravity &forceGenerator);

  std::vector<Circle> &getCircles();
  std::vector<Box> &getBoxes();
  size_t getBodySize() const;
  const CollisionData &getCollisionData() const;

  void findContacts();
  void resolveCollisions();
  void setSleepers();
  void warmStart(CollisionData &collisionData);
  // void regiterForceGenerator(ForceGenerator &forceGenerator);

  void setWindow(sf::RenderWindow &window);
  void showContacts(CollisionData &cd);
  void findContactsWarmStart();
  sf::RenderWindow *window;

private:
  std::vector<Box> boxes;
  std::vector<Circle> circles;
  CollisionData collisionData;
  ContactResolver contactResolver;
  std::vector<Gravity> gravity;
  bool isDebug = false;
};