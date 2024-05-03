#include "contactResolver.hpp"
#include "forceGeneration.hpp"
#include "rigidBody2D.hpp"
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Sleep.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/System/Vector2.hpp>
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
  const std::map<ManifoldKey, Manifold> &getManifolds() const;

  void findContacts();
  void resolveCollisions();
  void setSleepers();

  void setWindow(sf::RenderWindow &window);
  void showContacts(std::map<ManifoldKey, Manifold> &cd);
  sf::RenderWindow *window;

private:
  std::vector<std::unique_ptr<RigidBody2D>> bodies;
  ContactResolver contactResolver;
  std::vector<Gravity> gravity;
  std::map<ManifoldKey, Manifold> manifolds;
  bool isDebug = false;
};