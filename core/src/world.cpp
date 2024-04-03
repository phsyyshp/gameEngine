#include "world.hpp"
void World::startFrame() {
  for (auto &body : bodies) {
    body.clearAccumulators();
  }
}

void World::runPhysics(float deltaTime) {

  for (auto &body : bodies) {
    body.integrate(deltaTime);
  }
}