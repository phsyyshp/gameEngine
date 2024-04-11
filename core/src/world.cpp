#include "world.hpp"
void World::startFrame() {
  // for (auto &body : bodies) {
  //   body.clearAccumulators();
  // }
}
void World::runPhysics(float deltaTime, int subStep) {
  for (int i = 0; i < subStep; i++) {
    findContacts();
    contactResolver.resolveContacts(collisionData, deltaTime / subStep);
    collisionData.clear();
    for (auto &body : circles) {
      for (auto &forceGenerator : forceGenerators) {
        forceGenerator.updateForce(body, deltaTime / subStep);
      }
      body.integrate(deltaTime / subStep);
    }
    for (auto &body : boxes) {
      for (auto &forceGenerator : forceGenerators) {
        forceGenerator.updateForce(body, deltaTime / subStep);
      }
      body.integrate(deltaTime / subStep);
    }
  }
}

void World::registerBody(Box &body) { boxes.push_back(body); }
void World::registerBody(Circle &body) { circles.push_back(body); }
void World::registerForce(ForceGenerator &forceGenerator) {
  forceGenerators.push_back(forceGenerator);
}

void World::findContacts() {
  for (int i = 0; i < circles.size(); i++) {
    for (int j = i + 1; j < circles.size(); j++) {
      Collider::sphereAndSphere(circles[i], circles[j], collisionData);
    }
    for (auto &box : boxes) {
      Collider::sphereAndRectangle(circles[i], box, collisionData);
    }
  }
  for (int i = 0; i < boxes.size(); i++) {
    for (int j = i + 1; j < boxes.size(); j++) {
      Collider::rectangleAndRectangle(boxes[i], boxes[j], collisionData);
    }
  }
}