#include "world.hpp"
#include "rigidBody2D.hpp"
#include "shapes.hpp"
void World::startFrame() {
  // for (auto &body : bodies) {
  //   body.clearAccumulators();
  // }
}
void World::runPhysics(float deltaTime, int subStep) {

  for (int i = 0; i < subStep; i++) {

    // setSleepers();
    findContacts();

    contactResolver.resolveContacts(collisionData, deltaTime / subStep);
    if (isDebug) {
      showContacts(collisionData);
    }
    collisionData.clear();
    for (auto &body : circles) {
      if (body.isAwake()) {
        for (auto &forceGenerator : gravity) {
          forceGenerator.updateForce(body, deltaTime / subStep);
        }
        body.integrate(deltaTime / subStep);
      }
    }
    for (auto &body : boxes) {
      if (body.isAwake()) {
        for (auto &forceGenerator : gravity) {
          forceGenerator.updateForce(body, deltaTime / subStep);
        }
        body.integrate(deltaTime / subStep);
      }
    }
  }
}

void World::registerBody(Box &body) { boxes.push_back(body); }
void World::registerBody(Circle &body) { circles.push_back(body); }
size_t World::getBodySize() const { return boxes.size() + circles.size(); }
void World::registerGravity(Gravity &forceGenerator) {
  gravity.push_back(forceGenerator);
}
std::vector<Circle> &World::getCircles() { return circles; }
std::vector<Box> &World::getBoxes() { return boxes; }

const CollisionData &World::getCollisionData() const { return collisionData; }
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
void World::setSleepers() {
  for (auto &body : circles) {
    if (body.isAwake()) {
      if (magnitude(body.getVelocity()) < 15.5f) {
        if (body.getAwakeTimer() < 10) {
          body.setAwakeTimer(body.getAwakeTimer() + 1);
        } else {
          body.sleep();
        }
      }
    }
  }
  for (auto &body : boxes) {
    if (body.isAwake()) {
      if (magnitude(body.getVelocity()) < 15.5f) {
        if (body.getAwakeTimer() < 10) {
          body.setAwakeTimer(body.getAwakeTimer() + 1);
        } else {
          body.sleep();
        }
      }
    }
  }
}

void World::showContacts(const CollisionData &cd) {
  for (auto &contact : cd.contacts) {
    float radius = 2.f;
    sf::CircleShape ax(radius);
    ax.setFillColor(sf::Color::Green);
    ax.setPosition(contact.getContactPoint());
    ax.setOrigin(radius, radius);
    window->draw(ax);
    std::array<sf::Vertex, 2> line = {
        sf::Vertex(contact.getContactPoint(), sf::Color::Green),
        sf::Vertex(contact.getContactPoint() +
                       contact.getContactNormal() * 10.f,
                   sf::Color::Green),
    };
    window->draw(line.data(), 2, sf::Lines);
  }
}
void World::setWindow(sf::RenderWindow &window) { this->window = &window; }
