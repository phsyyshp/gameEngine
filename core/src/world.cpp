#include "world.hpp"
#include "collisionHandler.hpp"
#include "contact.hpp"
#include "rigidBody2D.hpp"
#include "shapes.hpp"
#include <SFML/System/Sleep.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/System/Vector2.hpp>
void World::startFrame() {
  // for (auto &body : bodies) {
  //   body.clearAccumulators();
  // }
}
void World::runPhysics(float deltaTime, int subStep) {

  for (int i = 0; i < subStep; i++) {

    // setSleepers();
    // findContacts();
    CollisionData collisionData_ = collisionData;
    warmStart(collisionData_);

    contactResolver.resolveContacts(collisionData, deltaTime / subStep);
    if (isDebug) {
      showContacts(collisionData);
    }
    // collisionData.clear();
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
      Collider::findContactManifold(boxes[i], boxes[j], collisionData, window);
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

void World::showContacts(CollisionData &cd) {
  for (auto &contactManifold : cd.getContactManifolds()) {

    for (auto &contact : contactManifold.getContacts()) {
      float radius = 2.f;

      sf::CircleShape ax(radius);
      auto contactPoint = contact.getContactPosition()[0];
      ax.setFillColor(sf::Color::Green);
      ax.setPosition(contactPoint);
      ax.setOrigin(radius, radius);
      window->draw(ax);
      std::array<sf::Vertex, 2> line = {
          sf::Vertex(contactPoint, sf::Color::Green),
          sf::Vertex(contactPoint + contact.getContactNormal() * 10.f,
                     sf::Color::Green),
      };
      window->draw(line.data(), 2, sf::Lines);
    }
  }
}
void World::setWindow(sf::RenderWindow &window) { this->window = &window; }

void World::warmStart(CollisionData &collisionData_) {
  // collisionData.clear();
  findContacts();
  if (collisionData_.isEmpty()) {
    return;
  }
  for (int i = 0; i < collisionData_.getContactManifolds().size(); i++) {
    ContactManifold &manifold = collisionData_.getContactManifolds()[i];
    for (int j = 0; j < manifold.getContacts().size(); j++) {
      Contact &contact = manifold.getContacts()[j];
      contact.getLocalContactPosition();
      auto globalA = contact.getBodies()[0].get().localToGlobal(
          contact.getLocalContactPosition()[0]);
      auto globalB = contact.getBodies()[1].get().localToGlobal(
          contact.getLocalContactPosition()[1]);
      globalA - globalB;
      if (dot(globalA - globalB, contact.getContactNormal()) <= 0) {
        contact.makePersistent();
        // sf::sleep(sf::seconds(2.0f)); // Convert integer to float

        // std::cout << "lala" << std::endl;
      } else {

        manifold.getContacts().erase(manifold.getContacts().begin() + j);
      }
    }
    if (manifold.size() == 0) {
      collisionData_.getContactManifolds().erase(
          collisionData_.getContactManifolds().begin() + i);
    }
  }
  collisionData.setContactManifolds(collisionData_.getContactManifolds());
}
