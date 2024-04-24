#include "world.hpp"
#include "collisionHandler.hpp"
#include "contact.hpp"
#include "rigidBody2D.hpp"
#include <SFML/System/Sleep.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/System/Vector2.hpp>
#include <memory>
void World::startFrame() {
  // for (auto &body : bodies) {
  //   body.clearAccumulators();
  // }
}
void World::runPhysics(float deltaTime, int subStep) {

  // setSleepers();
  findContacts();

  for (auto &body : bodies) {
    if (!body->isAwake()) {
      continue;
    }
    for (auto &forceGenerator : gravity) {
      forceGenerator.updateForce(*body, deltaTime);
      body->integrateForces(deltaTime);
    }
  }
  for (auto &[key, manifold] : manifolds) {
    for (auto &contact : manifold.getContacts()) {
      manifold.preStep(contact, deltaTime);
    }
  }
  float totalChange = 0;
  int i = 0;
  do {
    for (auto &[key, manifold] : manifolds) {
      for (auto &contact : manifold.getContacts()) {
        float lagrangianMultiplier =
            manifold.solveContactConstraints(contact, deltaTime);
        manifold.applyVelocityChange(lagrangianMultiplier, contact);
      }
    }
  } while (totalChange > 1.2F && i < 1000);
  for (auto &body : bodies) {
    body->integrateVelocities(deltaTime);
  }

  if (isDebug) {
    showContacts(manifolds);
  }
  // collisionData.clear();
}
void World::registerBody(std::unique_ptr<RigidBody2D> body) {
  bodies.push_back(std::move(body));
}
std::vector<std::unique_ptr<RigidBody2D>> &World::getBodies() { return bodies; }
size_t World::getBodySize() const { return bodies.size(); }
void World::registerGravity(Gravity &forceGenerator) {
  gravity.push_back(forceGenerator);
}
void World::findContacts() {
  for (int i = 0; i < bodies.size(); i++) {
    for (int j = i + 1; j < bodies.size(); j++) {
      if (bodies[i]->getInverseMass() == 0 &&
          bodies[j]->getInverseMass() == 0) {
        continue;
      }
      Manifold manifold(*bodies[i], *bodies[j]);
      ManifoldKey manifoldKey(*bodies[i], *bodies[j]);
      // std::cout << "manifold size is: " << manifold.size() << std::endl;
      if (manifold.size() > 0) {
        std::cout << "checking stuff" << std::endl;
        auto manIt = manifolds.find(manifoldKey);
        if (manIt == manifolds.end()) {

          std::cout << "new manifold" << std::endl;
          manifolds.insert(std::make_pair(manifoldKey, manifold));
        } else {

          std::cout << "existing manifold" << std::endl;
          manIt->second.update(manifold.getContacts());
        }
      } else {
        manifolds.erase(manifoldKey);
      }
    }
  }
}
void World::setSleepers() {
  for (auto &body : bodies) {
    if (body->isAwake()) {
      if (magnitude(body->getVelocity()) < 15.5f) {
        if (body->getAwakeTimer() < 10) {
          body->setAwakeTimer(body->getAwakeTimer() + 1);
        } else {
          body->sleep();
        }
      }
    }
  }
}
void World::showContacts(std::map<ManifoldKey, Manifold> &manifolds) {
  for (auto &[key, manifold] : manifolds) {

    for (auto &contact : manifold.getContacts()) {
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
