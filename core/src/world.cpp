#include "world.hpp"
#include "rigidBody2D.hpp"
#include <memory>
void World::startFrame() {
  // for (auto &body : bodies) {
  //   body.clearAccumulators();
  // }
}
void World::runPhysics(float deltaTime, int subStep) {

  findContacts();
  if (Collider::applySleepScheme) {
    solveIslands(deltaTime);
  }
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
    if (!manifold.getBodyA().isAwake() && !manifold.getBodyB().isAwake()) {
      continue;
    }
    for (auto &contact : manifold.getContacts()) {
      manifold.preStep(contact, deltaTime);
    }
  }
  float totalChange = 0;
  float lastChange = 0;

  float changeRatio = 0;
  float maxChangeRatio = 0;
  float maxChange = 0;
  float maxTotalChange = 0;
  int i = 0;
  do {
    maxChangeRatio = 0;
    // FIX IT: The break condition is not proper enough make it more roboust
    for (auto &[key, manifold] : manifolds) {
      if (!manifold.getBodyA().isAwake() && !manifold.getBodyB().isAwake()) {
        continue;
      }
      for (auto &contact : manifold.getContacts()) {
        auto lagrangianMultiplier = manifold.solveImpulse(contact, deltaTime);

        lastChange = lagrangianMultiplier;
        totalChange = contact.totalNormalImpulse;

        // How you treat totalChange==0 case gravely impacts the stability.
        changeRatio =
            (totalChange == 0) ? 0 : std::abs(lastChange) / (totalChange);
        if (changeRatio > maxChangeRatio) {
          maxChangeRatio = changeRatio;
          maxChange = lastChange;
          maxTotalChange = totalChange;
        }
      }
    }
    // std::cout << "Max Total change is: " << maxTotalChange
    //           << " last change is: " << lastChange << " iteration: " << i
    //           << " MAX change ratio is: " << maxChangeRatio
    //           << " Max change: " << maxChange << "\n";
    i++;
    // std::cout << "total it" << i << "\n";
    // } while (i < 10);
  } while (maxChangeRatio > 0.001F && i < 50);
  // } while (std::abs(lastChange) / (totalChange) > 0.001F && i < 1000);

  // std::cout << "Max Total change is: " << totalChange
  //           << " last change is: " << lastChange << " iteration: " << i
  //           << " MAX change ratio is: " << maxChangeRatio
  //           << " Max change: " << maxChange << "\n";
  for (auto &body : bodies) {
    if (!body->isAwake()) {
      continue;
    }
    body->integrateVelocities(deltaTime);
  }
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
      // if ((!bodies[i]->isAwake()) && (!bodies[j]->isAwake())) {
      //   continue;
      // }
      Manifold manifold(bodies[i].get(), bodies[j].get());
      ManifoldKey manifoldKey(bodies[i].get(), bodies[j].get());
      if (manifold.size() > 0) {
        auto manIt = manifolds.find(manifoldKey);
        if (manIt == manifolds.end()) {
          manifolds.insert(std::make_pair(manifoldKey, manifold));
        } else {
          manIt->second.update(manifold.getContacts());
        }
      } else {
        manifolds.erase(manifoldKey);
      }
    }
  }
}
void World::solveIslands(float deltaTime) {
  for (auto &body : bodies) {
    body->clearMark();
  }
  for (auto &[key, manifold] : manifolds) {
    manifold.clearMark();
  }
  Island island;
  for (auto &body : bodies) {
    if ((!body->isMarked()) && body->isDynamic() && body->isAwake()) {
      island.clear();
      body->mark();
      std::vector<RigidBody2D *> stack;
      stack.push_back(body.get());
      while (!stack.empty()) {
        RigidBody2D *tempBody = stack.back();
        island.add(tempBody);
        stack.pop_back();
        if (!tempBody->isDynamic()) {
          continue;
        }
        for (auto &manifoldOfBody : getAllManifolds(*tempBody)) {
          if (!manifoldOfBody->isMarked()) {
            island.add(manifoldOfBody);
            manifoldOfBody->mark();
            RigidBody2D *other = &(manifoldOfBody->getOtherBody(tempBody));
            if (!other->isMarked()) {
              other->wakeUp();
              other->mark();
              stack.push_back(other);
            }
          }
        }
      }
      island.simulate(deltaTime);
    }
  }
}
std::vector<Manifold *> World::getAllManifolds(const RigidBody2D &body) {
  std::vector<Manifold *> manifoldsOfBody;
  for (auto &[key, manifold] : manifolds) {
    if (manifold.getBodyA() == body || manifold.getBodyB() == body) {
      manifoldsOfBody.push_back(&manifold);
    }
  }
  return manifoldsOfBody;
}

std::map<ManifoldKey, Manifold> &World::getManifolds() { return manifolds; }