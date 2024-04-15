#include "contactResolver.hpp"
#include "collisionHandler.hpp"
#include "contact.hpp"
#include "rigidBody2D.hpp"
#include <SFML/System/Vector2.hpp>
#include <array>
#include <limits>

void ContactResolver::resolveContacts(CollisionData &collisionData,
                                      float deltaTime) {

  sequentialImpulse(collisionData, deltaTime);
}

void ContactResolver::resolvePenetration(CollisionData &collisionData) {

  std::vector<Contact> &contacts = collisionData.contacts;
  int positionsChecked = 0;
  //   int i = collisionData.size();
  //   while (positionsChecked <= contacts.size() * 10) {
  for (int i = 0; i < collisionData.size(); i++) {
    if ((!contacts[i].getBodies()[0].get().isAwake()) &&
        (!contacts[i].getBodies()[1].get().isAwake())) {
      continue;
    }

    // for (int index = 0; index < contacts.size(); index++) {
    //   float max = 0.f;
    //   if (contacts[index].getPenetrationDepth() > max) {
    //     max = contacts[index].getPenetrationDepth();
    //     i = index;
    //   }
    // }
    // if (i == collisionData.size()) {
    //   break;
    // }
    std::array<sf::Vector2f, 2> displacement;
    contacts[i].applyPositionChange(displacement);
    // for (int j = 0; j < collisionData.size(); j++) {
    //   for (int bdyIdx = 0; bdyIdx < 2; bdyIdx++) {
    //     for (int bdyIdx2 = 0; bdyIdx2 < 2; bdyIdx2++) {
    //       if (contacts[i].getBodies()[bdyIdx].get().getPosition() ==
    //           contacts[j].getBodies()[bdyIdx2].get().getPosition()) {
    //         contacts[j].setPenetrationDepth(
    //             contacts[j].getPenetrationDepth() +
    //             (bdyIdx2 ? 1 : -1) *
    //                 dot(contacts[j].getContactNormal(),
    //                 displacement[bdyIdx2]));
    //       }
    //     }
    //   }
    // }
    // positionsChecked++;
  }
}
void ContactResolver::sequentialImpulse(CollisionData &collisionData,
                                        float deltaTime) {
  float totalChange = 0;
  int i = 0;
  // std::cout << collisionData.size() << "\n";
  do {
    totalChange = 0;
    for (auto &contact : collisionData.contacts) {
      float totalImpulseNormal = contact.getTotalImpulseNormal();
      float lagrangianMultiplier = contact.solveContactConstraints(deltaTime);
      float oldImpulseNormal = totalImpulseNormal;
      totalImpulseNormal = std::clamp(lagrangianMultiplier + oldImpulseNormal,
                                      0.F, std::numeric_limits<float>::max());
      lagrangianMultiplier = totalImpulseNormal - oldImpulseNormal;
      contact.setTotalImpulseNormal(totalImpulseNormal);
      contact.applyVelocityChange(lagrangianMultiplier);
      totalChange += lagrangianMultiplier;
    }
    // std::cout << totalChange << "\n";
    i++;
  } while (totalChange > 0.001F && i < 30);
  std::cout << totalChange << "itNo" << i << "\n";
}