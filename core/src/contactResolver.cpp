#include "contactResolver.hpp"

void ContactResolver::resolveContacts(CollisionData &collisionData,
                                      float deltaTime) {

  resolvePenetration(collisionData);
  for (auto &contact : collisionData.contacts) {
    contact.applyVelocityChange();
  }
}
void ContactResolver::resolvePenetration(CollisionData &collisionData) {

  std::vector<Contact> &contacts = collisionData.contacts;
  int positionsChecked = 0;
  //   int i = collisionData.size();
  //   while (positionsChecked <= contacts.size() * 10) {
  for (int i = 0; i < collisionData.size(); i++) {
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