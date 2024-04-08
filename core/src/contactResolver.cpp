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
  for (int i = 0; i < collisionData.size(); i++) {

    std::array<sf::Vector2f, 2> displacement;
    contacts[i].applyPositionChange(displacement);
    for (int j = i + 1; j < collisionData.size(); j++) {
      for (int bdyIdx = 0; bdyIdx < 2; bdyIdx++) {
        for (int bdyIdx2 = 0; bdyIdx2 < 2; bdyIdx2++) {
          if (contacts[i].getBodies()[bdyIdx].get().getPosition() ==
              contacts[j].getBodies()[bdyIdx2].get().getPosition()) {
            contacts[j].setPenetrationDepth(
                contacts[j].getPenetrationDepth() +
                (bdyIdx2 ? 1 : -1) *
                    dot(contacts[j].getContactNormal(), displacement[bdyIdx2]));
          }
        }
      }
    }
  }
}