#pragma once
#include "collisionHandler.hpp"
class ContactResolver {
public:
  void resolveContacts(CollisionData &, float);
  void resolvePenetration(CollisionData &);
  void applyVelocityChange(Contact &contact);
  void applyPositionChange(Contact &contact);
  void sequentialImpulse(std::map<ManifoldKey, ContactManifold> manifolds,
                         float deltaTime);
};