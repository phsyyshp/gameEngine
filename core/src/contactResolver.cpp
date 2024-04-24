#include "contactResolver.hpp"
#include "contact.hpp"
#include <SFML/System/Vector2.hpp>
#include <array>
#include <limits>

void ContactResolver::sequentialImpulse(
    std::map<ManifoldKey, Manifold> manifolds, float deltaTime) {
  float totalChange = 0;
  int i = 0;
  do {
    for (auto &[key, manifold] : manifolds) {
      for (auto &contact : manifold.getContacts()) {
        float lagrangianMultiplier =
            manifold.solveContactConstraints(contact, deltaTime);

        float totalImpulseNormal = contact.getTotalImpulseNormal();
        float oldImpulseNormal = totalImpulseNormal;
        totalImpulseNormal = std::clamp(lagrangianMultiplier + oldImpulseNormal,
                                        0.F, std::numeric_limits<float>::max());
        lagrangianMultiplier = totalImpulseNormal - oldImpulseNormal;
        contact.setTotalImpulseNormal(totalImpulseNormal);
        manifold.applyVelocityChange(lagrangianMultiplier, contact);
        totalChange = lagrangianMultiplier;
      }
    }
    i++;
    std::cout << "total change is " << totalChange << "i is " << i << "\n";
  } while (totalChange > 1.2F && i < 1000);
}