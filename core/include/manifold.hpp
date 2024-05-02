#include "collisionHandler.hpp"
struct ManifoldKey {
public:
  ManifoldKey(RigidBody2D &a, RigidBody2D &b)
      //   : bodyA((&a < &b) ? a : b), bodyB((&a < &b) ? b : a) {}
      : bodyA(a), bodyB(b) {}
  RigidBody2D &bodyA;
  RigidBody2D &bodyB;
};

inline bool operator<(const ManifoldKey &a1, const ManifoldKey &a2) {
  if (&(a1.bodyA) < &(a2.bodyA)) {
    return true;
  }

  if (&a1.bodyA == &a2.bodyA && &a1.bodyB < &a2.bodyB) {
    return true;
  }

  return false;
}
class Manifold {
public:
  // Constructor now takes pointers instead of references.
  Manifold(RigidBody2D &a, RigidBody2D &b) : bodyA(a), bodyB(b) {
    Collider::collide(bodyA, bodyB, contacts);
  }
  RigidBody2D &getBodyA() const { return bodyA; }
  RigidBody2D &getBodyB() const { return bodyB; }
  std::vector<Contact> &getContacts() { return contacts; }
  size_t size() const { return contacts.size(); }

  void push_back(const Contact &contact) { contacts.push_back(contact); }

  void update(std::vector<Contact> &newContacts) {
    // contacts = newContacts;
    float threshold = 1.f;
    for (auto &contact : contacts) {
      for (auto &newContact : newContacts) {
        float positionDifference = magnitude(contact.getContactPosition() -
                                             newContact.getContactPosition());
        if (std::abs(positionDifference) < threshold &&
            newContact.getPenetrationDepth() > 0.F) {
          if (Collider::warmStart) {

            newContact.setTotalImpulseNormal(contact.getTotalImpulseNormal());
          } else {
            newContact.setTotalImpulseNormal(0.F);
          }
        }
      }
    }
    contacts = newContacts;
  }

  void preStep(Contact &contact, float deltaTime) {

    std::array<sf::Vector2f, 2> relativeContactPosition;
    sf::Vector2f contactNormal = contact.getContactNormal();
    auto contactPosition = contact.getContactPosition();
    relativeContactPosition[0] = contactPosition - bodyA.getPosition();
    relativeContactPosition[1] = contactPosition - bodyB.getPosition();
    float bias = 0;
    float beta = 0.2F;
    float slop = 0.01F;

    bias =
        -beta / deltaTime * std::max(contact.getPenetrationDepth() - slop, 0.F);
    float lagrangianMultiplier = contact.getTotalImpulseNormal();
    contact.setBias(bias);

    if (Collider::accumulateImpulse) {

      bodyA.addVelocity(contactNormal * bodyA.getInverseMass() *
                        lagrangianMultiplier);
      bodyA.addAngularVelocity(
          cross(relativeContactPosition[0], contactNormal) *
          bodyA.getInverseInertia() * lagrangianMultiplier);
      bodyB.addVelocity(-contactNormal * bodyB.getInverseMass() *
                        lagrangianMultiplier);
      bodyB.addAngularVelocity(
          cross(-relativeContactPosition[1], contactNormal) *
          bodyB.getInverseInertia() * lagrangianMultiplier);
    }
  }
  void applyVelocityChange(float lagrangianMultiplier, Contact &contact) {
    if (Collider::accumulateImpulse) {
      float oldImpulseNormal = contact.getTotalImpulseNormal();

      contact.setTotalImpulseNormal(
          std::max(oldImpulseNormal + lagrangianMultiplier, 0.0F));
      lagrangianMultiplier = contact.getTotalImpulseNormal() - oldImpulseNormal;
    } else {
      lagrangianMultiplier = std::max(lagrangianMultiplier, 0.F);
    }

    std::array<sf::Vector2f, 2> relativeContactPosition;
    sf::Vector2f contactNormal = contact.getContactNormal();
    auto contactPosition = contact.getContactPosition();
    relativeContactPosition[0] = contactPosition - bodyA.getPosition();
    relativeContactPosition[1] = contactPosition - bodyB.getPosition();
    bodyA.addVelocity(contactNormal * bodyA.getInverseMass() *
                      lagrangianMultiplier);
    bodyB.addVelocity(-contactNormal * bodyB.getInverseMass() *
                      lagrangianMultiplier);
    bodyA.addAngularVelocity(cross(relativeContactPosition[0], contactNormal) *
                             bodyA.getInverseInertia() * lagrangianMultiplier);
    bodyB.addAngularVelocity(cross(-relativeContactPosition[1], contactNormal) *
                             bodyB.getInverseInertia() * lagrangianMultiplier);
  }

  float solveContactConstraints(Contact &contact, float deltaTime) {
    std::array<sf::Vector2f, 2> relativeContactPosition;
    sf::Vector2f contactNormal = contact.getContactNormal();
    auto contactPosition = contact.getContactPosition();

    std::array<float, 2> angularComponent;
    relativeContactPosition[0] = contactPosition - bodyA.getPosition();
    relativeContactPosition[1] = contactPosition - bodyB.getPosition();
    float totalInverseMass = bodyA.getInverseMass() + bodyB.getInverseMass();
    std::array<sf::Vector2f, 2> velocity = {bodyA.getVelocity(),
                                            bodyB.getVelocity()};
    std::array<float, 2> angularSpeed = {bodyA.getAngularVelocity(),
                                         bodyB.getAngularVelocity()};
    std::array<sf::Vector2f, 2> angularVelocity = {
        perpendicular(relativeContactPosition[0]) * angularSpeed[0],
        perpendicular(relativeContactPosition[1]) * angularSpeed[1]};
    angularComponent[0] = cross(relativeContactPosition[0], contactNormal) *
                          cross(relativeContactPosition[0], contactNormal) *
                          bodyA.getInverseInertia();
    angularComponent[1] = cross(relativeContactPosition[1], contactNormal) *
                          cross(relativeContactPosition[1], contactNormal) *
                          bodyB.getInverseInertia();
    float deminator =
        angularComponent[0] + angularComponent[1] + totalInverseMass;
    // bias+=resitution*()

    float lagrangianMultiplier =
        -(dot(velocity[0] - velocity[1] + angularVelocity[0] -
                  angularVelocity[1],
              contactNormal) +
          contact.getBias()) /
        deminator;
    return lagrangianMultiplier;
  }

private:
  std::vector<Contact> contacts;
  RigidBody2D &bodyA;
  RigidBody2D &bodyB;
};