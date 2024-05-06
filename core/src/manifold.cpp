#include "manifold.hpp"
// Constructor now takes pointers instead of references.
RigidBody2D &Manifold::getBodyA() const { return *bodyA; }
RigidBody2D &Manifold::getBodyB() const { return *bodyB; }
RigidBody2D &Manifold::getOtherBody(RigidBody2D *body) const {
  if (body == bodyA) {
    return *bodyB;
  }
  return *bodyA;
}

std::vector<Contact> &Manifold::getContacts() { return contacts; }
size_t Manifold::size() const { return contacts.size(); }
void Manifold::push_back(const Contact &contact) {
  contacts.push_back(contact);
}
void Manifold::update(std::vector<Contact> &newContacts) {
  // contacts = newContacts;
  float absTol = 3.F;
  for (auto &contact : contacts) {
    for (auto &newContact : newContacts) {
      float positionDifference =
          magnitude(contact.position - newContact.position);
      if (std::abs(positionDifference) < absTol) {

        if (Collider::warmStart) {
          newContact.totalNormalImpulse = contact.totalNormalImpulse;
          newContact.totalTangentImpulse = contact.totalTangentImpulse;

        } else {
          newContact.totalNormalImpulse = 0.F;
          newContact.totalTangentImpulse = 0.F;
        }
      }
    }
  }
  contacts = newContacts;
}
void Manifold::preStep(Contact &contact, float deltaTime) {
  float beta = 0.2F;
  float slop = 0.01F;
  contact.relativeContactPositionA = contact.position - bodyA->getPosition();
  contact.relativeContactPositionB = contact.position - bodyB->getPosition();
  float totalInverseMass = bodyA->getInverseMass() + bodyB->getInverseMass();
  // update contacts
  float bias =
      -beta / deltaTime * std::max(contact.penetrationDepth - slop, 0.F);
  float deminatorNormal =
      std::pow(cross(contact.relativeContactPositionA, contact.normal), 2) *
          bodyA->getInverseInertia() +
      std::pow(cross(contact.relativeContactPositionB, contact.normal), 2) *
          bodyB->getInverseInertia() +
      totalInverseMass;
  la::Vector contactTangent = -perpendicular(contact.normal);
  float deminatorTangent =
      std::pow(cross(contact.relativeContactPositionA, contactTangent), 2) *
          bodyA->getInverseInertia() +
      std::pow(cross(contact.relativeContactPositionB, contactTangent), 2) *
          bodyB->getInverseInertia() +
      totalInverseMass;
  contact.bias = bias;
  contact.massNormal = 1.F / deminatorNormal;
  contact.massTangent = 1.F / deminatorTangent;

  // update the bodies
  if (Collider::accumulateImpulse) {
    la::Vector impulse = contact.totalNormalImpulse * contact.normal +
                         contact.totalTangentImpulse * contactTangent;

    bodyA->addVelocity(bodyA->getInverseMass() * impulse);
    bodyA->addAngularVelocity(cross(contact.relativeContactPositionA, impulse) *
                              bodyA->getInverseInertia());
    bodyB->addVelocity(-bodyB->getInverseMass() * impulse);
    bodyB->addAngularVelocity(
        cross(-contact.relativeContactPositionB, impulse) *
        bodyB->getInverseInertia());
  }
}

float Manifold::solveImpulse(Contact &contact, float deltaTime) {
  std::array<float, 2> angularComponent;
  contact.relativeContactPositionA = contact.position - bodyA->getPosition();
  contact.relativeContactPositionB = contact.position - bodyB->getPosition();

  la::Vector relativeContactVelocity =
      bodyA->getVelocity() - bodyB->getVelocity() +
      perpendicular(contact.relativeContactPositionA) *
          bodyA->getAngularSpeed() -
      perpendicular(contact.relativeContactPositionB) *
          bodyB->getAngularSpeed();

  float normalImpulse =
      -(dot(relativeContactVelocity, contact.normal) + contact.bias) *
      contact.massNormal;

  if (Collider::accumulateImpulse) {
    float oldImpulseNormal = contact.totalNormalImpulse;
    contact.totalNormalImpulse =
        std::max(oldImpulseNormal + normalImpulse, 0.0F);
    normalImpulse = contact.totalNormalImpulse - oldImpulseNormal;
  } else {
    normalImpulse = std::max(normalImpulse, 0.F);
  }
  // apply velocity change for normal force
  bodyA->addVelocity(contact.normal * bodyA->getInverseMass() * normalImpulse);
  bodyB->addVelocity(-contact.normal * bodyB->getInverseMass() * normalImpulse);
  bodyA->addAngularVelocity(
      cross(contact.relativeContactPositionA, contact.normal) *
      bodyA->getInverseInertia() * normalImpulse);
  bodyB->addAngularVelocity(
      cross(-contact.relativeContactPositionB, contact.normal) *
      bodyB->getInverseInertia() * normalImpulse);

  // same calculation for tangent
  relativeContactVelocity = bodyA->getVelocity() - bodyB->getVelocity() +
                            perpendicular(contact.relativeContactPositionA) *
                                bodyA->getAngularSpeed() -
                            perpendicular(contact.relativeContactPositionB) *
                                bodyB->getAngularSpeed();
  la::Vector contactTangent = -perpendicular(contact.normal);
  float tangentImpulse =
      -(dot(relativeContactVelocity, contactTangent)) * contact.massTangent;
  if (Collider::accumulateImpulse) {
    float maxTangentImpulse = contact.friction * contact.totalNormalImpulse;
    float oldTangentImpulse = contact.totalTangentImpulse;
    contact.totalTangentImpulse =
        std::clamp(oldTangentImpulse + tangentImpulse, -maxTangentImpulse,
                   maxTangentImpulse);
    tangentImpulse = contact.totalTangentImpulse - oldTangentImpulse;
  } else {
    float maxTangentImpulse = contact.friction * contact.totalNormalImpulse;
    tangentImpulse =
        std::clamp(tangentImpulse, -maxTangentImpulse, maxTangentImpulse);
  }

  bodyA->addVelocity(contactTangent * bodyA->getInverseMass() * tangentImpulse);
  bodyB->addVelocity(-contactTangent * bodyB->getInverseMass() *
                     tangentImpulse);
  bodyA->addAngularVelocity(
      cross(contact.relativeContactPositionA, contactTangent) *
      bodyA->getInverseInertia() * tangentImpulse);
  bodyB->addAngularVelocity(
      cross(-contact.relativeContactPositionB, contactTangent) *
      bodyB->getInverseInertia() * tangentImpulse);
  return normalImpulse;
}

bool Manifold::isMarked() const { return isMarked_; }
void Manifold::mark() { isMarked_ = true; }
void Manifold::clearMark() { isMarked_ = false; }
