#include "collisionHandler.hpp"
#include "contact.hpp"
#include "utils.hpp"
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Event.hpp>
#include <cfloat>
#include <cstdint>
#include <limits>
#include <vector>
bool Collider::sphereAndSphere(Circle &a, Circle &b,
                               CollisionData &collisionData) {
  if (!(a.isAwake()) && !(b.isAwake())) {
    return false;
  }
  if (a.getInverseMass() == 0 && b.getInverseMass() == 0) {
    return false;
  }
  sf::Vector2f positionA = a.RigidBody2D::getPosition();
  sf::Vector2f positionB = b.RigidBody2D::getPosition();
  sf::Vector2f midLine = positionA - positionB;
  float distance = std::sqrt(midLine.x * midLine.x + midLine.y * midLine.y);
  if (distance <= 0.0f || distance >= a.getRadius() + b.getRadius()) {
    return false;
  }
  sf::Vector2f normal = midLine / distance;
  Contact contact(a, b);

  contact.setContactNormal(normal);
  contact.setContactPosition(
      {positionA - normal * a.getRadius(), positionB + normal * b.getRadius()});

  contact.setPenetrationDepth(a.getRadius() + b.getRadius() - distance);
  ContactManifold contactManifold;
  contactManifold.push_back(contact);
  collisionData.push_back(contactManifold);
  return true;
}
bool Collider::sphereAndRectangle(Circle &circle, Box &box,
                                  CollisionData &collisionData) {
  if (!(circle.isAwake()) && !(box.isAwake())) {
    return false;
  }
  // Transform the centre of the sphere into box coordinates
  sf::Vector2f circleCenter = circle.RigidBody2D::getPosition();
  sf::Vector2f boxCenter = box.RigidBody2D::getPosition();
  sf::Vector2f relCenter = transformToCordinateSystem(
      circleCenter, boxCenter, box.RigidBody2D::getOrientation());
  // relCenter = circleCenter - boxCenter;

  // Early out check to see if we can exclude the contact
  if (std::abs(relCenter.x) - circle.getRadius() > box.getHalfSize().x ||
      std::abs(relCenter.y) - circle.getRadius() > box.getHalfSize().y) {
    return false;
  }
  sf::Vector2f closestPoint(0, 0);
  // Clamp each coordinate to the box.
  closestPoint.x =
      std::clamp(relCenter.x, -box.getHalfSize().x, box.getHalfSize().x);
  closestPoint.y =
      std::clamp(relCenter.y, -box.getHalfSize().y, box.getHalfSize().y);
  // Check we're in contact
  sf::Vector2f cp2center = closestPoint - relCenter;
  float distance = cp2center.x * cp2center.x + cp2center.y * cp2center.y;
  if (distance > circle.getRadius() * circle.getRadius()) {
    return false;
  }

  // Compile the contact
  sf::Vector2f closestPointWorld = inverseTransformToCordinateSystem(
      closestPoint, boxCenter, box.RigidBody2D::getOrientation());
  // closestPointWorld = closestPoint + boxCenter;
  Contact contact(circle, box);
  sf::Vector2f contactNormal = normalise(-closestPointWorld + circleCenter);
  contact.setContactNormal(contactNormal);
  contact.setPenetrationDepth(circle.getRadius() - std::sqrt(distance));
  contact.setContactPosition(
      {-circle.getRadius() * contactNormal + circleCenter, closestPointWorld});
  ContactManifold contactManifold;
  contactManifold.push_back(contact);
  collisionData.push_back(contactManifold);
  return true;
}
// bool Collider::rectangleAndRectangle(Box &boxA, Box &boxB,
//                                      CollisionData &collisionData) {
//   if (!(boxA.isAwake()) && !(boxB.isAwake())) {
//     return false;
//   }
//   if (boxA.getInverseMass() == 0 && boxB.getInverseMass() == 0) {
//     return false;
//   }
//   // get axes of Rectangle
//   sf::Vector2f positionA = boxA.RigidBody2D::getPosition();
//   sf::Vector2f halfSize = boxA.getHalfSize();
//   std::array<sf::Vector2f, 2> axesA =
//       getBaseCoordinateSystem(boxA.getOrientation());
//   sf::Vector2f positionB = boxB.RigidBody2D::getPosition();
//   sf::Vector2f halfSizeB = boxB.getHalfSize();
//   std::array<sf::Vector2f, 2> axesB =
//       getBaseCoordinateSystem(boxB.getOrientation());
//   sf::Vector2f distanceVector = positionA - positionB;
//   float minOverlap = 10'000;
//   sf::Vector2f smallestAxis;
//   bool overLapCondition;
//   for (int i = 0; i < 2; i++) {
//     sf::Vector2f axis = axesA[i];
//     overLapCondition =
//         absDot(axis, distanceVector) > (absDot(axesA[0] * halfSize.x, axis) +
//                                         absDot(axesA[1] * halfSize.y, axis) +
//                                         absDot(axesB[0] * halfSizeB.x, axis)
//                                         + absDot(axesB[1] * halfSizeB.y,
//                                         axis));

//     if (overLapCondition) {
//       return false;
//     } else {
//       float overlap = std::abs(absDot(axis, distanceVector) -
//                                (absDot(axesA[0] * halfSize.x, axis) +
//                                 absDot(axesA[1] * halfSize.y, axis) +
//                                 absDot(axesB[0] * halfSizeB.x, axis) +
//                                 absDot(axesB[1] * halfSizeB.y, axis)));
//       if (overlap < minOverlap) {
//         minOverlap = overlap;
//         smallestAxis = axis;
//       }
//     }
//     axis = axesB[i];
//     if (overLapCondition) {
//       return false;
//     } else {
//       float overlap = std::abs(absDot(axis, distanceVector) -
//                                (absDot(axesA[0] * halfSize.x, axis) +
//                                 absDot(axesA[1] * halfSize.y, axis) +
//                                 absDot(axesB[0] * halfSizeB.x, axis) +
//                                 absDot(axesB[1] * halfSizeB.y, axis)));
//       if (overlap < minOverlap) {
//         minOverlap = overlap;
//         smallestAxis = axis;
//       }
//     }
//   }
//   Contact contact(boxA, boxB);
//   sf::Vector2f normal = normalise(smallestAxis);
//   contact.setPenetrationDepth(minOverlap);
//   sf::Vector2f closestPoint;
//   std::array<sf::Vector2f, 4> verticesB = boxB.getVertices();
//   std::array<sf::Vector2f, 4> verticesA = boxA.getVertices();
//   bool isTouch = false;
//   bool isTouch2 = false;

//   ContactManifold contactManifold;
//   for (auto vertex : verticesB) {
//     if (boxA.isPointIn(vertex)) {
//       contact.setContactPoint(vertex);
//       isTouch = true;
//       if (boxB.isPointIn(normal * (1.001f) + vertex)) {
//         normal = -normal;
//       }
//       contact.setContactNormal(normal);
//       contact.setRelativeContactPosition(
//           {vertex - positionA - normal * minOverlap, vertex - positionB});
//       contactManifold.push_back(contact);
//     }
//   }

//   Contact contact2(boxA, boxB);
//   for (auto vertex : verticesA) {
//     if (boxB.isPointIn(vertex)) {
//       isTouch2 = true;
//       contact2.setContactPoint(vertex);
//       contact2.setPenetrationDepth(minOverlap);
//       if (dot(distanceVector, normal) < 0) {
//         normal = -normal;
//       }
//       if (boxB.isPointIn(vertex + normal * minOverlap)) {
//         contact2.setPenetrationDepth(minOverlap);

//       } else {

//         contact2.setPenetrationDepth(minOverlap);
//       }
//       contact2.setRelativeContactPosition(
//           {vertex - positionA, vertex - positionB + normal * minOverlap});
//       contact2.setContactNormal(normal);
//       contactManifold.push_back(contact2);
//     }
//   }
//   collisionData.push_back(contactManifold);
//   return isTouch || isTouch2;
// }

// bool Collider::genericCollision(RigidBody2D &bodyA, RigidBody2D &bodyB,
//                                 CollisionData &collisionData) {
//   if (Circle *circleAp = dynamic_cast<Circle *>(&bodyA)) {
//     if (Circle *circleBp = dynamic_cast<Circle *>(&bodyB)) {
//       return sphereAndSphere(*circleAp, *circleBp, collisionData);
//     }
//     Box *boxBp = dynamic_cast<Box *>(&bodyB);
//     return sphereAndRectangle(*circleAp, *boxBp, collisionData);
//   }
//   return rectangleAndRectangle(*dynamic_cast<Box *>(&bodyA),
//                                *dynamic_cast<Box *>(&bodyB), collisionData);
// }

bool Collider::GJKintersectionPP(Box &shapeA, Box &shapeB,
                                 std::vector<sf::Vector2f> &simplex) {
  // Step 1. Gjk(Gilbert-Johnson-Keerthi) algorithm;
  // Step 1a
  sf::Vector2f direction =
      shapeA.RigidBody2D::getPosition() - shapeB.RigidBody2D::getPosition();
  sf::Vector2f pointOnMinkowskiDiffAmB =
      shapeA.getSupport(direction) - shapeB.getSupport(-direction);
  simplex.push_back(pointOnMinkowskiDiffAmB);
  // std::vector<sf::Vector2f> simplex{pointOnMinkowskiDiffAmB};
  // Step 1b..z
  direction = -pointOnMinkowskiDiffAmB;
  while (true) {
    pointOnMinkowskiDiffAmB =
        shapeA.getSupport(direction) - shapeB.getSupport(-direction);
    if (dot(pointOnMinkowskiDiffAmB, direction) < 0) {
      return false;
    }
    simplex.push_back(pointOnMinkowskiDiffAmB);
    bool containsOrigin = nearestSimplex(simplex, direction, shapeA, shapeB);
    if (containsOrigin) {
      containsOrigin = true;
      break;
    }
  }
}

float Collider::findContactNormalPenetration(std::vector<sf::Vector2f> &simplex,
                                             Box &shapeA, Box &shapeB,
                                             sf::Vector2f &normal_,
                                             sf::RenderWindow *window) {
  //  EPA (Expanding Polytope Algorithm);
  // std::cout << "simplex size " << simplex.size() << std::endl;
  if (simplex.size() > 3) {
    simplex.pop_back();
  }
  //  here simplex contains origin, and have 3 edges.
  float minDistance = std::numeric_limits<float>::max();
  sf::Vector2f minNormal{0.F, 0.F};
  int minIndex = 0;
  while (minDistance == std::numeric_limits<float>::max()) {

    for (int i = 0; i < simplex.size(); i++) {
      int j = (i + 1) % simplex.size();
      sf::Vector2f sidei = simplex[j] - simplex[i];
      sf::Vector2f normal = normalise(-perpendicular(sidei));
      // std::cout << "normal "
      //           << "it " << i << " " << normal.x << " " << normal.y << "
      //           size"
      //           << simplex.size() << std::endl;
      float distance = dot(normal, simplex[i]);

      if (distance < 0) {
        distance *= -1.F;
        normal *= -1.F;
      }

      if (minDistance > distance) {
        minDistance = distance;
        minNormal = normal;
        minIndex = j;
      }
    }
    sf::Vector2f newVertex =
        shapeA.getSupport(minNormal) - shapeB.getSupport(-minNormal);
    float supportDistance = dot(minNormal, newVertex);
    if (std::abs(supportDistance - minDistance) > 0.001F) {
      minDistance = std::numeric_limits<float>::max();
      simplex.insert(simplex.begin() + minIndex, newVertex);
    }
  }
  // minNormal = (minDistance + 0.001F) * minNormal;
  normal_ = -minNormal;
  // for (int i = 0; i < simplex.size(); i++) {
  //   auto line = {simplex[i], simplex[(i + 1) % simplex.size()]};
  //   plotLine(line, *window, sf::Color::Red);
  // }
  // for (auto point : shapeA.getVertices()) {
  //   for (auto pointB : shapeB.getVertices()) {
  //     showPoints(*window, {point - pointB});
  //   }
  // }

  return minDistance;
}
bool Collider::polygonPolygon(Box &shapeA, Box &shapeB,
                              CollisionData &collisionData,
                              sf::RenderWindow *window) {}
bool Collider::findContactManifold(Box &shapeA, Box &shapeB,
                                   CollisionData &collisionData,
                                   sf::RenderWindow *window) {
  // SH alghortihm for cliping (Sutherland-Hodgman);
  std::vector<sf::Vector2f> simplex;
  if (!GJKintersectionPP(shapeA, shapeB, simplex)) {
    return false;
  }
  // Step0. collision normal and penetration depth
  // Normal is always assumed to from B->A
  sf::Vector2f normal;
  // std::cout << "simplex size " << simplex.size() << std::endl;
  // int ii = 0;
  // for (auto simp : simplex) {

  //   std::cout << ii << "simplex " << simp.x << " " << simp.y << std::endl;
  //   ii++;
  // }
  float penetrationDepth =
      findContactNormalPenetration(simplex, shapeA, shapeB, normal, window);
  // Step 1. Vertex furthest along the collision normal
  std::vector<sf::Vector2f> polygonA;
  std::vector<sf::Vector2f> polygonB;
  std::array<std::array<sf::Vector2f, 2>, 2> adjacentEdgesA;
  std::array<std::array<sf::Vector2f, 2>, 2> adjacentEdgesB;

  shapeA.getIncidentReferencePolygon(polygonA, -normal, adjacentEdgesA, window);
  shapeB.getIncidentReferencePolygon(polygonB, normal, adjacentEdgesB, window);

  // Step 2. Find incident and reference faces
  std::vector<sf::Vector2f> incidentFace;
  std::vector<sf::Vector2f> referenceFace;
  std::array<std::array<sf::Vector2f, 2>, 2> adjacentEdges;
  bool isReferenceA = false;
  if (std::abs(dot(perpendicular(polygonA[0] - polygonA[1]), normal)) >
      std::abs(dot(perpendicular(polygonB[0] - polygonB[1]), normal))) {
    referenceFace = polygonA;
    incidentFace = polygonB;
    adjacentEdges = adjacentEdgesA;
    isReferenceA = true;
  } else {

    referenceFace = polygonB;
    incidentFace = polygonA;
    adjacentEdges = adjacentEdgesB;
  }
  // Step 3. Cliping w.r.t. sides
  clip(incidentFace, adjacentEdges[0], window);
  clip(incidentFace, adjacentEdges[1], window);
  clip(incidentFace, {referenceFace[0], referenceFace[1]}, window, false);

  showPoints(*window, incidentFace, sf::Color::Magenta);

  // Step 4. Update Contacts
  ContactManifold contactManifold;
  for (auto &point : incidentFace) {
    Contact contact(shapeA, shapeB);
    contact.setPenetrationDepth(penetrationDepth);
    contact.setContactNormal(normal);
    if (isReferenceA) {
      contact.setContactPosition({point - normal * penetrationDepth, point});
    } else {
      contact.setContactPosition({point, point + normal * penetrationDepth});
    }
    contactManifold.push_back(contact);
  }
  collisionData.push_back(contactManifold);
}

bool Collider::nearestSimplex(std::vector<sf::Vector2f> &simplex,
                              sf::Vector2f &direction, const Box &shapeA,
                              const Box &shapeB) {
  sf::Vector2f sideCB = simplex[1] - simplex[0];
  sf::Vector2f sideC0 = -simplex[0];
  direction = tripleproduct(sideCB, sideC0, sideCB);
  simplex.push_back(shapeA.getSupport(direction) -
                    shapeB.getSupport(-direction));
  bool containsOrigin = false;
  sf::Vector2f sideAB = simplex[1] - simplex[2];
  sf::Vector2f sideAC = simplex[0] - simplex[2];
  sf::Vector2f perpAB = tripleproduct(sideAC, sideAB, sideAB);
  sf::Vector2f perpAC = tripleproduct(sideAB, sideAC, sideAC);
  if (dot(perpAB, -simplex[2]) > 0) {
    simplex.erase(simplex.begin());
    direction = perpAB;
  } else if (dot(perpAC, -simplex[2]) > 0) {
    simplex.erase(simplex.begin() + 1);
    direction = perpAC;
  } else {
    return true;
  }
  return containsOrigin;
}
void Collider::clip(std::vector<sf::Vector2f> &polygonToClip,
                    const std::array<sf::Vector2f, 2> &edge,
                    sf::RenderWindow *window, bool createNewPoint) {
  sf::Vector2f edgeVector = edge[1] - edge[0];
  sf::Vector2f edgeNormal = -perpendicular(normalise(edgeVector));
  std::vector<sf::Vector2f> tempVec;
  std::array<sf::Vector2f, 2> nn = {edge[0], edgeNormal};
  // plotLine(nn, *window);
  // if (createNewPoint) {

  //   plotLine(edge, *window);
  // } else {

  //   plotLine(edge, *window, sf::Color::Cyan);
  // }

  for (auto vertex : polygonToClip) {
    if (dot((vertex - edge[0]), edgeNormal) >= 0) {
      tempVec.push_back(vertex);
      continue;
    } else if (createNewPoint) {
      vertex = computeIntersection(polygonToClip, edge);
      if (vertex != sf::Vector2f{std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max()}) {
        tempVec.push_back(vertex);
      }
    }
  }
  polygonToClip = tempVec;
}