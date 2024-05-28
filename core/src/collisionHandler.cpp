#include "collisionHandler.hpp"

const bool Collider::warmStart = true;
const bool Collider::accumulateImpulse = true;
const bool Collider::applySleepScheme = false;

bool Collider::sphereAndSphere(Circle &a, Circle &b,
                               std::vector<Contact> &contacts) {
  la::Vector positionA = a.RigidBody2D::getPosition();
  la::Vector positionB = b.RigidBody2D::getPosition();
  la::Vector midLine = positionA - positionB;
  float distance = std::sqrt(midLine.x * midLine.x + midLine.y * midLine.y);
  if (distance <= 0.0f || distance >= a.getRadius() + b.getRadius()) {
    return false;
  }
  la::Vector normal = midLine / distance;
  Contact contact;
  contact.normal = normal;
  contact.position = positionB + normal * b.getRadius();
  contact.penetrationDepth = a.getRadius() + b.getRadius() - distance;
  contacts.push_back(contact);
  return true;
}
bool Collider::sphereAndRectangle(Circle &circle, Box &box,
                                  std::vector<Contact> &contacts) {
  la::Vector circleCenter = circle.RigidBody2D::getPosition();
  la::Vector boxCenter = box.RigidBody2D::getPosition();
  la::Vector relCenter = transformToCordinateSystem(
      circleCenter, boxCenter, box.RigidBody2D::getOrientation());

  // Early out check to see if we can exclude the contact
  if (std::abs(relCenter.x) - circle.getRadius() > box.getHalfSize().x ||
      std::abs(relCenter.y) - circle.getRadius() > box.getHalfSize().y) {
    return false;
  }
  // check if in contact
  la::Vector closestPoint(0, 0);
  closestPoint.x =
      std::clamp(relCenter.x, -box.getHalfSize().x, box.getHalfSize().x);
  closestPoint.y =
      std::clamp(relCenter.y, -box.getHalfSize().y, box.getHalfSize().y);
  la::Vector cp2center = closestPoint - relCenter;
  float distance = cp2center.x * cp2center.x + cp2center.y * cp2center.y;
  if (distance > circle.getRadius() * circle.getRadius()) {
    return false;
  }
  la::Vector closestPointWorld = inverseTransformToCordinateSystem(
      closestPoint, boxCenter, box.RigidBody2D::getOrientation());
  // update Contacts
  Contact contact;
  la::Vector contactNormal = normalise(-closestPointWorld + circleCenter);
  contact.normal = contactNormal;
  contact.penetrationDepth = circle.getRadius() - std::sqrt(distance);
  contact.position = closestPointWorld;
  contacts.push_back(contact);
  return true;
}

bool Collider::sphereAndRectangle(Box &box, Circle &circle,
                                  std::vector<Contact> &contacts) {
  // Transform the centre of the sphere into box coordinates
  la::Vector circleCenter = circle.RigidBody2D::getPosition();
  la::Vector boxCenter = box.RigidBody2D::getPosition();
  la::Vector relCenter = transformToCordinateSystem(
      circleCenter, boxCenter, box.RigidBody2D::getOrientation());

  // Early out check to see if we can exclude the contact
  if (std::abs(relCenter.x) - circle.getRadius() > box.getHalfSize().x ||
      std::abs(relCenter.y) - circle.getRadius() > box.getHalfSize().y) {
    return false;
  }
  la::Vector closestPoint(0, 0);
  // Clamp each coordinate to the box.
  closestPoint.x =
      std::clamp(relCenter.x, -box.getHalfSize().x, box.getHalfSize().x);
  closestPoint.y =
      std::clamp(relCenter.y, -box.getHalfSize().y, box.getHalfSize().y);
  // Check we're in contact
  la::Vector cp2center = closestPoint - relCenter;
  float distance = cp2center.x * cp2center.x + cp2center.y * cp2center.y;
  if (distance > circle.getRadius() * circle.getRadius()) {
    return false;
  }

  // Compile the contact
  la::Vector closestPointWorld = inverseTransformToCordinateSystem(
      closestPoint, boxCenter, box.RigidBody2D::getOrientation());
  // closestPointWorld = closestPoint + boxCenter;
  Contact contact;

  la::Vector contactNormal = normalise(-closestPointWorld + circleCenter);
  contact.normal = -contactNormal;
  contact.penetrationDepth = circle.getRadius() - std::sqrt(distance);
  contact.position = closestPointWorld;
  contacts.push_back(contact);
  return true;
}
bool Collider::collide(RigidBody2D &bodyA, RigidBody2D &bodyB,
                       std::vector<Contact> &contacts) {

  RigidBody2DType typeA = bodyA.type();
  RigidBody2DType typeB = bodyB.type();
  // std::cout << "a is" << typeA << "b is" << typeB << std::endl;

  if (typeA == RigidBody2DType::CIRCLE && typeB == RigidBody2DType::CIRCLE) {
    return sphereAndSphere(static_cast<Circle &>(bodyA),
                           static_cast<Circle &>(bodyB), contacts);
  }
  if (typeA == RigidBody2DType::CIRCLE && typeB == RigidBody2DType::BOX) {
    return sphereAndRectangle(static_cast<Circle &>(bodyA),
                              static_cast<Box &>(bodyB), contacts);
  }
  if (typeA == RigidBody2DType::BOX && typeB == RigidBody2DType::CIRCLE) {
    return sphereAndRectangle(static_cast<Box &>(bodyA),
                              static_cast<Circle &>(bodyB), contacts);
  }
  if (typeA == RigidBody2DType::BOX && typeB == RigidBody2DType::BOX) {

    return rectangleAndRectangle(static_cast<Box &>(bodyA),
                                 static_cast<Box &>(bodyB), contacts);
  }
}

bool Collider::GJKintersectionPP(Box &shapeA, Box &shapeB,
                                 std::vector<la::Vector> &simplex) {
  // Step 1. Gjk(Gilbert-Johnson-Keerthi) algorithm;
  // Step 1a
  la::Vector direction =
      shapeA.RigidBody2D::getPosition() - shapeB.RigidBody2D::getPosition();
  la::Vector pointOnMinkowskiDiffAmB =
      shapeA.getSupport(direction) - shapeB.getSupport(-direction);
  simplex.push_back(pointOnMinkowskiDiffAmB);
  // Step 1b..z
  direction = -pointOnMinkowskiDiffAmB;
  while (true) {
    // std::cout << simplex.size() << std::endl;
    pointOnMinkowskiDiffAmB =
        shapeA.getSupport(direction) - shapeB.getSupport(-direction);
    if (dot(pointOnMinkowskiDiffAmB, direction) < 0) {
      return false;
    }
    simplex.push_back(pointOnMinkowskiDiffAmB);
    bool containsOrigin = nearestSimplex(simplex, direction, shapeA, shapeB);
    if (containsOrigin) {
      containsOrigin = true;
      return true;
      break;
    }
  }
}

float Collider::findContactNormalPenetration(std::vector<la::Vector> &simplex,
                                             Box &shapeA, Box &shapeB,
                                             la::Vector &normal_) {
  //  EPA (Expanding Polytope Algorithm);
  // std::cout << simplex.size() << std::endl;
  if (simplex.size() > 3) {
    simplex.pop_back();
  }
  //  Here simplex contains origin, and have 3 edges.
  float minDistance = std::numeric_limits<float>::max();
  la::Vector minNormal{0.F, 0.F};
  int minIndex = 0;
  while (minDistance == std::numeric_limits<float>::max()) {

    for (int i = 0; i < simplex.size(); i++) {
      int j = (i + 1) % simplex.size();
      la::Vector sidei = simplex[j] - simplex[i];
      la::Vector normal = normalise(-perpendicular(sidei));
      float distance = dot(normal, simplex[i]);
      if (distance < 0.F) {
        distance *= -1.F;
        normal *= -1.F;
      }
      if (minDistance > distance) {
        minDistance = distance;
        minNormal = normal;
        minIndex = j;
      }
      if (std::abs(distance) <= 1e-3F) {
        return 0.001F;
      }
    }
    la::Vector newVertex =
        shapeA.getSupport(minNormal) - shapeB.getSupport(-minNormal);
    float supportDistance = dot(minNormal, newVertex);
    if (std::abs(supportDistance - minDistance) > 0.001F) {
      minDistance = std::numeric_limits<float>::max();
      simplex.insert(simplex.begin() + minIndex, newVertex);
    }
  }
  normal_ = -minNormal; // from b->a
  // Visualising the simplex and mDiff
  // if (Visual::isDebug) {
  //   for (int i = 0; i < simplex.size(); i++) {
  //     auto line = {simplex[i], simplex[(i + 1) % simplex.size()]};
  //     plotLine(line, Visual::window, sf::Color::Red);
  //   }
  //   for (auto point : shapeA.getVertices()) {
  //     for (auto pointB : shapeB.getVertices()) {
  //       showPoints(Visual::window, {point - pointB});
  //     }
  //   }
  // }
  return minDistance;
}

bool Collider::rectangleAndRectangle(Box &shapeA, Box &shapeB,
                                     std::vector<Contact> &contacts) {
  // SH alghortihm for cliping (Sutherland-Hodgman);
  std::vector<la::Vector> simplex;
  if (!GJKintersectionPP(shapeA, shapeB, simplex)) {
    return false;
  }
  // Step0. collision normal and penetration depth
  // Normal is always assumed to from B->A
  la::Vector normal;
  float penetrationDepth =
      findContactNormalPenetration(simplex, shapeA, shapeB, normal);
  // Step 1. Vertex furthest along the collision normal
  std::vector<la::Vector> polygonA;
  std::vector<la::Vector> polygonB;
  std::array<std::array<la::Vector, 2>, 2> adjacentEdgesA;
  std::array<std::array<la::Vector, 2>, 2> adjacentEdgesB;

  shapeA.getIncidentReferencePolygon(polygonA, -normal, adjacentEdgesA);
  shapeB.getIncidentReferencePolygon(polygonB, normal, adjacentEdgesB);

  // Step 2. Find incident and reference faces
  std::vector<la::Vector> incidentFace;
  std::vector<la::Vector> referenceFace;
  std::array<std::array<la::Vector, 2>, 2> adjacentEdges;
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
  clip(incidentFace, adjacentEdges[0]);
  clip(incidentFace, adjacentEdges[1]);
  clip(incidentFace, {referenceFace[0], referenceFace[1]}, false);

  // if (Visual::isDebug) {
  //   showPoints(Visual::window, incidentFace, sf::Color::Magenta);
  // }

  // Step 4. Update Contacts
  for (auto &point : incidentFace) {
    Contact contact;
    contact.penetrationDepth = penetrationDepth;
    contact.normal = normal;
    if (isReferenceA) {
      contact.position = point;
    } else {
      contact.position = point;
    }
    contacts.push_back(contact);
  }
  return true;
}

bool Collider::nearestSimplex(std::vector<la::Vector> &simplex,
                              la::Vector &direction, const Box &shapeA,
                              const Box &shapeB) {
  la::Vector sideCB = simplex[1] - simplex[0];
  la::Vector sideC0 = -simplex[0];
  direction = tripleproduct(sideCB, sideC0, sideCB);
  simplex.push_back(shapeA.getSupport(direction) -
                    shapeB.getSupport(-direction));
  bool containsOrigin = false;
  la::Vector sideAB = simplex[1] - simplex[2];
  la::Vector sideAC = simplex[0] - simplex[2];
  la::Vector perpAB = tripleproduct(sideAC, sideAB, sideAB);
  la::Vector perpAC = tripleproduct(sideAB, sideAC, sideAC);
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
void Collider::clip(std::vector<la::Vector> &polygonToClip,
                    const std::array<la::Vector, 2> &edge,
                    bool createNewPoint) {
  la::Vector edgeVector = edge[1] - edge[0];
  la::Vector edgeNormal = -perpendicular(normalise(edgeVector));
  std::vector<la::Vector> tempVec;
  std::array<la::Vector, 2> nn = {edge[0], edgeNormal};
  // debug visualization
  // if (Visual::isDebug) {
  //   plotLine(nn, Visual::window);
  //   if (createNewPoint) {

  //     plotLine(edge, Visual::window);
  //   } else {

  //     plotLine(edge, Visual::window, sf::Color::Cyan);
  //   }
  // }

  for (auto vertex : polygonToClip) {
    if (dot((vertex - edge[0]), edgeNormal) >= 0) {
      tempVec.push_back(vertex);
      continue;
    } else if (createNewPoint) {
      vertex = computeIntersection(polygonToClip, edge);
      if (vertex != la::Vector{std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::max()}) {
        tempVec.push_back(vertex);
      }
    }
  }
  polygonToClip = tempVec;
}