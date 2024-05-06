#pragma once
#include "contact.hpp"
#include "rigidBody2D.hpp"
#include "shapes.hpp"
#include <vector>

class Collider {
public:
  static bool sphereAndSphere(Circle &, Circle &, std::vector<Contact> &);
  static bool sphereAndRectangle(Circle &, Box &, std::vector<Contact> &);
  static bool sphereAndRectangle(Box &box, Circle &circle,
                                 std::vector<Contact> &contacts);
  static bool rectangleAndRectangle(Box &shapeA, Box &shapeB,
                                    std::vector<Contact> &contacts);
  static bool GJKintersectionPP(Box &shapeA, Box &shapeB,
                                std::vector<la::Vector> &simplex);
  static float findContactNormalPenetration(std::vector<la::Vector> &simplex,
                                            Box &shapeA, Box &shapeB,
                                            la::Vector &normal_);
  static bool nearestSimplex(std::vector<la::Vector> &simplex,
                             la::Vector &direction, const Box &shapeA,
                             const Box &shapeB);
  static void clip(std::vector<la::Vector> &polygonToClip,
                   const std::array<la::Vector, 2> &edge,
                   bool createNewPoint = true);
  static bool collide(RigidBody2D &bodyA, RigidBody2D &bodyB,
                      std::vector<Contact> &contacts);
  static const bool warmStart;
  static const bool accumulateImpulse;
  static const bool applySleepScheme;
};