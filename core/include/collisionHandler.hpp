#pragma once
#include "contact.hpp"
#include "rigidBody2D.hpp"
#include "shapes.hpp"
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Sleep.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Event.hpp>
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
                                std::vector<sf::Vector2f> &simplex);
  static float findContactNormalPenetration(std::vector<sf::Vector2f> &simplex,
                                            Box &shapeA, Box &shapeB,
                                            sf::Vector2f &normal_);
  static bool nearestSimplex(std::vector<sf::Vector2f> &simplex,
                             sf::Vector2f &direction, const Box &shapeA,
                             const Box &shapeB);
  static void clip(std::vector<sf::Vector2f> &polygonToClip,
                   const std::array<sf::Vector2f, 2> &edge,
                   bool createNewPoint = true);
  static bool collide(RigidBody2D &bodyA, RigidBody2D &bodyB,
                      std::vector<Contact> &contacts);
  static const bool warmStart;
  static const bool accumulateImpulse;
};