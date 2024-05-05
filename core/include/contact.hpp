#pragma once
#include <SFML/System/Vector2.hpp>
struct Contact {
  Contact() = default;
  sf::Vector2f position = {};
  sf::Vector2f relativeContactPositionA;
  sf::Vector2f relativeContactPositionB;
  sf::Vector2f normal; // Always from B to A;
  float totalNormalImpulse = 0.F;
  float totalTangentImpulse = 0.F;
  float penetrationDepth = 0.F;
  float friction = 0.2F;
  float resitution = 0.5F;
  float bias = 0.F;
  float massNormal = 0.F;
  float massTangent = 0.F;
};