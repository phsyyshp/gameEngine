#pragma once
#include <SFML/System/Vector2.hpp>
struct Contact {
  Contact() = default;
  sf::Vector2f position = {};
  sf::Vector2f normal; // Always from B to A;
  float totalImpulse = 0.F;
  float penetrationDepth = 0.F;
  float friction = 0.F;
  float resitution = 0.5F;
  bool persistent = false;
  float bias = 0.F;
};