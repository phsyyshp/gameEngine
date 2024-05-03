#pragma once
#include "manifold.hpp"
#include <SFML/System/Vector2.hpp>

class ContactResolver {
public:
  void prestep(std::map<ManifoldKey, Manifold> manifolds, float deltaTime);
  void sequentialImpulse(std::map<ManifoldKey, Manifold> manifolds,
                         float deltaTime);
};