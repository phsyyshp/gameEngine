#pragma once
#include "manifold.hpp"
class ContactResolver {
public:
  void prestep(std::map<ManifoldKey, Manifold> manifolds, float deltaTime);
  void sequentialImpulse(std::map<ManifoldKey, Manifold> manifolds,
                         float deltaTime);
};