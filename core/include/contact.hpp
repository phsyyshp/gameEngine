#pragma once
#include "linearalg.hpp"
struct Contact {
  Contact() = default;
  la::Vector position = {};
  la::Vector relativeContactPositionA;
  la::Vector relativeContactPositionB;
  la::Vector normal; // Always from B to A;
  float totalNormalImpulse = 0.F;
  float totalTangentImpulse = 0.F;
  float penetrationDepth = 0.F;
  float friction = 0.2F;
  float resitution = 0.5F;
  float bias = 0.F;
  float massNormal = 0.F;
  float massTangent = 0.F;
};