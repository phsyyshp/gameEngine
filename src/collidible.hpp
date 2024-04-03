#include "rigidBody2D.hpp"
class Collidible : public RigidBody2D {
public:
  Collidible(float x, float y) : RigidBody2D(x, y) {}
};