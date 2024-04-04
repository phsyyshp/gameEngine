#include "collidible.hpp"
#include "shapes.hpp"
#include "types.hpp"
#include <cmath>

bool collosionDetector(const Collidible &, const Collidible &);
bool sphereAndSphere(const Circle &, const Circle &, CollisionData &);