#include "physics/math/Vec3.h"
#include "physics/collision/AABB.h"
#include "physics/dynamics/RigidBody.h"
#include "physics/world/World.h"
#include "physics/collision/CollisionDetection.h"

void run_vec3_tests();
void runAABBTests();
void runRigidBodyTests();
void runWorldTests();
void runCollisionDetectionTests();


int main() {
  run_vec3_tests();
  runAABBTests();
  runRigidBodyTests();
  runWorldTests();
  runCollisionDetectionTests();
  return 0;
}
