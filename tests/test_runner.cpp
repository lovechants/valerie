#include "physics/math/Vec3.h"
#include "physics/collision/AABB.h"
#include "physics/dynamics/RigidBody.h"
#include "physics/world/World.h"


void run_vec3_tests();
void runAABBTests();
void runRigidBodyTests();
void runWorldTests();

int main() {
  run_vec3_tests();
  runAABBTests();
  runRigidBodyTests();
  runWorldTests();
  return 0;
}
