#pragma once 
#include "physics/dynamics/RigidBody.h"
#include "physics/math/Vec3.h"

namespace physics::collision {

struct CollisionInfo {
  bool hasCollision;
  physics::math::Vec3 contactPoint;
  physics::math::Vec3 normal;
  float penetrationDepth;

  CollisionInfo();
  CollisionInfo(const physics::math::Vec3& point, const physics::math::Vec3& normal, float depth);
};

class CollisionDetection {
public:
  static CollisionInfo checkGroundCollision(const physics::dynamics::RigidBody& body, float groundY = 0.0f);
  static void resolveGroundCollision(physics::dynamics::RigidBody& body, const CollisionInfo& collision);
  static bool checkAABBCollision(const physics::dynamics::RigidBody& bodyA, const physics::dynamics::RigidBody& bodyB);
  static CollisionInfo getAABBCollisionInfo(const physics::dynamics::RigidBody& bodyA, const physics::dynamics::RigidBody& bodyB);
  static void resolveAABBCollision(physics::dynamics::RigidBody& bodyA, physics::dynamics::RigidBody& bodyB, const CollisionInfo& collision);

private:
  static physics::math::Vec3 calculateSeparationVector(const AABB& aabbA, const AABB& aabbB);
};

}
