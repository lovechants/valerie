#include "physics/collision/CollisionDetection.h"
#include <algorithm>
#include <cmath>
#include <iostream>
namespace physics::collision {

using Vec3 = physics::math::Vec3;
using RigidBody = physics::dynamics::RigidBody;

CollisionInfo::CollisionInfo()
  : hasCollision(false), contactPoint(0, 0, 0), normal(0, 1, 0), penetrationDepth(0) {}

CollisionInfo::CollisionInfo(const Vec3& point, const Vec3& normal, float depth) 
  : hasCollision(true), contactPoint(point), normal(normal), penetrationDepth(depth) {}

CollisionInfo CollisionDetection::checkGroundCollision(const RigidBody& body, float groundY) {
  AABB aabb = body.getAABB();

  if (aabb.min.y <= groundY) {
    float penetration = groundY - aabb.min.y;
    Vec3 contactPoint(body.position.x, groundY, body.position.z);
    Vec3 normal(0, 1, 0);

    return CollisionInfo(contactPoint, normal, penetration);
  }

  return CollisionInfo();
}

void CollisionDetection::resolveGroundCollision(RigidBody& body, const CollisionInfo& collision) {
  if (!collision.hasCollision) return;

  body.position.y += collision.penetrationDepth; 
  body.onGround = true;

  if (body.velocity.y < 0) {
    body.velocity.y = -body.velocity.y * body.restitution;
    if (std::abs(body.velocity.y) < 1.0f) {
      body.velocity.y = 0.0f;
    }
  }
}


bool CollisionDetection::checkAABBCollision(const RigidBody& bodyA, const RigidBody& bodyB) {
    AABB aabbA = bodyA.getAABB();
    AABB aabbB = bodyB.getAABB();
    
    return aabbA.intersects(aabbB);
}

CollisionInfo CollisionDetection::getAABBCollisionInfo(const RigidBody& bodyA, const RigidBody& bodyB) {
    AABB aabbA = bodyA.getAABB();
    AABB aabbB = bodyB.getAABB();
    
    if (!aabbA.intersects(aabbB)) {
        return CollisionInfo();
    }
    
    Vec3 separation = calculateSeparationVector(aabbA, aabbB);
    float penetrationDepth = separation.length();
    Vec3 normal = separation.normalized();
    
    Vec3 contactPoint = (bodyA.position + bodyB.position) * 0.5f;
    
    return CollisionInfo(contactPoint, normal, penetrationDepth);
}

void CollisionDetection::resolveAABBCollision(RigidBody& bodyA, RigidBody& bodyB, const CollisionInfo& collision) {
    if (!collision.hasCollision) return;
    
    if (bodyA.isStatic && bodyB.isStatic) return;
    
    Vec3 separation = collision.normal * collision.penetrationDepth;
    
    if (bodyA.isStatic) {
        bodyB.position = bodyB.position + separation;
    } else if (bodyB.isStatic) {
        bodyA.position = bodyA.position - separation;
    } else {
        float totalInverseMass = bodyA.inverseMass + bodyB.inverseMass;
        float ratioA = bodyA.inverseMass / totalInverseMass;
        float ratioB = bodyB.inverseMass / totalInverseMass;
        bodyA.position = bodyA.position + separation * ratioA;
        bodyB.position = bodyB.position - separation * ratioB;
        Vec3 separation = collision.normal * collision.penetrationDepth;
        std::cout << "DEBUG: separation vector=(" << separation.x << ", " << separation.y << ", " << separation.z << ")\n";
        std::cout << "DEBUG: ratioA=" << ratioA << " ratioB=" << ratioB << "\n";     }
    
    Vec3 relativeVelocity = bodyA.velocity - bodyB.velocity;
    float velocityAlongNormal = relativeVelocity.dot(collision.normal);
    
    if (velocityAlongNormal > 0) return;
    
    float restitution = std::min(bodyA.restitution, bodyB.restitution);
    float impulseScalar = -(1 + restitution) * velocityAlongNormal;
    impulseScalar /= (bodyA.inverseMass + bodyB.inverseMass);
    
    Vec3 impulse = collision.normal * impulseScalar;
    
    if (!bodyA.isStatic) {
        bodyA.velocity = bodyA.velocity + impulse * bodyA.inverseMass;
    }
    if (!bodyB.isStatic) {
        bodyB.velocity = bodyB.velocity - impulse * bodyB.inverseMass;
    }
}

Vec3 CollisionDetection::calculateSeparationVector(const AABB& aabbA, const AABB& aabbB) {
    float xOverlap = std::min(aabbA.max.x, aabbB.max.x) - std::max(aabbA.min.x, aabbB.min.x);
    float yOverlap = std::min(aabbA.max.y, aabbB.max.y) - std::max(aabbA.min.y, aabbB.min.y);
    float zOverlap = std::min(aabbA.max.z, aabbB.max.z) - std::max(aabbA.min.z, aabbB.min.z);
    
    if (xOverlap <= yOverlap && xOverlap <= zOverlap) {
        float direction = (aabbA.getCenter().x < aabbB.getCenter().x) ? -1.0f : 1.0f;
        return Vec3(xOverlap * direction, 0, 0);
    } else if (yOverlap <= zOverlap) {
        float direction = (aabbA.getCenter().y < aabbB.getCenter().y) ? -1.0f : 1.0f;
        return Vec3(0, yOverlap * direction, 0);
    } else {
        float direction = (aabbA.getCenter().z < aabbB.getCenter().z) ? -1.0f : 1.0f;
        return Vec3(0, 0, zOverlap * direction);
    }
}
}
