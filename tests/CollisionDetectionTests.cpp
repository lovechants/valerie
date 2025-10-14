#include "physics/collision/CollisionDetection.h"
#include <iostream>
#include <cassert>
#include <iomanip>
#include <cmath>

using namespace physics::collision;
using namespace physics::dynamics;
using namespace physics::math;

void testGroundCollisionDetection() {
    std::cout << std::fixed << std::setprecision(3);
    
    RigidBody aboveGround(Vec3(0, 5, 0), Vec3(2, 2, 2), 1.0f);
    CollisionInfo noCollision = CollisionDetection::checkGroundCollision(aboveGround);
    std::cout << "Body above ground (y=5, size=2): collision=" << (noCollision.hasCollision ? "true" : "false") << "\n";
    assert(!noCollision.hasCollision);
    
    RigidBody onGround(Vec3(0, 1, 0), Vec3(2, 2, 2), 1.0f);
    CollisionInfo groundCollision = CollisionDetection::checkGroundCollision(onGround);
    std::cout << "Body intersecting ground (y=1, size=2, min.y=0): collision=" << (groundCollision.hasCollision ? "true" : "false") << "\n";
    std::cout << "Contact point: (" << groundCollision.contactPoint.x << ", " << groundCollision.contactPoint.y << ", " << groundCollision.contactPoint.z << ")\n";
    std::cout << "Normal: (" << groundCollision.normal.x << ", " << groundCollision.normal.y << ", " << groundCollision.normal.z << ") penetration=" << groundCollision.penetrationDepth << "\n";
    assert(groundCollision.hasCollision);
    assert(groundCollision.normal.y == 1.0f && groundCollision.normal.x == 0.0f && groundCollision.normal.z == 0.0f);
    assert(groundCollision.penetrationDepth == 0.0f);
    
    RigidBody belowGround(Vec3(0, -2, 0), Vec3(2, 2, 2), 1.0f);
    CollisionInfo deepCollision = CollisionDetection::checkGroundCollision(belowGround);
    std::cout << "Body below ground (y=-2, size=2, min.y=-3): collision=" << (deepCollision.hasCollision ? "true" : "false") << " penetration=" << deepCollision.penetrationDepth << "\n";
    assert(deepCollision.hasCollision);
    assert(deepCollision.penetrationDepth == 3.0f);
}

void testGroundCollisionResponse() {
    RigidBody fallingBody(Vec3(0, -1, 0), Vec3(2, 2, 2), 1.0f);
    fallingBody.velocity = Vec3(0, -10, 0);
    fallingBody.restitution = 0.8f;
    
    CollisionInfo collision = CollisionDetection::checkGroundCollision(fallingBody);
    std::cout << "Falling body before resolution: pos(" << fallingBody.position.x << ", " << fallingBody.position.y << ", " << fallingBody.position.z << ") vel(" << fallingBody.velocity.x << ", " << fallingBody.velocity.y << ", " << fallingBody.velocity.z << ")\n";
    std::cout << "OnGround: " << (fallingBody.onGround ? "true" : "false") << "\n";
    
    CollisionDetection::resolveGroundCollision(fallingBody, collision);
    
    std::cout << "After resolution: pos(" << fallingBody.position.x << ", " << fallingBody.position.y << ", " << fallingBody.position.z << ") vel(" << fallingBody.velocity.x << ", " << fallingBody.velocity.y << ", " << fallingBody.velocity.z << ")\n";
    std::cout << "OnGround: " << (fallingBody.onGround ? "true" : "false") << "\n";
    
    assert(fallingBody.position.y == 1.0f);
    assert(std::abs(fallingBody.velocity.y - 8.0f) < 0.001f);
    assert(fallingBody.onGround);
    
    RigidBody slowBody(Vec3(0, -0.5f, 0), Vec3(2, 2, 2), 1.0f);
    slowBody.velocity = Vec3(0, -0.05f, 0);
    slowBody.restitution = 0.5f;
    
    CollisionInfo slowCollision = CollisionDetection::checkGroundCollision(slowBody);
    CollisionDetection::resolveGroundCollision(slowBody, slowCollision);
    
    std::cout << "Slow falling body after resolution: vel.y=" << slowBody.velocity.y << "\n";
    assert(slowBody.velocity.y == 0.0f);
}

void testAABBCollisionDetection() {
    RigidBody bodyA(Vec3(0, 0, 0), Vec3(2, 2, 2), 1.0f);
    RigidBody bodyB(Vec3(1, 0, 0), Vec3(2, 2, 2), 1.0f);
    RigidBody bodyC(Vec3(5, 0, 0), Vec3(2, 2, 2), 1.0f);
    
    bool collisionAB = CollisionDetection::checkAABBCollision(bodyA, bodyB);
    bool collisionAC = CollisionDetection::checkAABBCollision(bodyA, bodyC);
    
    std::cout << "Bodies A(0,0,0) and B(1,0,0) with size(2,2,2): collision=" << (collisionAB ? "true" : "false") << "\n";
    std::cout << "Bodies A(0,0,0) and C(5,0,0) with size(2,2,2): collision=" << (collisionAC ? "true" : "false") << "\n";
    
    assert(collisionAB == true);
    assert(collisionAC == false);
    
    CollisionInfo infoAB = CollisionDetection::getAABBCollisionInfo(bodyA, bodyB);
    std::cout << "Collision A-B: contact(" << infoAB.contactPoint.x << ", " << infoAB.contactPoint.y << ", " << infoAB.contactPoint.z << ") normal(" << infoAB.normal.x << ", " << infoAB.normal.y << ", " << infoAB.normal.z << ") depth=" << infoAB.penetrationDepth << "\n";
    
    assert(infoAB.hasCollision);
    assert(std::abs(infoAB.normal.x - (-1.0f)) < 0.001f || std::abs(infoAB.normal.x - 1.0f) < 0.001f);
    assert(infoAB.penetrationDepth == 1.0f);
}

void testAABBCollisionResponse() {
    RigidBody bodyA(Vec3(-0.5f, 0, 0), Vec3(2, 2, 2), 2.0f);
    RigidBody bodyB(Vec3(0.5f, 0, 0), Vec3(2, 2, 2), 1.0f);
    
    bodyA.velocity = Vec3(2, 0, 0);
    bodyB.velocity = Vec3(-1, 0, 0);
    bodyA.restitution = 0.8f;
    bodyB.restitution = 0.6f;
    
    std::cout << "Before collision resolution:\n";
    std::cout << "BodyA: pos(" << bodyA.position.x << ", " << bodyA.position.y << ", " << bodyA.position.z << ") vel(" << bodyA.velocity.x << ", " << bodyA.velocity.y << ", " << bodyA.velocity.z << ") mass=" << bodyA.mass << "\n";
    std::cout << "BodyB: pos(" << bodyB.position.x << ", " << bodyB.position.y << ", " << bodyB.position.z << ") vel(" << bodyB.velocity.x << ", " << bodyB.velocity.y << ", " << bodyB.velocity.z << ") mass=" << bodyB.mass << "\n";
    
    CollisionInfo collision = CollisionDetection::getAABBCollisionInfo(bodyA, bodyB);
    CollisionDetection::resolveAABBCollision(bodyA, bodyB, collision);
    
    std::cout << "After collision resolution:\n";
    std::cout << "BodyA: pos(" << bodyA.position.x << ", " << bodyA.position.y << ", " << bodyA.position.z << ") vel(" << bodyA.velocity.x << ", " << bodyA.velocity.y << ", " << bodyA.velocity.z << ")\n";
    std::cout << "BodyB: pos(" << bodyB.position.x << ", " << bodyB.position.y << ", " << bodyB.position.z << ") vel(" << bodyB.velocity.x << ", " << bodyB.velocity.y << ", " << bodyB.velocity.z << ")\n";
    
    assert(bodyA.position.x < bodyB.position.x);
    assert(bodyA.velocity.x < 2.0f && bodyB.velocity.x > -1.0f);
}

void testStaticBodyCollisions() {
    RigidBody movingBody(Vec3(-1, 0, 0), Vec3(2, 2, 2), 1.0f);
    RigidBody staticBody(Vec3(1, 0, 0), Vec3(2, 2, 2), 0.0f);
    
    movingBody.velocity = Vec3(5, 0, 0);
    staticBody.makeStatic();
    
    std::cout << "Moving body vs static body:\n";
    std::cout << "Before: movingBody pos(" << movingBody.position.x << ", " << movingBody.position.y << ", " << movingBody.position.z << ") vel(" << movingBody.velocity.x << ", " << movingBody.velocity.y << ", " << movingBody.velocity.z << ")\n";
    std::cout << "Before: staticBody pos(" << staticBody.position.x << ", " << staticBody.position.y << ", " << staticBody.position.z << ") vel(" << staticBody.velocity.x << ", " << staticBody.velocity.y << ", " << staticBody.velocity.z << ")\n";
    
    CollisionInfo collision = CollisionDetection::getAABBCollisionInfo(movingBody, staticBody);
    CollisionDetection::resolveAABBCollision(movingBody, staticBody, collision);
    
    std::cout << "After: movingBody pos(" << movingBody.position.x << ", " << movingBody.position.y << ", " << movingBody.position.z << ") vel(" << movingBody.velocity.x << ", " << movingBody.velocity.y << ", " << movingBody.velocity.z << ")\n";
    std::cout << "After: staticBody pos(" << staticBody.position.x << ", " << staticBody.position.y << ", " << staticBody.position.z << ") vel(" << staticBody.velocity.x << ", " << staticBody.velocity.y << ", " << staticBody.velocity.z << ")\n";
    
    assert(staticBody.position.x == 1.0f && staticBody.position.y == 0.0f && staticBody.position.z == 0.0f);
    assert(staticBody.velocity.x == 0.0f && staticBody.velocity.y == 0.0f && staticBody.velocity.z == 0.0f);
    assert(movingBody.position.x < 0.0f);
}

void testMultiAxisCollisions() {
    RigidBody bodyA(Vec3(0, 0, 0), Vec3(2, 4, 2), 1.0f);
    RigidBody bodyB(Vec3(0, 1.5f, 0), Vec3(2, 1, 2), 1.0f);
    
    std::cout << "Multi-axis collision test:\n";
    std::cout << "BodyA: pos(" << bodyA.position.x << ", " << bodyA.position.y << ", " << bodyA.position.z << ") size(" << bodyA.size.x << ", " << bodyA.size.y << ", " << bodyA.size.z << ")\n";
    std::cout << "BodyB: pos(" << bodyB.position.x << ", " << bodyB.position.y << ", " << bodyB.position.z << ") size(" << bodyB.size.x << ", " << bodyB.size.y << ", " << bodyB.size.z << ")\n";
    
    CollisionInfo collision = CollisionDetection::getAABBCollisionInfo(bodyA, bodyB);
    std::cout << "Collision normal: (" << collision.normal.x << ", " << collision.normal.y << ", " << collision.normal.z << ") depth=" << collision.penetrationDepth << "\n";
    
    assert(collision.hasCollision);
    assert(std::abs(collision.normal.y) == 1.0f);
    assert(collision.penetrationDepth == 1.0f);
}

void testNoCollisionResponse() {
    RigidBody bodyA(Vec3(0, 0, 0), Vec3(1, 1, 1), 1.0f);
    RigidBody bodyB(Vec3(5, 0, 0), Vec3(1, 1, 1), 1.0f);
    
    Vec3 originalPosA = bodyA.position;
    Vec3 originalPosB = bodyB.position;
    Vec3 originalVelA = bodyA.velocity;
    Vec3 originalVelB = bodyB.velocity;
    
    CollisionInfo collision = CollisionDetection::getAABBCollisionInfo(bodyA, bodyB);
    CollisionDetection::resolveAABBCollision(bodyA, bodyB, collision);
    
    std::cout << "No collision test: bodies should remain unchanged\n";
    std::cout << "BodyA position changed: " << (bodyA.position.x != originalPosA.x ? "true" : "false") << "\n";
    std::cout << "BodyB position changed: " << (bodyB.position.x != originalPosB.x ? "true" : "false") << "\n";
    
    assert(bodyA.position.x == originalPosA.x && bodyA.position.y == originalPosA.y && bodyA.position.z == originalPosA.z);
    assert(bodyB.position.x == originalPosB.x && bodyB.position.y == originalPosB.y && bodyB.position.z == originalPosB.z);
    assert(bodyA.velocity.x == originalVelA.x && bodyA.velocity.y == originalVelA.y && bodyA.velocity.z == originalVelA.z);
    assert(bodyB.velocity.x == originalVelB.x && bodyB.velocity.y == originalVelB.y && bodyB.velocity.z == originalVelB.z);
}

void runCollisionDetectionTests() {
    testGroundCollisionDetection();
    testGroundCollisionResponse();
    testAABBCollisionDetection();
    testAABBCollisionResponse();
    testStaticBodyCollisions();
    testMultiAxisCollisions();
    testNoCollisionResponse();
}
