#include "physics/dynamics/RigidBody.h"
#include <iostream>
#include <cassert>
#include <iomanip>
#include <cmath>

using namespace physics::dynamics;
using namespace physics::math;
using namespace physics::collision;

void testRigidBodyConstruction() {
    std::cout << std::fixed << std::setprecision(3);
    
    RigidBody body1;
    std::cout << "Default RigidBody: pos(" << body1.position.x << ", " << body1.position.y << ", " << body1.position.z << ") mass=" << body1.mass << " inverseMass=" << body1.inverseMass << "\n";
    assert(body1.mass == 1.0f && body1.inverseMass == 1.0f);
    assert(!body1.isStatic);
    
    RigidBody body2(Vec3(5, 10, 15), Vec3(2, 4, 6), 2.5f);
    std::cout << "Custom RigidBody: pos(" << body2.position.x << ", " << body2.position.y << ", " << body2.position.z << ") size(" << body2.size.x << ", " << body2.size.y << ", " << body2.size.z << ") mass=" << body2.mass << "\n";
    assert(body2.position.x == 5 && body2.position.y == 10 && body2.position.z == 15);
    assert(body2.mass == 2.5f && std::abs(body2.inverseMass - 0.4f) < 0.001f);
    
    RigidBody body3(Vec3(0, 0, 0), Vec3(1, 1, 1), 0.0f);
    std::cout << "Static RigidBody: mass=" << body3.mass << " inverseMass=" << body3.inverseMass << " isStatic=" << (body3.isStatic ? "true" : "false") << "\n";
    assert(body3.isStatic && body3.inverseMass == 0.0f);
}

void testMassManagement() {
    RigidBody body;
    
    body.setMass(4.0f);
    std::cout << "After setMass(4.0): mass=" << body.mass << " inverseMass=" << body.inverseMass << " isStatic=" << (body.isStatic ? "true" : "false") << "\n";
    assert(body.mass == 4.0f && std::abs(body.inverseMass - 0.25f) < 0.001f && !body.isStatic);
    
    body.makeStatic();
    std::cout << "After makeStatic(): mass=" << body.mass << " inverseMass=" << body.inverseMass << " isStatic=" << (body.isStatic ? "true" : "false") << "\n";
    std::cout << "Static body velocity: (" << body.velocity.x << ", " << body.velocity.y << ", " << body.velocity.z << ")\n";
    assert(body.isStatic && body.inverseMass == 0.0f);
    assert(body.velocity.x == 0 && body.velocity.y == 0 && body.velocity.z == 0);
    
    body.makeDynamic(2.0f);
    std::cout << "After makeDynamic(2.0): mass=" << body.mass << " inverseMass=" << body.inverseMass << " isStatic=" << (body.isStatic ? "true" : "false") << "\n";
    assert(body.mass == 2.0f && std::abs(body.inverseMass - 0.5f) < 0.001f && !body.isStatic);
}

void testForceApplication() {
    RigidBody body(Vec3(0, 0, 0), Vec3(1, 1, 1), 2.0f);
    
    Vec3 force1(10, 0, 0);
    body.applyForce(force1);
    std::cout << "Applied force(10,0,0) to mass=2.0 body: acceleration=(" << body.acceleration.x << ", " << body.acceleration.y << ", " << body.acceleration.z << ")\n";
    assert(body.acceleration.x == 5.0f && body.acceleration.y == 0.0f && body.acceleration.z == 0.0f);
    
    Vec3 force2(0, 8, 0);
    body.applyForce(force2);
    std::cout << "Applied additional force(0,8,0): acceleration=(" << body.acceleration.x << ", " << body.acceleration.y << ", " << body.acceleration.z << ")\n";
    assert(body.acceleration.x == 5.0f && body.acceleration.y == 4.0f && body.acceleration.z == 0.0f);
    
    body.clearForces();
    std::cout << "After clearForces(): acceleration=(" << body.acceleration.x << ", " << body.acceleration.y << ", " << body.acceleration.z << ")\n";
    assert(body.acceleration.x == 0.0f && body.acceleration.y == 0.0f && body.acceleration.z == 0.0f);
    
    RigidBody staticBody;
    staticBody.makeStatic();
    staticBody.applyForce(Vec3(100, 100, 100));
    std::cout << "Applied force(100,100,100) to static body: acceleration=(" << staticBody.acceleration.x << ", " << staticBody.acceleration.y << ", " << staticBody.acceleration.z << ")\n";
    assert(staticBody.acceleration.x == 0.0f && staticBody.acceleration.y == 0.0f && staticBody.acceleration.z == 0.0f);
}

void testImpulseApplication() {
    RigidBody body(Vec3(0, 0, 0), Vec3(1, 1, 1), 2.0f);
    body.velocity = Vec3(1, 2, 3);
    
    Vec3 impulse(4, 0, 0);
    body.applyImpulse(impulse);
    std::cout << "Initial velocity(1,2,3), applied impulse(4,0,0) to mass=2.0: new velocity=(" << body.velocity.x << ", " << body.velocity.y << ", " << body.velocity.z << ")\n";
    assert(body.velocity.x == 3.0f && body.velocity.y == 2.0f && body.velocity.z == 3.0f);
    
    RigidBody staticBody;
    staticBody.makeStatic();
    /*staticBody.velocity = Vec3(5, 5, 5);*/
    staticBody.applyImpulse(Vec3(10, 10, 10));
    std::cout << "Applied impulse(10,10,10) to static body: velocity=(" << staticBody.velocity.x << ", " << staticBody.velocity.y << ", " << staticBody.velocity.z << ")\n";
    assert(staticBody.velocity.x == 0.0f && staticBody.velocity.y == 0.0f && staticBody.velocity.z == 0.0f);
}

void testIntegration() {
    RigidBody body(Vec3(0, 0, 0), Vec3(1, 1, 1), 1.0f);
    body.velocity = Vec3(2, 3, 4);
    body.acceleration = Vec3(1, -2, 0.5f);
    
    std::cout << "Before integration: pos(" << body.position.x << ", " << body.position.y << ", " << body.position.z << ") vel(" << body.velocity.x << ", " << body.velocity.y << ", " << body.velocity.z << ")\n";
    std::cout << "Acceleration: (" << body.acceleration.x << ", " << body.acceleration.y << ", " << body.acceleration.z << ") deltaTime=0.1\n";
    
    body.integrate(0.1f);
    
    std::cout << "After integration: pos(" << body.position.x << ", " << body.position.y << ", " << body.position.z << ") vel(" << body.velocity.x << ", " << body.velocity.y << ", " << body.velocity.z << ")\n";
    std::cout << "Acceleration after integration: (" << body.acceleration.x << ", " << body.acceleration.y << ", " << body.acceleration.z << ")\n";
    
    assert(std::abs(body.velocity.x - 2.1f) < 0.001f);
    assert(std::abs(body.velocity.y - 2.8f) < 0.001f);
    assert(std::abs(body.velocity.z - 4.05f) < 0.001f);
    assert(std::abs(body.position.x - 0.21f) < 0.001f);
    assert(std::abs(body.position.y - 0.28f) < 0.001f);
    assert(std::abs(body.position.z - 0.405f) < 0.001f);
    assert(body.acceleration.x == 0.0f && body.acceleration.y == 0.0f && body.acceleration.z == 0.0f);
    
    RigidBody staticBody;
    staticBody.makeStatic();
    Vec3 originalPos = staticBody.position;
    staticBody.integrate(1.0f);
    std::cout << "Static body position after integration: (" << staticBody.position.x << ", " << staticBody.position.y << ", " << staticBody.position.z << ")\n";
    assert(staticBody.position.x == originalPos.x && staticBody.position.y == originalPos.y && staticBody.position.z == originalPos.z);
}

void testAABBGeneration() {
    RigidBody body(Vec3(10, 20, 30), Vec3(4, 6, 8), 1.0f);
    
    AABB aabb = body.getAABB();
    std::cout << "RigidBody at (10,20,30) with size(4,6,8): AABB min(" << aabb.min.x << ", " << aabb.min.y << ", " << aabb.min.z << ") max(" << aabb.max.x << ", " << aabb.max.y << ", " << aabb.max.z << ")\n";
    
    Vec3 center = body.getCenter();
    std::cout << "RigidBody center: (" << center.x << ", " << center.y << ", " << center.z << ")\n";
    
    assert(aabb.min.x == 8 && aabb.min.y == 17 && aabb.min.z == 26);
    assert(aabb.max.x == 12 && aabb.max.y == 23 && aabb.max.z == 34);
    assert(center.x == 10 && center.y == 20 && center.z == 30);
    
    std::cout << "Body hasInfiniteMass: " << (body.hasInfiniteMass() ? "true" : "false") << "\n";
    assert(!body.hasInfiniteMass());
    
    body.makeStatic();
    std::cout << "Static body hasInfiniteMass: " << (body.hasInfiniteMass() ? "true" : "false") << "\n";
    assert(body.hasInfiniteMass());
}

void runRigidBodyTests() {
    testRigidBodyConstruction();
    testMassManagement();
    testForceApplication();
    testImpulseApplication();
    testIntegration();
    testAABBGeneration();
}
