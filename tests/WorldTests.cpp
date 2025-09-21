#include "physics/world/World.h"
#include <iostream>
#include <cassert>
#include <iomanip>
#include <cmath>

using namespace physics::world;
using namespace physics::dynamics;
using namespace physics::math;

void testWorldConstruction() {
    std::cout << std::fixed << std::setprecision(3);
    
    World world1;
    std::cout << "Default world gravity: (" << world1.gravity.x << ", " << world1.gravity.y << ", " << world1.gravity.z << ") timeStep=" << world1.timeStep << "\n";
    assert(world1.gravity.x == 0.0f && world1.gravity.y == -9.81f && world1.gravity.z == 0.0f);
    assert(std::abs(world1.timeStep - (1.0f/60.0f)) < 0.001f);
    
    World world2(Vec3(0, -20, 0), 0.02f);
    std::cout << "Custom world gravity: (" << world2.gravity.x << ", " << world2.gravity.y << ", " << world2.gravity.z << ") timeStep=" << world2.timeStep << "\n";
    assert(world2.gravity.y == -20.0f && world2.timeStep == 0.02f);
    
    std::cout << "Initial body count: " << world1.getBodyCount() << "\n";
    assert(world1.getBodyCount() == 0);
}

void testBodyManagement() {
    World world;
    
    auto body1 = std::make_unique<RigidBody>(Vec3(0, 10, 0), Vec3(1, 1, 1), 2.0f);
    auto body2 = std::make_unique<RigidBody>(Vec3(5, 15, 0), Vec3(2, 2, 2), 3.0f);
    auto body3 = std::make_unique<RigidBody>(Vec3(-5, 0, 0), Vec3(1, 1, 1), 0.0f);
    
    Vec3 body1Pos = body1->position;
    Vec3 body2Pos = body2->position;
    bool body3Static = body3->isStatic;
    
    world.addBody(std::move(body1));
    world.addBody(std::move(body2));
    world.addBody(std::move(body3));
    
    std::cout << "Added 3 bodies, count: " << world.getBodyCount() << "\n";
    assert(world.getBodyCount() == 3);
    
    RigidBody* retrievedBody1 = world.getBody(0);
    RigidBody* retrievedBody2 = world.getBody(1);
    RigidBody* retrievedBody3 = world.getBody(2);
    
    std::cout << "Body 0 position: (" << retrievedBody1->position.x << ", " << retrievedBody1->position.y << ", " << retrievedBody1->position.z << ") mass=" << retrievedBody1->mass << "\n";
    std::cout << "Body 1 position: (" << retrievedBody2->position.x << ", " << retrievedBody2->position.y << ", " << retrievedBody2->position.z << ") mass=" << retrievedBody2->mass << "\n";
    std::cout << "Body 2 position: (" << retrievedBody3->position.x << ", " << retrievedBody3->position.y << ", " << retrievedBody3->position.z << ") isStatic=" << (retrievedBody3->isStatic ? "true" : "false") << "\n";
    
    assert(retrievedBody1->position.x == body1Pos.x && retrievedBody1->position.y == body1Pos.y && retrievedBody1->position.z == body1Pos.z);
    assert(retrievedBody2->mass == 3.0f);
    assert(retrievedBody3->isStatic == body3Static);
    
    world.removeBody(1);
    std::cout << "After removing body 1, count: " << world.getBodyCount() << "\n";
    assert(world.getBodyCount() == 2);
    
    RigidBody* nullBody = world.getBody(10);
    std::cout << "Body at invalid index 10: " << (nullBody ? "valid" : "null") << "\n";
    assert(nullBody == nullptr);
}

void testGravityApplication() {
    World world(Vec3(0, -10, 0));
    
    auto dynamicBody = std::make_unique<RigidBody>(Vec3(0, 10, 0), Vec3(1, 1, 1), 2.0f);
    auto staticBody = std::make_unique<RigidBody>(Vec3(0, 0, 0), Vec3(1, 1, 1), 0.0f);
    
    world.addBody(std::move(dynamicBody));
    world.addBody(std::move(staticBody));
    
    world.applyGravity();
    
    RigidBody* dynamic = world.getBody(0);
    RigidBody* static_body = world.getBody(1);
    
    std::cout << "After gravity application:\n";
    std::cout << "Dynamic body (mass=2.0) acceleration: (" << dynamic->acceleration.x << ", " << dynamic->acceleration.y << ", " << dynamic->acceleration.z << ")\n";
    std::cout << "Static body acceleration: (" << static_body->acceleration.x << ", " << static_body->acceleration.y << ", " << static_body->acceleration.z << ")\n";
    
    assert(dynamic->acceleration.x == 0.0f && dynamic->acceleration.y == -10.0f && dynamic->acceleration.z == 0.0f);
    assert(static_body->acceleration.x == 0.0f && static_body->acceleration.y == 0.0f && static_body->acceleration.z == 0.0f);
}

void testWorldStep() {
    World world(Vec3(0, -10, 0), 0.1f);
    
    auto body = std::make_unique<RigidBody>(Vec3(0, 10, 0), Vec3(1, 1, 1), 1.0f);
    body->velocity = Vec3(2, 5, 0);
    
    world.addBody(std::move(body));
    
    RigidBody* testBody = world.getBody(0);
    
    std::cout << "Before step: pos(" << testBody->position.x << ", " << testBody->position.y << ", " << testBody->position.z << ") vel(" << testBody->velocity.x << ", " << testBody->velocity.y << ", " << testBody->velocity.z << ")\n";
    
    world.step();
    
    std::cout << "After step (dt=0.1): pos(" << testBody->position.x << ", " << testBody->position.y << ", " << testBody->position.z << ") vel(" << testBody->velocity.x << ", " << testBody->velocity.y << ", " << testBody->velocity.z << ")\n";
    std::cout << "Acceleration after step: (" << testBody->acceleration.x << ", " << testBody->acceleration.y << ", " << testBody->acceleration.z << ")\n";
    
    assert(std::abs(testBody->velocity.x - 2.0f) < 0.001f);
    assert(std::abs(testBody->velocity.y - 4.0f) < 0.001f);
    assert(std::abs(testBody->position.x - 0.2f) < 0.001f);
    assert(std::abs(testBody->position.y - 10.4f) < 0.001f);
    assert(testBody->acceleration.x == 0.0f && testBody->acceleration.y == 0.0f && testBody->acceleration.z == 0.0f);
}

void testMultiBodySimulation() {
    World world(Vec3(0, -9.81f, 0));
    
    auto body1 = std::make_unique<RigidBody>(Vec3(0, 20, 0), Vec3(1, 1, 1), 1.0f);
    auto body2 = std::make_unique<RigidBody>(Vec3(10, 25, 0), Vec3(1, 1, 1), 2.0f);
    auto ground = std::make_unique<RigidBody>(Vec3(0, -5, 0), Vec3(100, 1, 100), 0.0f);
    
    world.addBody(std::move(body1));
    world.addBody(std::move(body2));
    world.addBody(std::move(ground));
    
    std::cout << "Multi-body simulation (3 steps):\n";
    
    for (int i = 0; i < 3; i++) {
        world.step();
        
        RigidBody* b1 = world.getBody(0);
        RigidBody* b2 = world.getBody(1);
        RigidBody* g = world.getBody(2);
        
        std::cout << "Step " << (i+1) << ":\n";
        std::cout << "  Body1: pos(" << b1->position.x << ", " << b1->position.y << ", " << b1->position.z << ") vel(" << b1->velocity.x << ", " << b1->velocity.y << ", " << b1->velocity.z << ")\n";
        std::cout << "  Body2: pos(" << b2->position.x << ", " << b2->position.y << ", " << b2->position.z << ") vel(" << b2->velocity.x << ", " << b2->velocity.y << ", " << b2->velocity.z << ")\n";
        std::cout << "  Ground: pos(" << g->position.x << ", " << g->position.y << ", " << g->position.z << ") vel(" << g->velocity.x << ", " << g->velocity.y << ", " << g->velocity.z << ")\n";
        
        assert(g->velocity.y == 0.0f);
        assert(b1->velocity.y < 0.0f && b2->velocity.y < 0.0f);
    }
}

void testCustomTimeStep() {
    World world(Vec3(0, -10, 0));
    
    auto body = std::make_unique<RigidBody>(Vec3(0, 10, 0), Vec3(1, 1, 1), 1.0f);
    world.addBody(std::move(body));
    
    RigidBody* testBody = world.getBody(0);
    
    std::cout << "Custom timestep test:\n";
    std::cout << "Before step: vel(" << testBody->velocity.x << ", " << testBody->velocity.y << ", " << testBody->velocity.z << ")\n";
    
    world.step(0.2f);
    
    std::cout << "After step(0.2): vel(" << testBody->velocity.x << ", " << testBody->velocity.y << ", " << testBody->velocity.z << ")\n";
    
    assert(std::abs(testBody->velocity.y - (-2.0f)) < 0.001f);
}

void runWorldTests() {
    testWorldConstruction();
    testBodyManagement();
    testGravityApplication();
    testWorldStep();
    testMultiBodySimulation();
    testCustomTimeStep();
}
