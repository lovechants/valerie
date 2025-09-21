#include "physics/world/World.h"

namespace physics::world {

using Vec3 = physics::math::Vec3;
using RigidBody = physics::dynamics::RigidBody;

World::World() : gravity(0, -9.81f, 0), timeStep(1.0f / 60.0f) {}

World::World(const Vec3& gravity, float timeStep) 
    : gravity(gravity), timeStep(timeStep) {}

void World::addBody(std::unique_ptr<RigidBody> body) {
    bodies.push_back(std::move(body));
}

void World::removeBody(size_t index) {
    if (index < bodies.size()) {
        bodies.erase(bodies.begin() + index);
    }
}

void World::clearBodies() {
    bodies.clear();
}

void World::step() {
    step(timeStep);
}

void World::step(float deltaTime) {
    applyGravity();
    integrateBodies(deltaTime);
}

size_t World::getBodyCount() const {
    return bodies.size();
}

RigidBody* World::getBody(size_t index) {
    if (index < bodies.size()) {
        return bodies[index].get();
    }
    return nullptr;
}

const RigidBody* World::getBody(size_t index) const {
    if (index < bodies.size()) {
        return bodies[index].get();
    }
    return nullptr;
}

void World::applyGravity() {
    for (auto& body : bodies) {
        if (!body->isStatic) {
            Vec3 gravityForce = gravity * body->mass;
            body->applyForce(gravityForce);
        }
    }
}

void World::integrateBodies(float deltaTime) {
    for (auto& body : bodies) {
        body->integrate(deltaTime);
    }
}

}
