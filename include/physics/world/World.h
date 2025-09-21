#pragma once
#include "physics/dynamics/RigidBody.h"
#include <vector>
#include <memory>

namespace physics::world {

class World {
public:
    physics::math::Vec3 gravity;
    float timeStep;
    
    World();
    World(const physics::math::Vec3& gravity, float timeStep = 1.0f / 60.0f);
    
    void addBody(std::unique_ptr<physics::dynamics::RigidBody> body);
    void removeBody(size_t index);
    void clearBodies();
    
    void step();
    void step(float deltaTime);
    
    size_t getBodyCount() const;
    physics::dynamics::RigidBody* getBody(size_t index);
    const physics::dynamics::RigidBody* getBody(size_t index) const;
    
    void applyGravity();
    void integrateBodies(float deltaTime);
    
private:
    std::vector<std::unique_ptr<physics::dynamics::RigidBody>> bodies;
};

}
