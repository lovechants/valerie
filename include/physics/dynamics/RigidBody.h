#pragma once
#include "physics/math/Vec3.h"
#include "physics/collision/AABB.h"

namespace physics::dynamics {

class RigidBody {
public:
    physics::math::Vec3 position;
    physics::math::Vec3 velocity;
    physics::math::Vec3 acceleration;
    physics::math::Vec3 size;
    float mass;
    float inverseMass;
    float friction;
    float restitution;
    bool isStatic;
    bool onGround;
    
    RigidBody();
    RigidBody(const physics::math::Vec3& position, const physics::math::Vec3& size, float mass = 1.0f);
    
    void setMass(float mass);
    void makeStatic();
    void makeDynamic(float mass);
    
    void applyForce(const physics::math::Vec3& force);
    void applyImpulse(const physics::math::Vec3& impulse);
    void clearForces();
    
    void integrate(float deltaTime);
    
    physics::collision::AABB getAABB() const;
    physics::math::Vec3 getCenter() const;
    
    bool hasInfiniteMass() const;
};

}
