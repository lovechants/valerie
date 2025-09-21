#include "physics/dynamics/RigidBody.h"

namespace physics::dynamics {

using Vec3 = physics::math::Vec3;
using AABB = physics::collision::AABB;

RigidBody::RigidBody() 
    : position(0, 0, 0), velocity(0, 0, 0), acceleration(0, 0, 0), size(1, 1, 1),
      mass(1.0f), inverseMass(1.0f), friction(0.7f), restitution(0.3f), 
      isStatic(false), onGround(false) {}

RigidBody::RigidBody(const Vec3& position, const Vec3& size, float mass)
    : position(position), velocity(0, 0, 0), acceleration(0, 0, 0), size(size),
      mass(mass), friction(0.7f), restitution(0.3f), isStatic(false), onGround(false) {
    setMass(mass);
}

void RigidBody::setMass(float mass) {
    this->mass = mass;
    if (mass > 0.0f) {
        inverseMass = 1.0f / mass;
        isStatic = false;
    } else {
        inverseMass = 0.0f;
        isStatic = true;
    }
}

void RigidBody::makeStatic() {
    setMass(0.0f);
    velocity = Vec3(0, 0, 0);
    acceleration = Vec3(0, 0, 0);
}

void RigidBody::makeDynamic(float mass) {
    setMass(mass);
}

void RigidBody::applyForce(const Vec3& force) {
    if (!isStatic && inverseMass > 0.0f) {
        acceleration += force * inverseMass;
    }
}

void RigidBody::applyImpulse(const Vec3& impulse) {
    if (!isStatic && inverseMass > 0.0f) {
        velocity += impulse * inverseMass;
    }
}

void RigidBody::clearForces() {
    acceleration = Vec3(0, 0, 0);
}

void RigidBody::integrate(float deltaTime) {
    if (isStatic) return;
    
    velocity += acceleration * deltaTime;
    position += velocity * deltaTime;
    clearForces();
}

AABB RigidBody::getAABB() const {
    Vec3 halfSize = size * 0.5f;
    return AABB(position - halfSize, position + halfSize);
}

Vec3 RigidBody::getCenter() const {
    return position;
}

bool RigidBody::hasInfiniteMass() const {
    return inverseMass == 0.0f;
}

}
