#include "physics/collision/AABB.h"
#include <algorithm>

namespace physics::collision {

using Vec3 = physics::math::Vec3;

AABB::AABB() : min(0, 0, 0), max(0, 0, 0) {}

AABB::AABB(const Vec3& min, const Vec3& max) : min(min), max(max) {}

AABB::AABB(const Vec3& center, float width, float height, float depth) {
    Vec3 halfSize(width * 0.5f, height * 0.5f, depth * 0.5f);
    min = center - halfSize;
    max = center + halfSize;
}

bool AABB::intersects(const AABB& other) const {
    return (min.x <= other.max.x && max.x >= other.min.x) &&
           (min.y <= other.max.y && max.y >= other.min.y) &&
           (min.z <= other.max.z && max.z >= other.min.z);
}

bool AABB::contains(const Vec3& point) const {
    return (point.x >= min.x && point.x <= max.x) &&
           (point.y >= min.y && point.y <= max.y) &&
           (point.z >= min.z && point.z <= max.z);
}

Vec3 AABB::getCenter() const {
    return (min + max) * 0.5f;
}

Vec3 AABB::getSize() const {
    return max - min;
}

float AABB::getVolume() const {
    Vec3 size = getSize();
    return size.x * size.y * size.z;
}

void AABB::expand(float amount) {
    Vec3 expansion(amount, amount, amount);
    min = min - expansion;
    max = max + expansion;
}

void AABB::expandToInclude(const Vec3& point) {
    min.x = std::min(min.x, point.x);
    min.y = std::min(min.y, point.y);
    min.z = std::min(min.z, point.z);
    max.x = std::max(max.x, point.x);
    max.y = std::max(max.y, point.y);
    max.z = std::max(max.z, point.z);
}

void AABB::expandToInclude(const AABB& other) {
    expandToInclude(other.min);
    expandToInclude(other.max);
}

}
