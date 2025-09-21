#pragma once
#include "physics/math/Vec3.h"

namespace physics::collision {

class AABB {
  public:
    physics::math::Vec3 min;
    physics::math::Vec3 max;

    AABB();
    AABB(const physics::math::Vec3& min, const physics::math::Vec3& max);
    AABB(const physics::math::Vec3& center, float width, float height, float depth);

    bool intersects(const AABB& other) const;
    bool contains(const physics::math::Vec3& point) const;

    physics::math::Vec3 getCenter() const;
    physics::math::Vec3 getSize() const;
    float getVolume() const;

    void expand(float amount);
    void expandToInclude(const physics::math::Vec3& point);
    void expandToInclude(const AABB& other);
};
}
