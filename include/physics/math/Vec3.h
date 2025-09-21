#pragma once

namespace physics::math {

class Vec3 {
  public:
    float x, y, z;

    Vec3();
    Vec3(float x, float y, float z);

    Vec3 operator+(const Vec3& other) const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator*(float scalar) const;
    Vec3& operator+=(const Vec3& other);
    Vec3& operator*=(float scalar);


    float dot(const Vec3& other) const;
    Vec3 cross(const Vec3& other) const;
    float length() const;
    float lengthSq() const;
    Vec3 normalized() const;
    void normalize();
};
}
