#include "physics/math/Vec3.h"
#include <iostream>
#include <iomanip>


using namespace physics::math;

void testVec3ops() {
  std::cout << std::fixed << std::setprecision(3);

  Vec3 a(1.0f, 2.0f, 3.0f);
  Vec3 b(4.0f, 5.0f, 6.0f);

  std::cout << "3D Vec A = " << a.x << ", " << a.y << ", " << a.z << "\n";
  std::cout << "3D Vec B = " << b.x << ", " << b.y << ", " << b.z << "\n";

  Vec3 sum = a + b;
  std::cout << "a + b = " << sum.x << ", " << sum.y << ", " << sum.z << "\n";

  Vec3 diff = a - b;
  std::cout << "a - b = (" << diff.x << ", " << diff.y << ", " << diff.z << ")\n";
  
  Vec3 scaled = a * 2.5f;
  std::cout << "a * 2.5 = (" << scaled.x << ", " << scaled.y << ", " << scaled.z << ")\n\n";
  
  float dotProduct = a.dot(b);
  std::cout << "a · b = " << dotProduct << "\n";
  
  Vec3 crossProduct = a.cross(b);
  std::cout << "a × b = (" << crossProduct.x << ", " << crossProduct.y << ", " << crossProduct.z << ")\n\n";
  
  float lengthA = a.length();
  std::cout << "|a| = " << lengthA << "\n";
  std::cout << "|a|² = " << a.lengthSq() << "\n\n";
  
  Vec3 normalized = a.normalized();
  std::cout << "normalized a = (" << normalized.x << ", " << normalized.y << ", " << normalized.z << ")\n";
  std::cout << "|normalized a| = " << normalized.length() << "\n\n";
  
  Vec3 c = a;
  c += b;
  std::cout << "a += b: c = (" << c.x << ", " << c.y << ", " << c.z << ")\n";
  
  c *= 0.5f;
  std::cout << "c *= 0.5: c = (" << c.x << ", " << c.y << ", " << c.z << ")\n\n";
}

void testCrossProductProperties() {
    Vec3 i(1, 0, 0);
    Vec3 j(0, 1, 0);
    Vec3 k(0, 0, 1);
    
    Vec3 i_cross_j = i.cross(j);
    Vec3 j_cross_k = j.cross(k);
    Vec3 k_cross_i = k.cross(i);
    
    std::cout << "i × j = (" << i_cross_j.x << ", " << i_cross_j.y << ", " << i_cross_j.z << ") [should be (0, 0, 1)]\n";
    std::cout << "j × k = (" << j_cross_k.x << ", " << j_cross_k.y << ", " << j_cross_k.z << ") [should be (1, 0, 0)]\n";
    std::cout << "k × i = (" << k_cross_i.x << ", " << k_cross_i.y << ", " << k_cross_i.z << ") [should be (0, 1, 0)]\n\n";
}

void testEdgeCases() {
    Vec3 zero(0, 0, 0);
    std::cout << "Zero vector length: " << zero.length() << "\n";
    
    Vec3 zeroNormalized = zero.normalized();
    std::cout << "Normalized zero vector: (" << zeroNormalized.x << ", " << zeroNormalized.y << ", " << zeroNormalized.z << ")\n";
    
    zero.normalize();
    std::cout << "Zero vector after normalize(): (" << zero.x << ", " << zero.y << ", " << zero.z << ")\n\n";
}

void run_vec3_tests() {
    testVec3ops();
    testCrossProductProperties();
    testEdgeCases();
}
