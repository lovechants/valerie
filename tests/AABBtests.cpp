#include "physics/collision/AABB.h"
#include <iostream>
#include <cassert>
#include <iomanip>

using namespace physics::collision;
using namespace physics::math;

void testAABBConstruction() {
    std::cout << std::fixed << std::setprecision(3);
    
    AABB box1;
    std::cout << "Default AABB: min(" << box1.min.x << ", " << box1.min.y << ", " << box1.min.z << ") max(" << box1.max.x << ", " << box1.max.y << ", " << box1.max.z << ")\n";
    
    AABB box2(Vec3(-1, -2, -3), Vec3(1, 2, 3));
    std::cout << "Min/Max AABB: min(" << box2.min.x << ", " << box2.min.y << ", " << box2.min.z << ") max(" << box2.max.x << ", " << box2.max.y << ", " << box2.max.z << ")\n";
    
    AABB box3(Vec3(0, 0, 0), 4, 6, 8);
    std::cout << "Center AABB (0,0,0) size(4,6,8): min(" << box3.min.x << ", " << box3.min.y << ", " << box3.min.z << ") max(" << box3.max.x << ", " << box3.max.y << ", " << box3.max.z << ")\n";
    
    assert(box3.min.x == -2 && box3.min.y == -3 && box3.min.z == -4);
    assert(box3.max.x == 2 && box3.max.y == 3 && box3.max.z == 4);
}

void testAABBIntersection() {
    AABB box1(Vec3(0, 0, 0), Vec3(2, 2, 2));
    AABB box2(Vec3(1, 1, 1), Vec3(3, 3, 3));
    AABB box3(Vec3(3, 3, 3), Vec3(5, 5, 5));
    AABB box4(Vec3(-1, -1, -1), Vec3(1, 1, 1));
    
    std::cout << "Box1 [0,0,0 to 2,2,2] intersects Box2 [1,1,1 to 3,3,3]: " << (box1.intersects(box2) ? "true" : "false") << "\n";
    std::cout << "Box1 [0,0,0 to 2,2,2] intersects Box3 [3,3,3 to 5,5,5]: " << (box1.intersects(box3) ? "true" : "false") << "\n";
    std::cout << "Box1 [0,0,0 to 2,2,2] intersects Box4 [-1,-1,-1 to 1,1,1]: " << (box1.intersects(box4) ? "true" : "false") << "\n";
    
    assert(box1.intersects(box2) == true);
    assert(box1.intersects(box3) == false);
    assert(box1.intersects(box4) == true);
}

void testAABBContains() {
    AABB box(Vec3(-1, -1, -1), Vec3(1, 1, 1));
    
    Vec3 points[] = {Vec3(0, 0, 0), Vec3(1, 1, 1), Vec3(-1, -1, -1), Vec3(1.1f, 0, 0), Vec3(0, 1.1f, 0)};
    bool expected[] = {true, true, true, false, false};
    
    for(int i = 0; i < 5; i++) {
        bool contains = box.contains(points[i]);
        std::cout << "Box [-1,-1,-1 to 1,1,1] contains (" << points[i].x << ", " << points[i].y << ", " << points[i].z << "): " << (contains ? "true" : "false") << "\n";
        assert(contains == expected[i]);
    }
}

void testAABBProperties() {
    AABB box(Vec3(-2, -3, -4), Vec3(2, 3, 4));
    
    Vec3 center = box.getCenter();
    std::cout << "Box [-2,-3,-4 to 2,3,4] center: (" << center.x << ", " << center.y << ", " << center.z << ")\n";
    
    Vec3 size = box.getSize();
    std::cout << "Box size: (" << size.x << ", " << size.y << ", " << size.z << ")\n";
    
    float volume = box.getVolume();
    std::cout << "Box volume: " << volume << "\n";
    
    assert(center.x == 0 && center.y == 0 && center.z == 0);
    assert(size.x == 4 && size.y == 6 && size.z == 8);
    assert(volume == 192);
}

void testAABBExpansion() {
    AABB box(Vec3(-1, -1, -1), Vec3(1, 1, 1));
    std::cout << "Original box: min(" << box.min.x << ", " << box.min.y << ", " << box.min.z << ") max(" << box.max.x << ", " << box.max.y << ", " << box.max.z << ")\n";
    
    box.expand(1.0f);
    std::cout << "After expand(1.0): min(" << box.min.x << ", " << box.min.y << ", " << box.min.z << ") max(" << box.max.x << ", " << box.max.y << ", " << box.max.z << ")\n";
    
    AABB box2(Vec3(0, 0, 0), Vec3(1, 1, 1));
    box2.expandToInclude(Vec3(3, -2, 4));
    std::cout << "Box [0,0,0 to 1,1,1] expanded to include (3,-2,4): min(" << box2.min.x << ", " << box2.min.y << ", " << box2.min.z << ") max(" << box2.max.x << ", " << box2.max.y << ", " << box2.max.z << ")\n";
    
    assert(box.min.x == -2 && box.min.y == -2 && box.min.z == -2);
    assert(box2.max.x == 3 && box2.min.y == -2 && box2.max.z == 4);
}

void runAABBTests() {
    testAABBConstruction();
    testAABBIntersection();
    testAABBContains();
    testAABBProperties();
    testAABBExpansion();
}
