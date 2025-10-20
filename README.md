# Valerie's Physics Engine Foundation

Geniunely have no clue why I am doing this.
- Trying to use very minimal dependencies


## Things to do and use with this Implementation
- Updated math in C++20 (inspired by glm)
- Render | Visual engine interface  
- Wireframe interface

## Vec3 - 3D Vector Mathematics

### Mathematical Foundation

A 3D vector represents a quantity with magnitude and direction in three-dimensional space.
- Position (location in 3D space)
- Velocity (rate of change of position)
- Acceleration (rate of change of velocity)
- Forces (cause acceleration according to Newton's laws)

### Core Operations

**Addition and Subtraction:**
```
v₁ + v₂ = (x₁ + x₂, y₁ + y₂, z₁ + z₂)
v₁ - v₂ = (x₁ - x₂, y₁ - y₂, z₁ - z₂)
```

**Scalar Multiplication:**
```
k * v = (k * x, k * y, k * z)
```

**Dot Product:**
```
v₁ · v₂ = x₁x₂ + y₁y₂ + z₁z₂ = |v₁||v₂|cos(θ)
```
The dot product measures how much two vectors point in the same direction.
- Angle calculations between vectors
- Projecting one vector onto another
- Determining if vectors are perpendicular (dot product = 0)

**Cross Product:**
```
v₁ × v₂ = (y₁z₂ - z₁y₂, z₁x₂ - x₁z₂, x₁y₂ - y₁x₂)
```
The cross product produces a vector perpendicular to both input vectors.
- Finding surface normals for collision detection
- Calculating torque in rotational physics
- Right-hand rule: i × j = k, j × k = i, k × i = j

**Length and Normalization:**
```
|v| = √(x² + y² + z²)
normalized(v) = v / |v|
```
Vector length represents magnitude. Normalization creates unit vectors (length = 1)
- Direction representation without magnitude
- Consistent force application regardless of input vector size

### Implementation Details

The Vec3 class stores components as `float x, y, z` for direct memory access and compatibility with graphics APIs. All operations are implemented as efficient inline functions without dynamic allocation.

## AABB - Axis-Aligned Bounding Box

### Mathematical Foundation

An Axis-Aligned Bounding Box represents a rectangular volume in 3D space where all faces are parallel to the coordinate axes. 
- Simple and fast intersection tests
- Conservative approximation of complex shapes
- Efficient broad-phase collision culling

### Intersection Mathematics

Two AABBs intersect if they overlap on all three axes:
```
intersects = (min₁.x ≤ max₂.x && max₁.x ≥ min₂.x) &&
             (min₁.y ≤ max₂.y && max₁.y ≥ min₂.y) &&
             (min₁.z ≤ max₂.z && max₁.z ≥ min₂.z)
```

This is known as the "separating axis theorem" for axis-aligned boxes. If the boxes don't overlap on any axis, they don't intersect.

### Point Containment

A point is inside an AABB if it lies within bounds on all axes:
```
contains = (point.x ≥ min.x && point.x ≤ max.x) &&
           (point.y ≥ min.y && point.y ≤ max.y) &&
           (point.z ≥ min.z && point.z ≤ max.z)
```

### Properties

**Center:** `center = (min + max) / 2`
**Size:** `size = max - min`
**Volume:** `volume = size.x * size.y * size.z`

### Implementation Details

The AABB class stores `Vec3 min, max` representing opposite corners. Three constructors support:
1. Default: Creates empty AABB at origin
2. Min/Max: Direct corner specification
3. Center/Dimensions: More intuitive for game objects

Expansion methods support broad-phase optimization by growing AABBs to encompass moving objects or combine multiple bounds.

## RigidBody - Dynamic Physics Object

### Mathematical Foundation

A rigid body is an idealized object that doesn't deform under forces. 
- Mass (resistance to acceleration)
- Position (location in world space)
- Velocity (rate of position change)
- Acceleration (rate of velocity change, caused by forces)

### Newton's Laws Implementation

**Newton's Second Law:** F = ma
```
acceleration = force / mass
acceleration += force * inverseMass  // More efficient computation
```

**Integration (Euler Method):**
```
velocity += acceleration * deltaTime
position += velocity * deltaTime
```

This is semi-implicit Euler integration, which is more stable than explicit Euler for physics simulation.

### Mass and Inverse Mass

The engine stores both `mass` and `inverseMass = 1/mass` because:
- Division is expensive, multiplication is fast
- Static objects have infinite mass (inverseMass = 0)
- Collision response often needs 1/mass calculations

### Force vs Impulse

**Forces** are continuous and accumulate over time:
- Gravity, spring forces, drag
- Applied each frame until cleared
- Effect: `acceleration += force * inverseMass`

**Impulses** are instantaneous velocity changes:
- Collision responses, explosions
- Applied once, immediate effect
- Effect: `velocity += impulse * inverseMass`

### Static vs Dynamic Bodies

**Static Bodies:**
- `mass = 0`, `inverseMass = 0`
- Never move regardless of forces applied
- Used for walls, floors, immovable objects

**Dynamic Bodies:**
- `mass > 0`, `inverseMass = 1/mass`
- Respond to forces and move according to physics

### Integration Process

Each physics timestep:
1. Apply forces (gravity, springs, etc.)
2. Integrate: update velocity from acceleration
3. Integrate: update position from velocity
4. Clear accumulated forces
5. Handle collisions (separate step)

### AABB Integration

RigidBody generates its collision AABB from position and size:
```
halfSize = size * 0.5
aabb.min = position - halfSize
aabb.max = position + halfSize
```

This assumes the position represents the center of mass and the object is axis-aligned.

## Component Relationships

### Vec3 → AABB
- AABB corners are Vec3 objects
- AABB operations use Vec3 arithmetic
- Size, center calculations return Vec3

### Vec3 → RigidBody
- Position, velocity, acceleration are Vec3
- Forces and impulses applied as Vec3
- Integration uses Vec3 operations

### AABB → RigidBody
- RigidBody generates AABB for collision detection
- AABB represents the collision bounds of the rigid body
- Position changes update the AABB automatically

### Physics Pipeline
1. **Vec3** provides mathematical foundation
2. **RigidBody** uses Vec3 for dynamics simulation
3. **AABB** uses Vec3 for collision bounds
4. Collision detection uses AABB intersection
5. Collision response applies impulses to RigidBody
6. Integration updates RigidBody position using Vec3 math

## World - Physics Simulation Container

### Mathematical Foundation

The World class implements a physics simulation container that manages multiple rigid bodies and applies global forces. It represents the complete physical environment where objects interact according to Newton's laws.

### Physics Simulation Loop

A physics simulation follows a standard pattern each frame:
1. **Force Application:** Apply global forces (gravity) to all dynamic bodies
2. **Integration:** Update velocities and positions using accumulated forces
3. **Collision Detection:** Check for intersecting objects (not yet implemented)
4. **Collision Response:** Resolve collisions with impulses (not yet implemented)

### Gravity Implementation

Gravity is implemented as a force rather than direct acceleration:
```
F_gravity = mass * gravity_acceleration
F_gravity = m * g
```

This approach is physically correct and allows for:
- Proper interaction with other forces
- Mass-dependent gravitational effects
- Easy modification of gravity strength per world

**Earth Gravity:** Default gravity of (0, -9.81, 0) m/s² represents standard Earth acceleration due to gravity in the negative Y direction.

### Time Integration

The World uses fixed timestep integration with a default of 1/60 second (60 FPS):
```
timeStep = 1.0 / 60.0 = 0.0167 seconds
```

**Fixed vs Variable Timestep:**
- Fixed timestep provides deterministic simulation idk if I want any non deterministic situations occurring yet without consideration
- Consistent physics behavior regardless of frame rate
- Simplifies collision detection and response

### Memory Management

The World uses `std::unique_ptr<RigidBody>` for automatic memory management:
- Clear ownership semantics (World owns the bodies)
- Automatic cleanup when bodies are removed or World is destroyed
- Move semantics prevent unnecessary copying
- Exception safety in construction and destruction

### Multi-Body Simulation

The World treats each RigidBody independently during force application and integration:
```cpp
// Apply gravity to all dynamic bodies
for (auto& body : bodies) {
    if (!body->isStatic) {
        Vec3 gravityForce = gravity * body->mass;
        body->applyForce(gravityForce);
    }
}

// Integrate all bodies
for (auto& body : bodies) {
    body->integrate(deltaTime);
}
```

This approach allows:
- Different masses to fall at the same rate (F = mg, a = F/m = g)
- Static bodies to remain unaffected by global forces
- Independent object behavior before collision interactions

### Implementation Details

**Body Storage:** Bodies are stored in a `std::vector` 

**Static Body Optimization:** The gravity application loop checks `isStatic` to skip unnecessary force calculations on immovable objects.

**Encapsulation:** The World separates concerns:
- `applyGravity()` handles global force application
- `integrateBodies()` handles position/velocity updates
- `step()` orchestrates the complete physics update

**Safe Access:** Body retrieval methods return nullptr for invalid indices rather than throwing exceptions

## Component Relationships

### Vec3 → AABB → RigidBody → World

The dependency hierarchy flows upward:
- **Vec3** provides mathematical primitives
- **AABB** uses Vec3 for collision bounds
- **RigidBody** uses Vec3 for physics state and AABB for collision
- **World** manages multiple RigidBody objects and applies global forces

### Physics Pipeline
1. **World** applies gravity forces to all **RigidBody** objects
2. **RigidBody** integrates forces using **Vec3** math operations
3. **RigidBody** updates its collision **AABB** based on new position
4. Future: **World** detects collisions between **AABB** bounds
5. Future: **World** resolves collisions with **RigidBody** impulse responses

### Data Flow
```
World.step() →
  World.applyGravity() →
    RigidBody.applyForce() →
      Vec3 force accumulation
  World.integrateBodies() →
    RigidBody.integrate() →
      Vec3 velocity/position updates →
        AABB bounds recalculation
```

## Design Decisions

### Why Force-Based Gravity?
- Physically accurate representation (F = mg)
- Consistent with other force sources (springs, motors)
- Allows proper force accumulation before integration
- Enables different gravity effects per body if needed

### Why Fixed Timestep?
- Deterministic simulation results
- Simplified collision detection algorithms
- Consistent physics behavior across different hardware
- Easier debugging and testing

### Why Unique Pointers?
- Clear ownership model (World owns bodies)
- Automatic memory management prevents leaks
- Move semantics avoid unnecessary copying
- Exception safety during dynamic allocation

### Why Separate Force/Integration Steps?
- Modular design allows different integration methods
- Clear separation of concerns (forces vs motion)
- Easier to add collision detection between steps
- Standard pattern in physics engines

## Performance Considerations

**Linear Complexity:** Both gravity application and integration are O(n) where n is the number of bodies. This scales well enough I think.

**Memory Locality:** Bodies stored in contiguous vector provide good cache performance during iteration.

**Static Body Optimization:** Checking `isStatic` flag avoids unnecessary calculations on immovable objects.

**Force Clearing:** RigidBody automatically clears forces after integration, preventing force accumulation bugs.

## Future Extensions

- **Collision Detection:** Broad-phase (spatial partitioning) and narrow-phase (shape-specific) collision
- **Collision Response:** Impulse-based collision resolution with restitution and friction
- **Constraints:** Spring joints, distance constraints, angular limits
- **Advanced Integration:** Runge-Kutta or Verlet integration for improved stability
- **Spatial Optimization:** Octree or grid-based broad-phase collision detection
- **Multithreading:** Parallel force application and integration for large object counts
- **Rendering Integration:** Abstract renderer interface for visualization
- **Source Movement:** Specialized player movement with air-strafing mechanics


