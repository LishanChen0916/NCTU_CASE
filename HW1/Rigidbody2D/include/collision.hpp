#pragma once

/**
 *  This is where all the collision detection and manifold generation code
 *  lives. Classes that implements the shape visitor interface should be
 *  calling helper functions here, instead of implementing its own collision
 *  detection method.
 */

#include "aabb.hpp"
#include "circle.hpp"

#include "manifold.hpp"

class CollisionHelper
{
public:
    // AABB to Circle
    static Manifold GenerateManifold(std::shared_ptr<const AABB> _a, std::shared_ptr<const Circle> _b);
};