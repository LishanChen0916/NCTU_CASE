#include "collision.hpp"

#include <algorithm>
#include <iostream>

#include "linalg.h"

Manifold CollisionHelper::GenerateManifold(std::shared_ptr<const AABB> _a, std::shared_ptr<const Circle> _b)
{
	// TODO


	// This is a template return object, you should remove it and return your own Manifold
	return Manifold(
		_a->m_body,
		_b->m_body,
		linalg::aliases::float2(0.0f, 0.0f),
        0.0f,
        false
    );
}