#include "collision.hpp"

#include <algorithm>
#include <iostream>

#include "linalg.h"

typedef linalg::aliases::float2 float2;

float2 AABB_Toward_Outside_Closest_Point(float2, float2, float2);
float2 Inside_Closest_Point_Toward_AABB(float2, float2, float2);
bool Is_Center_Inside_AABB(float2, float2, float2);

Manifold CollisionHelper::GenerateManifold(std::shared_ptr<const AABB> _a, std::shared_ptr<const Circle> _b)
{
	float2 AABB_min = _a->m_body->GetPosition() - _a->m_extent / 2;
	float2 AABB_max = _a->m_body->GetPosition() + _a->m_extent / 2;

	if (Is_Center_Inside_AABB(_b->m_body->GetPosition(), AABB_min, AABB_max)) {
		float2 contact_normal = Inside_Closest_Point_Toward_AABB(_b->m_body->GetPosition(), AABB_min, AABB_max);

		return Manifold(
			_a->m_body,
			_b->m_body,
			contact_normal,
			_b->m_radius + linalg::length(contact_normal),
			Is_Center_Inside_AABB(_b->m_body->GetPosition(), AABB_min, AABB_max)
		);
	}

	float2 closest_point = AABB_Toward_Outside_Closest_Point(_b->m_body->GetPosition(), AABB_min, AABB_max);
	float2 closest_point_to_centre = _b->m_body->GetPosition() - closest_point;

	return Manifold(
		_a->m_body,
		_b->m_body,
		linalg::normalize(closest_point_to_centre),
		_b->m_radius - linalg::length(closest_point_to_centre),
		_b->m_radius > linalg::length(closest_point_to_centre)
    );
}

float2 AABB_Toward_Outside_Closest_Point(float2 p, float2 b_min, float2 b_max)
{
	float2 q;
	for (int i = 0; i < 2; i++) {
		float v = p[i];
		if (v < b_min[i]) v = b_min[i];
		if (v > b_max[i]) v = b_max[i];
		q[i] = v;
	}

	return q;
}

float2 Inside_Closest_Point_Toward_AABB(float2 p, float2 b_min, float2 b_max)
{
	float2 Centre_To_Corner = linalg::distance2(p, b_min) > linalg::distance2(p, b_max) ? b_max - p : b_min - p;
	float2 Centre_To_Closest_Point = linalg::abs(Centre_To_Corner.x) > linalg::abs(Centre_To_Corner.y) ?
		linalg::normalize(float2(0.0, Centre_To_Corner.y)) : linalg::normalize(float2(Centre_To_Corner.x, 0.0));
	return Centre_To_Closest_Point;
}

bool Is_Center_Inside_AABB(float2 c, float2 b_min, float2 b_max)
{
	if (c.x > b_min.x && c.x < b_max.x && c.y > b_min.y && c.y < b_max.y) return true;
	else return false;
}