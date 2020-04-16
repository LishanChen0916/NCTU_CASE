#include "aabb.hpp"

#include "manifold.hpp"
#include "circle.hpp"
#include "collision.hpp"

#include "GL/freeglut.h"

Manifold AABB::accept(std::shared_ptr<const ShapeVisitor<Manifold>> visitor) const
{
    return visitor->visitAABB(shared_from_this());
}

Manifold AABB::visitAABB(std::shared_ptr<const AABB> _shape) const
{	
	float2 center_of_mass_distance = _shape->m_body->GetPosition() - m_body->GetPosition();
	float2 penetration_depth = (m_extent + _shape->m_extent) / 2 - linalg::abs(center_of_mass_distance);
	float2 nor;

	if (penetration_depth.x > penetration_depth.y) {
		nor = linalg::normalize(float2(0.0, center_of_mass_distance.y));
	}
	else {
		nor = linalg::normalize(float2(center_of_mass_distance.x, 0.0));
	}

	return Manifold(
		m_body,
		_shape->m_body,
		nor,
		(penetration_depth.x > penetration_depth.y) ? penetration_depth.y : penetration_depth.x,
		penetration_depth.x > 0 && penetration_depth.y > 0
	);

}

Manifold AABB::visitCircle(std::shared_ptr<const Circle> _shape) const
{
    auto manifold = CollisionHelper::GenerateManifold(
        shared_from_this(),
        _shape
    );

    return manifold;
}

void AABB::Render() const
{
    glPushMatrix();

    glTranslatef(m_body->GetPosition().x, m_body->GetPosition().y, 0);

    glBegin(GL_LINE_LOOP);
    {
        float2 half_extent = m_extent / 2.0f;

        glVertex2f(0 - half_extent[0], 0 - half_extent[1]);
        glVertex2f(0 - half_extent[0], 0 + half_extent[1]);
        glVertex2f(0 + half_extent[0], 0 + half_extent[1]);
        glVertex2f(0 + half_extent[0], 0 - half_extent[1]);
    }
    glEnd();

    glBegin(GL_POINTS);
    {
        glPushMatrix();

        glVertex2f(0, 0);

        glPopMatrix();
    }
    glEnd();

    glPopMatrix();
}