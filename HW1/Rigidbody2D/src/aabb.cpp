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
    // TODO


	// This is a template return object, you should remove it and return your own Manifold
    return Manifold(
        m_body,
        _shape->m_body,
		float2(0.0f, 0.0f),
        0.0f,
        false
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