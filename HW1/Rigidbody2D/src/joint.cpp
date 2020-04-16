#include "joint.hpp"

#include "GL/freeglut.h"

#include "rigidbody2D.hpp"
#include "util.hpp"

#include <iostream>

void SpringJoint::ApplyConstriant() const
{
	float center_of_mass_distance = linalg::distance(m_body0->GetPosition(), m_body1->GetPosition());
	float damper_coef = m_stiffness / 30;

	float2 xr = m_body0->GetPosition() - m_body1->GetPosition();
	float2 vr = m_body0->GetVelocity() - m_body1->GetVelocity();
	float2 l = xr / center_of_mass_distance;

	// Spring force
	m_body0->AddForce(-m_stiffness * (center_of_mass_distance - m_restLength) * l);
	m_body1->AddForce(-m_stiffness * (center_of_mass_distance - m_restLength) * -l);

	// Dameper force
	m_body0->AddForce(-damper_coef * linalg::dot(vr, xr) / center_of_mass_distance * l);
	m_body1->AddForce(-damper_coef * linalg::dot(vr, xr) / center_of_mass_distance * -l);

}

void SpringJoint::Render() const
{
    glPushMatrix();
    glPushAttrib(GL_CURRENT_BIT);
    {
        glBegin(GL_LINES);
        // red for spring joint
        glColor3f(1, 0, 0);
        glVertex2f( m_body0->GetPosition().x, m_body0->GetPosition().y );
        glVertex2f( m_body1->GetPosition().x, m_body1->GetPosition().y );
        glEnd();
    }
    glPopAttrib();
    glPopMatrix();
}

void DistanceJoint::ApplyConstriant() const
{
	float center_of_mass_distance = linalg::distance(m_body0->GetPosition(), m_body1->GetPosition());

	float2 normal = linalg::normalize(m_body1->GetPosition() - m_body0->GetPosition());

	float2 vr = m_body1->GetVelocity() - m_body0->GetVelocity();
	float vr_x = linalg::dot(vr, normal);
	float xr = center_of_mass_distance - m_restLength;
	float remove = vr_x + xr / m_deltaTime;

	float2 l = remove / (m_body0->GetInvMass() + m_body1->GetInvMass()) * normal;

	m_body0->AddVelocity(m_body0->GetInvMass() * l);
	m_body1->AddVelocity(-(m_body1->GetInvMass() * l));

}

void DistanceJoint::Render() const
{
    glPushMatrix();
    glPushAttrib(GL_CURRENT_BIT);
    {
        glBegin(GL_LINES);
        // green for distance joint
        glColor3f(0, 1, 0);
        glVertex2f( m_body0->GetPosition().x, m_body0->GetPosition().y );
        glVertex2f( m_body1->GetPosition().x, m_body1->GetPosition().y );
        glEnd();
    }
    glPopAttrib();
    glPopMatrix();
}