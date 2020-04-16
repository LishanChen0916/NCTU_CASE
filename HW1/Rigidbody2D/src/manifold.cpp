#include "manifold.hpp"

#include <iostream>

Manifold::Manifold(
    std::shared_ptr<RigidBody2D> _body0, 
    std::shared_ptr<RigidBody2D> _body1,
    float2 _normal,
    float _penetration,
    bool _isHit)
    : m_body0(_body0), m_body1(_body1), m_normal(_normal),
      m_penetration(_penetration), m_isHit(_isHit)
    {}

void Manifold::Resolve() const
{
	if (!m_isHit) return;

    float e = std::min(m_body0->m_restitution, m_body1->m_restitution);
	float2 vr = m_body1->GetVelocity() - m_body0->GetVelocity();

	// Solve the same direction of relative velocity and normal
	if (linalg::dot(vr, m_normal) > 0) return;

	float2 I = (1 + e) * m_normal * linalg::dot(vr, m_normal) / (m_body0->GetInvMass() + m_body1->GetInvMass());

	m_body0->AddVelocity(I * m_body0->GetInvMass());
	m_body1->AddVelocity(-I * m_body1->GetInvMass());
}

void Manifold::PositionalCorrection() const
{
	if (!m_isHit) return;
    const float percent = 0.4f; // usually 20% to 80%, when fps is 1/60
    const float slop = 0.01f;

	const float inv_mass_a = m_body0->GetInvMass();
	const float inv_mass_b = m_body1->GetInvMass();

    if(inv_mass_a == 0.0f && inv_mass_b == 0.0f)
        return;

    float2 correction = 
        (std::max( m_penetration - slop, 0.0f ) / (inv_mass_a + inv_mass_b))
        * percent * m_normal;

    m_body0->m_position -= inv_mass_a * correction;
    m_body1->m_position += inv_mass_b * correction;
}