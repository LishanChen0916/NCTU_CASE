#include "integrator.hpp"

#include <algorithm>

#include "scene.hpp"


void ExplicitEulerIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
	const float2 gravity(0.0f, -9.8f);

	for (int i = 0; i < _bodies.size(); i++) {
		if (_bodies[i]->GetInvMass() == 0) {
			_bodies[i]->SetVelocity(float2(0.0, 0.0));
			continue;
		}
		_bodies[i]->AddForce(gravity);
		_bodies[i]->AddPosition(_bodies[i]->GetVelocity() * deltaTime);
		_bodies[i]->AddVelocity(_bodies[i]->GetForce() * _bodies[i]->GetInvMass() * deltaTime);
		_bodies[i]->SetForce(float2(0.0, 0.0));
	}
}

void RungeKuttaFourthIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
    if(scene == nullptr)
    {
        throw std::runtime_error("RungeKuttaFourthIntegrator has no target scene.");
    }

	const float2 gravity(0.0f, -9.8f);

	// The absolute value of position and velocity
	std::vector<StateStep> currentState(_bodies.size());

	// The delta value of each state
	std::vector<StateStep> deltaK1State(_bodies.size());
	std::vector<StateStep> deltaK2State(_bodies.size());
	std::vector<StateStep> deltaK3State(_bodies.size());
	std::vector<StateStep> deltaK4State(_bodies.size());

	for (int i = 0; i < _bodies.size(); i++) {
		if (_bodies[i]->GetInvMass() == 0) {
			_bodies[i]->SetVelocity(float2(0.0, 0.0));
			continue;
		}
		_bodies[i]->AddForce(gravity);

		// Explicit Euler Method
		deltaK1State[i].position = _bodies[i]->GetVelocity();
		deltaK1State[i].velocity = _bodies[i]->GetForce() * _bodies[i]->GetInvMass();

		// Compute K2
		deltaK2State[i].position = _bodies[i]->GetVelocity() + deltaK1State[i].velocity * deltaTime/ 2;
		deltaK2State[i].velocity = deltaK1State[i].velocity;

		// Compute K3
		deltaK3State[i].position = _bodies[i]->GetVelocity() + deltaK2State[i].velocity * deltaTime / 2;
		deltaK3State[i].velocity = deltaK1State[i].velocity;

		// Compute K4
		deltaK4State[i].position = _bodies[i]->GetVelocity() + deltaK3State[i].velocity * deltaTime;
		deltaK4State[i].velocity = deltaK1State[i].velocity;

		_bodies[i]->AddPosition((deltaK1State[i].position / 6 + deltaK2State[i].position / 3 + 
			deltaK3State[i].position / 3 + deltaK4State[i].position / 6) * deltaTime);

		_bodies[i]->AddVelocity((deltaK1State[i].velocity / 6 + deltaK2State[i].velocity / 3 +
			deltaK3State[i].velocity / 3 + deltaK4State[i].velocity / 6) * deltaTime);

		_bodies[i]->SetForce(float2(0.0, 0.0));
	}

}