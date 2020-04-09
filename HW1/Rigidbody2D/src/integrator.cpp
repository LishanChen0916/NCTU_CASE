#include "integrator.hpp"

#include <algorithm>

#include "scene.hpp"

void ExplicitEulerIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
	// TODO
    
}

void RungeKuttaFourthIntegrator::Integrate(const std::vector<BodyRef>& _bodies, float deltaTime)
{
    if(scene == nullptr)
    {
        throw std::runtime_error("RungeKuttaFourthIntegrator has no target scene.");
    }

	// Use this temprorary ExplicitEulerIntegrator to do the integration
	// Required in RK4
	const auto originalIntegrator = scene->m_integrator;
	const auto originalDeltaTime = scene->m_deltaTime;
	scene->m_integrator = std::make_shared<ExplicitEulerIntegrator>();

	const float2 gravity(0.0f, -9.8f);
	// TODO


}
