#pragma once

#include <vector>

#include "rigidbody2D.hpp"

class Scene;

class Integrator
{
protected:
    typedef linalg::aliases::float2 float2;
    typedef std::shared_ptr<RigidBody2D> BodyRef;

public:
    virtual void Integrate(const std::vector<BodyRef>& _bodies, float deltaTime) = 0;
};

class ExplicitEulerIntegrator : public Integrator
{
public:
    virtual void Integrate(const std::vector<BodyRef>& _bodies, float deltaTime) override;
};

class RungeKuttaFourthIntegrator : public Integrator
{
private:
    friend class Scene;
    
    struct StateStep
    {
        float2 position;
        float2 velocity;
    };

public:
    std::shared_ptr<Scene> scene;

    virtual void Integrate(const std::vector<BodyRef>& _bodies, float deltaTime) override;
};