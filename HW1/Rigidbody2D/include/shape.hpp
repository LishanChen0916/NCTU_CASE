#pragma once

#include <memory>

#include "manifold.hpp"
class RigidBody2D;

class AABB;
class Circle;

template <typename R>
class ShapeVisitor
{
public:
    virtual R visitAABB(std::shared_ptr<const AABB> _shape) const = 0;
    virtual R visitCircle(std::shared_ptr<const Circle> _shape) const = 0;
};

class Shape : public ShapeVisitor<Manifold>
{
public:
    std::shared_ptr<RigidBody2D> m_body;
public:
    virtual Manifold accept(std::shared_ptr<const ShapeVisitor<Manifold>> visitor) const = 0;

    virtual void Render() const = 0;
};