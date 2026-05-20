#pragma once

#include "AABB.h"
#include "IDrawable.h"

#include <vector>
#include <cstdint>

// The kinds of shapes an Object can be
enum class ShapeType : uint8_t
{
    Triangle,
    Square,
    Circle,
    Polygon,
    Freeform
};

// A 2D shape that can be drawn and moved around.
// Vertices are stored in local space (relative to position).
class Object : public IDrawable
{
public:
    // --- Factory methods (each one builds the shape's local vertices) ---
    static Object CreateTriangle(Vec2 position, float size);
    static Object CreateSquare(Vec2 position, float size);
    static Object CreateCircle(Vec2 position, float radius, int segments = 32);
    static Object CreatePolygon(Vec2 position, const std::vector<Vec2>& vertices);
    static Object CreateFreeform(Vec2 position, const std::vector<Vec2>& points, bool closeCurve = false);

    // Draws the shape using SDL2_gfx. Color depends on state:
    //   green = static, cyan = dynamic, red = colliding
    void Draw(SDL_Renderer* renderer) const override;

    // Draw the object rotated by angleDegrees
    void DrawRotated(SDL_Renderer* renderer, float angleDegrees) const;

    // Draw the object rotated and translated (for preview)
    void DrawRotatedTranslated(SDL_Renderer* renderer, float angleDegrees, Vec2 offset) const;

    // Moves the object along its direction if it's dynamic
    void Update(float dt);

    // --- Getters ---
    ShapeType                GetShapeType() const { return m_shapeType; }
    Vec2                     GetPosition()  const { return m_position; }
    const std::vector<Vec2>& GetVertices()  const { return m_vertices; }
    float                    GetRadius()    const { return m_radius; }
    bool                     IsDynamic()    const { return m_dynamic; }
    bool                     IsColliding()  const { return m_colliding; }
    float                    GetSpeed()     const { return m_speed; }
    Vec2                     GetDirection() const { return m_direction; }

    // --- Setters ---
    void SetPosition(Vec2 position)     { m_position = position; }
    void SetDynamic(bool dynamic)       { m_dynamic = dynamic; }
    void SetColliding(bool colliding)   { m_colliding = colliding; }
    void SetSpeed(float speed)          { m_speed = speed; }
    void SetDirection(Vec2 direction)   { m_direction = direction; }

    // Returns vertices transformed to world space (position + local vertex)
    std::vector<Vec2> GetWorldVertices() const;

    // Returns the axis-aligned bounding box in world space
    AABB GetAABB() const;

    // Creates a rotated copy of this object (rotates around its center)
    Object GetRotated(float angleDegrees) const;

private:
    Object() = default;

    ShapeType         m_shapeType = ShapeType::Polygon;
    Vec2              m_position  = {};                  // center of the object in world space
    std::vector<Vec2> m_vertices;                        // vertices in local space (relative to position)
    float             m_radius = 0.0f;                   // only used by circles

    // --- Movement ---
    bool  m_dynamic   = false;           // static objects don't move
    bool  m_colliding = false;           // set each frame by the grid
    float m_speed     = 0.0f;            // pixels per second
    Vec2  m_direction = {1.0f, 0.0f};    // unit vector
};
