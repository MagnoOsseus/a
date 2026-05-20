#pragma once

#include <glm/glm.hpp>
#include <algorithm>

// Alias so we can write Vec2 instead of glm::vec2 everywhere
using Vec2 = glm::vec2;

// 2D axis-aligned bounding box defined by its min and max corners
struct AABB
{
    Vec2 min = {};
    Vec2 max = {};

    float Width()  const { return max.x - min.x; }
    float Height() const { return max.y - min.y; }
    Vec2  Center() const { return (min + max) * 0.5f; }

    // Returns true if this box overlaps another box
    bool Intersects(const AABB& other) const
    {
        return min.x <= other.max.x && max.x >= other.min.x
            && min.y <= other.max.y && max.y >= other.min.y;
    }

    // Returns true if a point is inside this box
    bool Contains(Vec2 point) const
    {
        return point.x >= min.x && point.x <= max.x
            && point.y >= min.y && point.y <= max.y;
    }

    // Returns true if a circle overlaps this box.
    // Finds the closest point on the box to the circle center
    // and checks if it's within the radius.
    bool IntersectsCircle(Vec2 center, float radius) const
    {
        float cx = std::clamp(center.x, min.x, max.x);
        float cy = std::clamp(center.y, min.y, max.y);
        float dx = center.x - cx;
        float dy = center.y - cy;
        return (dx * dx + dy * dy) <= (radius * radius);
    }
};
