#pragma once

#include "AABB.h"
#include "IDrawable.h"

#include <vector>

class Grid;
class Object;

enum class CSpaceStatus { Safe, Unsafe, Uncertain };

struct CSpaceQueryResult
{
    CSpaceStatus status;
    int level;      // 1 = main grid, 2 = first refinement, etc.
};

// Adaptive multi-resolution refinement for uncertain C-Space cells.
class CSpaceRefiner : public IDrawable
{
public:
    // Refine uncertain C-Space cells.
    // Each level halves the cell size and re-runs the same algorithm.
    // Dynamic objects are rotated by angleDegrees before rasterization.
    void Refine(const Grid& grid, const std::vector<Object>& objects, int levels, float angleDegrees = 0.0f);

    CSpaceQueryResult Query(Vec2 t, const Grid& grid) const;

    // Draw refined results: green/red dots at sub-cell corners
    void Draw(SDL_Renderer* renderer) const override;

    void Clear();

private:
    struct RefinedPoint
    {
        Vec2 pos;    // screen position (sub-cell corner, like the main grid)
        bool safe;   // true = green, false = red
    };

    std::vector<RefinedPoint> m_points;

    struct RefinementLevel
    {
        float cellSize;
        int cols, rows;
        float startX, startY;
        std::vector<bool> safe;
        std::vector<bool> unsafe;
        std::vector<bool> tested;
    };
    std::vector<RefinementLevel> m_levels;

    // Rasterize a single object onto a sub-grid (same logic as Grid::RasterizeObject)
    static void RasterizeObject(const Object& obj,
                                float startX, float startY,
                                float cellSize, int cols, int rows,
                                float screenW, float screenH,
                                std::vector<bool>& staticOccupied,
                                std::vector<bool>& staticInner,
                                std::vector<bool>& dynamicOccupied,
                                std::vector<bool>& dynamicInner);
};
