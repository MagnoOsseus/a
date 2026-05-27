#pragma once

#include "Grid.h"       // CSpaceStatus, Grid::QueryCSpace
#include "IDrawable.h"
#include "AABB.h"

#include <vector>

class Object;

struct CSpaceQueryResult
{
    CSpaceStatus status;
    int level;      // always 1; kept for API compatibility with App.cpp
};

// Thin wrapper that delegates C-Space queries to the Grid's own quadtree.
// Refinement is now built into Grid::ComputeCSpace() via adaptive subdivision,
// so Refine(), Draw() and Clear() are all no-ops.
class CSpaceRefiner : public IDrawable
{
public:
    void Refine(const Grid& /*grid*/, const std::vector<Object>& /*objects*/,
                int /*levels*/, float /*angleDegrees*/ = 0.0f) {}

    CSpaceQueryResult Query(Vec2 t, const Grid& grid) const;

    // No refined overlay to draw – Grid::DrawCSpace() handles visualisation.
    void Draw(SDL_Renderer* /*renderer*/) const override {}

    void Clear() {}
};
