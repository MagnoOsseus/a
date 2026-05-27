#include "CSpaceRefiner.h"
#include "Grid.h"

CSpaceQueryResult CSpaceRefiner::Query(Vec2 t, const Grid& grid) const
{
    return { grid.QueryCSpace(t), 1 };
}
