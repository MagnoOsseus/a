#include "CSpaceRefiner.h"
#include "Grid.h"
#include "Object.h"

#include <SDL2/SDL2_gfxPrimitives.h>
#include <cmath>
#include <algorithm>

// --- Public -----------------------------------------------------------------

void CSpaceRefiner::Clear()
{
    m_points.clear();
    m_levels.clear();
}

void CSpaceRefiner::Refine(const Grid& grid, const std::vector<Object>& objects, int levels, float angleDegrees)
{
    Clear();
    if (levels <= 0) return;

    const float startX  = grid.GetStartX();
    const float startY  = grid.GetStartY();
    const float screenW = grid.GetScreenW();
    const float screenH = grid.GetScreenH();

    const auto& mainSafe   = grid.GetCSpaceSafe();
    const auto& mainUnsafe = grid.GetCSpaceUnsafe();
    if (mainSafe.empty()) return;

    int   prevCols   = grid.GetCols();
    int   prevRows   = grid.GetRows();
    float prevCellSz = grid.GetCellSize();

    // Build initial uncertain mask from the main grid
    std::vector<bool> uncertainMask(static_cast<size_t>(prevCols) * prevRows, false);
    for (int r = 0; r < prevRows; ++r) {
        for (int c = 0; c < prevCols; ++c) {
            size_t idx = static_cast<size_t>(r) * prevCols + c;
            bool isSafe   = mainSafe[idx];
            bool isUnsafe = !mainUnsafe.empty() && mainUnsafe[idx];
            uncertainMask[idx] = !isSafe && !isUnsafe;
        }
    }

    // Iterative refinement: each level halves the cell size
    for (int level = 0; level < levels; ++level) {
        float subCellSz = prevCellSz * 0.5f;
        int subCols = static_cast<int>(std::ceil((screenW - startX) / subCellSz));
        int subRows = static_cast<int>(std::ceil((screenH - startY) / subCellSz));
        size_t subTotal = static_cast<size_t>(subCols) * subRows;

        // Map previous uncertain cells to their 4 sub-cell children
        std::vector<bool> testMask(subTotal, false);
        bool anyToTest = false;
        for (int r = 0; r < prevRows; ++r) {
            for (int c = 0; c < prevCols; ++c) {
                if (!uncertainMask[static_cast<size_t>(r) * prevCols + c]) continue;
                for (int dr = 0; dr < 2; ++dr) {
                    for (int dc = 0; dc < 2; ++dc) {
                        int sc = c * 2 + dc;
                        int sr = r * 2 + dr;
                        if (sc < subCols && sr < subRows) {
                            testMask[static_cast<size_t>(sr) * subCols + sc] = true;
                            anyToTest = true;
                        }
                    }
                }
            }
        }
        if (!anyToTest) break;

        // --- Rasterize all objects at this finer resolution ----------------
        std::vector<bool> sOcc(subTotal, false), sInn(subTotal, false);
        std::vector<bool> dOcc(subTotal, false), dInn(subTotal, false);

        for (const auto& obj : objects) {
            if (obj.IsDynamic()) {
                Object rotated = obj.GetRotated(angleDegrees);
                RasterizeObject(rotated, startX, startY, subCellSz, subCols, subRows,
                                screenW, screenH, sOcc, sInn, dOcc, dInn);
            } else {
                RasterizeObject(obj, startX, startY, subCellSz, subCols, subRows,
                                screenW, screenH, sOcc, sInn, dOcc, dInn);
            }
        }

        // --- Expand static occupied by 1 sub-cell (Minkowski) -------------
        std::vector<bool> expandedStatic(subTotal, false);
        for (int r = 0; r < subRows; ++r) {
            for (int c = 0; c < subCols; ++c) {
                if (!sOcc[static_cast<size_t>(r) * subCols + c]) continue;
                for (int dr = -1; dr <= 1; ++dr) {
                    for (int dc = -1; dc <= 1; ++dc) {
                        int nr = r + dr, nc = c + dc;
                        if (nr >= 0 && nr < subRows && nc >= 0 && nc < subCols)
                            expandedStatic[static_cast<size_t>(nr) * subCols + nc] = true;
                    }
                }
            }
        }

        // --- Gather dynamic cells and compute reference -------------------
        std::vector<std::pair<int, int>> dynCells, dynInnerCells;
        bool hasStaticInner = false;
        for (int r = 0; r < subRows; ++r) {
            for (int c = 0; c < subCols; ++c) {
                size_t idx = static_cast<size_t>(r) * subCols + c;
                if (dOcc[idx]) dynCells.emplace_back(c, r);
                if (dInn[idx]) dynInnerCells.emplace_back(c, r);
                if (sInn[idx]) hasStaticInner = true;
            }
        }
        if (dynCells.empty()) break;

        float refX = 0.0f, refY = 0.0f;
        for (const auto& [c, r] : dynCells) { refX += c; refY += r; }
        refX /= static_cast<float>(dynCells.size());
        refY /= static_cast<float>(dynCells.size());
        int refC = static_cast<int>(std::round(refX));
        int refR = static_cast<int>(std::round(refY));

        // --- C-Space passes (only for test positions) ---------------------
        std::vector<bool> subSafe(subTotal, false);
        std::vector<bool> subUnsafe(subTotal, false);

        for (int tr = 0; tr < subRows; ++tr) {
            for (int tc = 0; tc < subCols; ++tc) {
                if (!testMask[static_cast<size_t>(tr) * subCols + tc]) continue;

                int dx = tc - refC;
                int dy = tr - refR;

                // Pass 1: overlap with expanded static
                bool collision = false;
                for (const auto& [dc, dr] : dynCells) {
                    int nc = dc + dx, nr = dr + dy;
                    if (nc >= 0 && nc < subCols && nr >= 0 && nr < subRows
                        && expandedStatic[static_cast<size_t>(nr) * subCols + nc]) {
                        collision = true;
                        break;
                    }
                }
                if (!collision)
                    subSafe[static_cast<size_t>(tr) * subCols + tc] = true;

                // Pass 2: inner overlap
                if (hasStaticInner && !dynInnerCells.empty()) {
                    bool innerCol = false;
                    for (const auto& [dc, dr] : dynInnerCells) {
                        int nc = dc + dx, nr = dr + dy;
                        if (nc >= 0 && nc < subCols && nr >= 0 && nr < subRows
                            && sInn[static_cast<size_t>(nr) * subCols + nc]) {
                            innerCol = true;
                            break;
                        }
                    }
                    if (innerCol)
                        subUnsafe[static_cast<size_t>(tr) * subCols + tc] = true;
                }
            }
        }

        // --- Classify results and prepare next iteration ------------------
        std::vector<bool> newUncertain(subTotal, false);
        for (int r = 0; r < subRows; ++r) {
            for (int c = 0; c < subCols; ++c) {
                size_t idx = static_cast<size_t>(r) * subCols + c;
                if (!testMask[idx]) continue;

                Vec2 pos = { startX + c * subCellSz, startY + r * subCellSz };

                if (subSafe[idx]) {
                    m_points.push_back({ pos, true });
                } else if (subUnsafe[idx]) {
                    m_points.push_back({ pos, false });
                } else {
                    newUncertain[idx] = true;
                    // No dot for still-uncertain positions (same as main grid)
                }
            }
        }

        m_levels.push_back({ subCellSz, subCols, subRows, startX, startY,
                             std::move(subSafe), std::move(subUnsafe),
                             std::move(testMask) });

        uncertainMask = std::move(newUncertain);
        prevCols   = subCols;
        prevRows   = subRows;
        prevCellSz = subCellSz;
    }
}

// --- Rasterization (same logic as Grid::RasterizeObject) --------------------

void CSpaceRefiner::RasterizeObject(const Object& obj,
                                float startX, float startY,
                                float cellSize, int cols, int rows,
                                float screenW, float screenH,
                                std::vector<bool>& staticOccupied,
                                std::vector<bool>& staticInner,
                                std::vector<bool>& dynamicOccupied,
                                std::vector<bool>& dynamicInner)
{
    AABB box = obj.GetAABB();
    box.min.x = std::max(box.min.x, 0.0f);
    box.min.y = std::max(box.min.y, 0.0f);
    box.max.x = std::min(box.max.x, screenW);
    box.max.y = std::min(box.max.y, screenH);

    int colMin = std::max(0, static_cast<int>(std::floor((box.min.x - startX) / cellSize)));
    int colMax = std::min(cols - 1, static_cast<int>(std::floor((box.max.x - startX) / cellSize)));
    int rowMin = std::max(0, static_cast<int>(std::floor((box.min.y - startY) / cellSize)));
    int rowMax = std::min(rows - 1, static_cast<int>(std::floor((box.max.y - startY) / cellSize)));

    const bool isCircle = (obj.GetShapeType() == ShapeType::Circle);
    const Vec2 center   = obj.GetPosition();
    const float radius  = obj.GetRadius();

    std::vector<Vec2> worldVerts;
    if (!isCircle)
        worldVerts = obj.GetWorldVertices();

    for (int row = rowMin; row <= rowMax; ++row) {
        for (int col = colMin; col <= colMax; ++col) {
            AABB cell = {
                { startX + col * cellSize,       startY + row * cellSize },
                { startX + (col + 1) * cellSize, startY + (row + 1) * cellSize }
            };

            bool hit = isCircle
                ? Grid::CellOverlapsCircle(cell, center, radius)
                : Grid::CellOverlapsPolygon(cell, worldVerts);

            if (hit) {
                size_t idx = static_cast<size_t>(row) * cols + col;
                bool inside = isCircle
                    ? Grid::CellInsideCircle(cell, center, radius)
                    : Grid::CellInsidePolygon(cell, worldVerts);

                if (obj.IsDynamic()) {
                    dynamicOccupied[idx] = true;
                    if (inside) dynamicInner[idx] = true;
                } else {
                    staticOccupied[idx] = true;
                    if (inside) staticInner[idx] = true;
                }
            }
        }
    }
}


// --- Query ----------------------------------------------------------------

CSpaceQueryResult CSpaceRefiner::Query(Vec2 t, const Grid& grid) const
{
    const float startX = grid.GetStartX();
    const float startY = grid.GetStartY();

    // --- Level 1: main grid ------------------------------------------------
    const float mainCellSz = grid.GetCellSize();
    const int   mainCols   = grid.GetCols();
    const int   mainRows   = grid.GetRows();

    // Map the continuous translation to a discrete cell index
    int col = static_cast<int>(std::floor((t.x - startX) / mainCellSz));
    int row = static_cast<int>(std::floor((t.y - startY) / mainCellSz));

    // Out of bounds → no data, report undecided at level 1
    if (col < 0 || col >= mainCols || row < 0 || row >= mainRows)
        return { CSpaceStatus::Uncertain, 1 };

    const auto& mainSafe   = grid.GetCSpaceSafe();
    const auto& mainUnsafe = grid.GetCSpaceUnsafe();

    // C-Space hasn't been computed yet
    if (mainSafe.empty())
        return { CSpaceStatus::Uncertain, 1 };

    size_t idx = static_cast<size_t>(row) * mainCols + col;

    // Main grid already resolved this cell → return at level 1
    if (mainSafe[idx])
        return { CSpaceStatus::Safe, 1 };
    if (!mainUnsafe.empty() && mainUnsafe[idx])
        return { CSpaceStatus::Unsafe, 1 };

    // --- Levels 2..N: refinement levels (finest first) ---------------------

    for (int i = static_cast<int>(m_levels.size()) - 1; i >= 0; --i) {
        const auto& lvl = m_levels[i];

        // Map t to this level's finer grid
        int c = static_cast<int>(std::floor((t.x - lvl.startX) / lvl.cellSize));
        int r = static_cast<int>(std::floor((t.y - lvl.startY) / lvl.cellSize));

        if (c < 0 || c >= lvl.cols || r < 0 || r >= lvl.rows)
            continue;

        size_t lidx = static_cast<size_t>(r) * lvl.cols + c;

        // This cell wasn't evaluated at this level (it wasn't uncertain
        // at the previous level), so skip to a coarser level
        if (!lvl.tested[lidx])
            continue;

        int level = i + 2; 

        if (lvl.safe[lidx])
            return { CSpaceStatus::Safe, level };
        if (lvl.unsafe[lidx])
            return { CSpaceStatus::Unsafe, level };

        return { CSpaceStatus::Uncertain, level };
    }

    return { CSpaceStatus::Uncertain, 1 };
}

// --- Drawing ----------------------------------------------------------------

void CSpaceRefiner::Draw(SDL_Renderer* renderer) const
{
    for (const auto& pt : m_points) {
        Sint16 cx = static_cast<Sint16>(pt.pos.x);
        Sint16 cy = static_cast<Sint16>(pt.pos.y);
        if (pt.safe)
            filledCircleRGBA(renderer, cx, cy, 2, 0, 255, 0, 255);
        else
            filledCircleRGBA(renderer, cx, cy, 2, 255, 60, 60, 255);
    }
}
