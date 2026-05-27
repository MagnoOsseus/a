#include "Grid.h"
#include "Object.h"

#include <SDL2/SDL2_gfxPrimitives.h>
#include <cmath>
#include <algorithm>

namespace
{
constexpr std::size_t CellIndex(int col, int row, int cols)
{
    return static_cast<std::size_t>(row) * static_cast<std::size_t>(cols) + static_cast<std::size_t>(col);
}
}

// --- Public -------------------------------------------------------------

// Rebuild the grid and rasterize all objects onto it
// Dynamic objects are rotated by angleDegrees before rasterization
void Grid::Build(float screenW, float screenH, float cellSize, const std::vector<Object>& objects, Vec2 origin, float angleDegrees)
{
    m_width  = screenW;
    m_height = screenH;
    m_cellSize = cellSize;
    m_originX  = origin.x;
    m_originY  = origin.y;

    // Snap grid lines to the origin
    float rx = std::fmod(origin.x, cellSize);
    if (rx < 0) rx += cellSize;
    float ry = std::fmod(origin.y, cellSize);
    if (ry < 0) ry += cellSize;
    m_startX = rx - cellSize;
    m_startY = ry - cellSize;

    int newCols = static_cast<int>(std::ceil((screenW - m_startX) / cellSize));
    int newRows = static_cast<int>(std::ceil((screenH - m_startY) / cellSize));

    // Reset C-Space if grid size changed
    if (newCols != m_cols || newRows != m_rows) {
        m_cspaceSafe.clear();
        m_cspaceUnsafe.clear();
    }

    m_cols = newCols;
    m_rows = newRows;

    const size_t totalCells = static_cast<size_t>(m_cols) * static_cast<size_t>(m_rows);

    m_staticOccupied.assign(totalCells, false);
    m_dynamicOccupied.assign(totalCells, false);
    m_staticInner.assign(totalCells, false);
    m_dynamicInner.assign(totalCells, false);
    m_staticGray.assign(totalCells, false);
    m_dynamicGray.assign(totalCells, false);
    ResetSpatialIndex();

    // Mark which cells each object covers (rotate dynamic objects first)
    for (const auto& obj : objects) {
        if (obj.IsDynamic()) {
            Object rotated = obj.GetRotated(angleDegrees);
            RasterizeObject(rotated);
        } else {
            RasterizeObject(obj);
        }
    }

    SyncOccupancyArrays();
}

void Grid::Clear()
{
    m_cellEntries.clear();
    m_cellLookup.clear();
    m_occupancyTree.reset();
    m_staticOccupied.clear();
    m_dynamicOccupied.clear();
    m_staticInner.clear();
    m_dynamicInner.clear();
    m_staticGray.clear();
    m_dynamicGray.clear();
    m_cspaceSafe.clear();
    m_cspaceUnsafe.clear();
    m_cols = 0;
    m_rows = 0;
}

// Draw grid cells colored by what occupies them
void Grid::Draw(SDL_Renderer* renderer) const
{
    if (m_cols == 0 || m_rows == 0) return;

    for (int row = 0; row < m_rows; ++row) {
        for (int col = 0; col < m_cols; ++col) {
            Sint16 x1 = static_cast<Sint16>(m_startX + col * m_cellSize);
            Sint16 y1 = static_cast<Sint16>(m_startY + row * m_cellSize);
            Sint16 x2 = static_cast<Sint16>(m_startX + (col + 1) * m_cellSize);
            Sint16 y2 = static_cast<Sint16>(m_startY + (row + 1) * m_cellSize);

            size_t idx = static_cast<size_t>(row) * m_cols + col;
            bool sOcc   = m_staticOccupied[idx];
            bool dOcc   = m_dynamicOccupied[idx];
            bool sInner = m_staticInner[idx];
            bool dInner = m_dynamicInner[idx];
            bool sGray  = m_staticGray[idx];
            bool dGray  = m_dynamicGray[idx];

            if (sOcc && dOcc) {
                // Both objects overlap in this cell
                if (sGray || dGray) {
                    // At least one is a boundary cell → orange boundary
                    boxRGBA(renderer, x1, y1, x2, y2, 255, 140, 0, 80);
                    rectangleRGBA(renderer, x1, y1, x2, y2, 255, 140, 0, 220);
                } else {
                    // Both interiors → solid orange
                    boxRGBA(renderer, x1, y1, x2, y2, 255, 140, 0, 120);
                    rectangleRGBA(renderer, x1, y1, x2, y2, 255, 140, 0, 220);
                }
            } else if (sOcc) {
                if (sGray) {
                    // Gray: static boundary
                    boxRGBA(renderer, x1, y1, x2, y2, 255, 255, 255, 30);
                    rectangleRGBA(renderer, x1, y1, x2, y2, 0, 200, 0, 200);
                } else {
                    // Black: static interior
                    boxRGBA(renderer, x1, y1, x2, y2, 0, 255, 0, 60);
                    rectangleRGBA(renderer, x1, y1, x2, y2, 0, 255, 0, 120);
                }
            } else if (dOcc) {
                if (dGray) {
                    // Gray: dynamic boundary
                    boxRGBA(renderer, x1, y1, x2, y2, 255, 255, 255, 30);
                    rectangleRGBA(renderer, x1, y1, x2, y2, 0, 200, 200, 200);
                } else {
                    // Black: dynamic interior
                    boxRGBA(renderer, x1, y1, x2, y2, 0, 255, 255, 60);
                    rectangleRGBA(renderer, x1, y1, x2, y2, 0, 255, 255, 120);
                }
            } else {
                // White: exterior
                rectangleRGBA(renderer, x1, y1, x2, y2, 50, 50, 50, 40);
            }
        }
    }
}

// --- Configuration Space ------------------------------------------------

void Grid::ComputeCSpace()
{
    const size_t total = static_cast<size_t>(m_cols) * m_rows;
    m_cspaceSafe.assign(total, false);

    // Gather dynamic and static cell positions
    std::vector<std::pair<int, int>> dynCells;
    bool hasStatic = false;

    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
            size_t idx = static_cast<size_t>(r) * m_cols + c;
            if (m_dynamicOccupied[idx])
                dynCells.emplace_back(c, r);
            if (m_staticOccupied[idx])
                hasStatic = true;
        }
    }

    if (dynCells.empty()) return;

    // No obstacles means everywhere is safe
    if (!hasStatic) {
        m_cspaceSafe.assign(total, true);
        return;
    }

    // Grow static cells by 1 in each direction (Minkowski-ish)
    std::vector<bool> expandedStatic(total, false);
    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
            if (!m_staticOccupied[static_cast<size_t>(r) * m_cols + c])
                continue;
            for (int dr = -1; dr <= 1; ++dr) {
                for (int dc = -1; dc <= 1; ++dc) {
                    int nr = r + dr;
                    int nc = c + dc;
                    if (nr >= 0 && nr < m_rows && nc >= 0 && nc < m_cols)
                        expandedStatic[static_cast<size_t>(nr) * m_cols + nc] = true;
                }
            }
        }
    }

    // Use the dynamic object's centroid as reference point
    float refX = 0.0f, refY = 0.0f;
    for (const auto& [c, r] : dynCells) {
        refX += c;
        refY += r;
    }
    refX /= static_cast<float>(dynCells.size());
    refY /= static_cast<float>(dynCells.size());
    int refC = static_cast<int>(std::round(refX));
    int refR = static_cast<int>(std::round(refY));
    m_refScreenPos = { m_startX + refC * m_cellSize, m_startY + refR * m_cellSize };

    // First pass: check if placing the object here causes a collision
    for (int tr = 0; tr < m_rows; ++tr) {
        int dy = tr - refR;
        for (int tc = 0; tc < m_cols; ++tc) {
            int dx = tc - refC;

            bool collision = false;
            for (const auto& [dc, dr] : dynCells) {
                int nc = dc + dx;
                int nr = dr + dy;
                if (nc >= 0 && nc < m_cols && nr >= 0 && nr < m_rows
                    && expandedStatic[static_cast<size_t>(nr) * m_cols + nc]) {
                    collision = true;
                    break;
                }
            }

            if (!collision)
                m_cspaceSafe[static_cast<size_t>(tr) * m_cols + tc] = true;
        }
    }

    // Second pass: definite collisions using only inner cells
    m_cspaceUnsafe.assign(total, false);

    std::vector<std::pair<int, int>> dynInnerCells;
    bool hasStaticInner = false;
    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
            size_t idx = static_cast<size_t>(r) * m_cols + c;
            if (m_dynamicInner[idx])
                dynInnerCells.emplace_back(c, r);
            if (m_staticInner[idx])
                hasStaticInner = true;
        }
    }

    if (!dynInnerCells.empty() && hasStaticInner) {
        for (int tr = 0; tr < m_rows; ++tr) {
            int dy = tr - refR;
            for (int tc = 0; tc < m_cols; ++tc) {
                int dx = tc - refC;

                bool collision = false;
                for (const auto& [dc, dr] : dynInnerCells) {
                    int nc = dc + dx;
                    int nr = dr + dy;
                    if (nc >= 0 && nc < m_cols && nr >= 0 && nr < m_rows
                        && m_staticInner[static_cast<size_t>(nr) * m_cols + nc]) {
                        collision = true;
                        break;
                    }
                }

                if (collision)
                    m_cspaceUnsafe[static_cast<size_t>(tr) * m_cols + tc] = true;
            }
        }
    }
}

// Draw C-Space results (green=safe, red=collision)
void Grid::DrawCSpace(SDL_Renderer* renderer) const
{
    if (m_cols == 0 || m_rows == 0) return;

    bool hasSafe   = !m_cspaceSafe.empty();
    bool hasUnsafe = !m_cspaceUnsafe.empty();

    for (int row = 0; row < m_rows; ++row) {
        for (int col = 0; col < m_cols; ++col) {
            Sint16 x1 = static_cast<Sint16>(m_startX + col * m_cellSize);
            Sint16 y1 = static_cast<Sint16>(m_startY + row * m_cellSize);
            Sint16 x2 = static_cast<Sint16>(m_startX + (col + 1) * m_cellSize);
            Sint16 y2 = static_cast<Sint16>(m_startY + (row + 1) * m_cellSize);

            rectangleRGBA(renderer, x1, y1, x2, y2, 50, 50, 50, 40);

            size_t idx = static_cast<size_t>(row) * m_cols + col;
            Sint16 cx = static_cast<Sint16>(m_startX + col * m_cellSize);
            Sint16 cy = static_cast<Sint16>(m_startY + row * m_cellSize);

            if (hasSafe && m_cspaceSafe[idx]) {
                // Safe: green
                filledCircleRGBA(renderer, cx, cy, 2, 0, 255, 0, 255);
            } else if (hasUnsafe && m_cspaceUnsafe[idx]) {
                // Collision: red
                filledCircleRGBA(renderer, cx, cy, 2, 255, 60, 60, 255);
            }
            // Uncertain: no marking
        }
    }
}

// Returns the max distance from origin to the center of the farthest dynamic inner cell
float Grid::GetMaxDynamicInnerDistance(Vec2 origin) const
{
    float maxDist = 0.0f;
    for (int row = 0; row < m_rows; ++row) {
        for (int col = 0; col < m_cols; ++col) {
            if (!m_dynamicInner[static_cast<size_t>(row) * m_cols + col]) continue;

            // Center of this cell
            float cx = m_startX + (col + 0.5f) * m_cellSize;
            float cy = m_startY + (row + 0.5f) * m_cellSize;

            float dx = cx - origin.x;
            float dy = cy - origin.y;
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist > maxDist) maxDist = dist;
        }
    }
    return maxDist;
}

// Draw a crosshair at the grid origin
void Grid::DrawOrigin(SDL_Renderer* renderer) const
{
    Sint16 ox = static_cast<Sint16>(m_originX);
    Sint16 oy = static_cast<Sint16>(m_originY);
    Sint16 len = static_cast<Sint16>(m_cellSize * 0.4f);
    thickLineRGBA(renderer, ox - len, oy, ox + len, oy, 2, 255, 255, 0, 255);
    thickLineRGBA(renderer, ox, oy - len, ox, oy + len, 2, 255, 255, 0, 255);
    filledCircleRGBA(renderer, ox, oy, 3, 255, 255, 0, 255);
}

// --- Rasterization ------------------------------------------------------

void Grid::ResetSpatialIndex()
{
    m_cellEntries.clear();
    m_cellLookup.clear();

    if (m_cols <= 0 || m_rows <= 0 || m_cellSize <= 0.0f) {
        m_occupancyTree.reset();
        return;
    }

    m_occupancyTree = std::make_unique<OccupancyTree>(
        quadtree::Box<float>(m_startX, m_startY, m_cols * m_cellSize, m_rows * m_cellSize)
    );
}

Grid::CellEntry& Grid::EnsureCellEntry(int col, int row)
{
    const std::size_t idx = CellIndex(col, row, m_cols);
    auto it = m_cellLookup.find(idx);
    if (it != m_cellLookup.end())
        return *it->second;

    auto entry = std::make_unique<CellEntry>();
    entry->col = col;
    entry->row = row;
    entry->box = quadtree::Box<float>(
        m_startX + col * m_cellSize,
        m_startY + row * m_cellSize,
        m_cellSize,
        m_cellSize
    );

    CellEntry* raw = entry.get();
    m_cellEntries.push_back(std::move(entry));
    m_cellLookup.emplace(idx, raw);
    if (m_occupancyTree)
        m_occupancyTree->add(raw);

    return *raw;
}

void Grid::SyncOccupancyArrays()
{
    const size_t totalCells = static_cast<size_t>(m_cols) * m_rows;

    m_staticOccupied.assign(totalCells, false);
    m_dynamicOccupied.assign(totalCells, false);
    m_staticInner.assign(totalCells, false);
    m_dynamicInner.assign(totalCells, false);
    m_staticGray.assign(totalCells, false);
    m_dynamicGray.assign(totalCells, false);

    for (const auto& entry : m_cellEntries) {
        const size_t idx = CellIndex(entry->col, entry->row, m_cols);
        m_staticOccupied[idx] = entry->state.staticOccupied;
        m_dynamicOccupied[idx] = entry->state.dynamicOccupied;
        m_staticInner[idx] = entry->state.staticInner;
        m_dynamicInner[idx] = entry->state.dynamicInner;
        m_staticGray[idx] = entry->state.staticGray;
        m_dynamicGray[idx] = entry->state.dynamicGray;
    }
}

// Mark which cells an object occupies
void Grid::RasterizeObject(const Object& obj)
{
    const bool isCircle   = (obj.GetShapeType() == ShapeType::Circle);
    const bool isFreeform = (obj.GetShapeType() == ShapeType::Freeform);
    const Vec2  center = obj.GetPosition();
    const float radius = obj.GetRadius();

    if (isFreeform) {
        // Freeform polygons: Daum 2012 Gray/flood-fill method
        auto worldVerts = obj.GetWorldVertices();
        RasterizePolygonDaum(obj, worldVerts);
        return;
    }

    // All other shapes: original per-cell overlap approach
    AABB box = obj.GetAABB();
    box.min.x = std::max(box.min.x, 0.0f);
    box.min.y = std::max(box.min.y, 0.0f);
    box.max.x = std::min(box.max.x, m_width);
    box.max.y = std::min(box.max.y, m_height);

    int colMin = std::max(0, static_cast<int>(std::floor((box.min.x - m_startX) / m_cellSize)));
    int colMax = std::min(m_cols - 1, static_cast<int>(std::floor((box.max.x - m_startX) / m_cellSize)));
    int rowMin = std::max(0, static_cast<int>(std::floor((box.min.y - m_startY) / m_cellSize)));
    int rowMax = std::min(m_rows - 1, static_cast<int>(std::floor((box.max.y - m_startY) / m_cellSize)));

    std::vector<Vec2> worldVerts;
    if (!isCircle)
        worldVerts = obj.GetWorldVertices();

    for (int row = rowMin; row <= rowMax; ++row) {
        for (int col = colMin; col <= colMax; ++col) {
            AABB cell = {
                { m_startX + col * m_cellSize,       m_startY + row * m_cellSize },
                { m_startX + (col + 1) * m_cellSize, m_startY + (row + 1) * m_cellSize }
            };

            bool hit = isCircle
                ? CellOverlapsCircle(cell, center, radius)
                : CellOverlapsPolygon(cell, worldVerts);

            if (hit) {
                CellEntry& entry = EnsureCellEntry(col, row);
                bool inside = isCircle
                    ? CellInsideCircle(cell, center, radius)
                    : CellInsidePolygon(cell, worldVerts);

                if (obj.IsDynamic()) {
                    entry.state.dynamicOccupied = true;
                    if (inside) entry.state.dynamicInner = true;
                } else {
                    entry.state.staticOccupied = true;
                    if (inside) entry.state.staticInner = true;
                }
            }
        }
    }
}


void Grid::RasterizePolygonDaum(const Object& obj, const std::vector<Vec2>& worldVerts)
{
    const int n = static_cast<int>(worldVerts.size());
    if (n < 2) return;

    // Bounding region in grid coordinates (clamped to grid)
    AABB box = obj.GetAABB();
    int colMin = std::max(0, static_cast<int>(std::floor((box.min.x - m_startX) / m_cellSize)));
    int colMax = std::min(m_cols - 1, static_cast<int>(std::ceil((box.max.x - m_startX) / m_cellSize)));
    int rowMin = std::max(0, static_cast<int>(std::floor((box.min.y - m_startY) / m_cellSize)));
    int rowMax = std::min(m_rows - 1, static_cast<int>(std::ceil((box.max.y - m_startY) / m_cellSize)));

    // Extend bounding region by 1 cell so the flood-fill can enter from all sides
    int cMin = std::max(0,         colMin - 1);
    int cMax = std::min(m_cols - 1, colMax + 1);
    int rMin = std::max(0,         rowMin - 1);
    int rMax = std::min(m_rows - 1, rowMax + 1);

    int regionW = cMax - cMin + 1;
    int regionH = rMax - rMin + 1;
    int regionSize = regionW * regionH;

    // Cell color within the local region
    // 0 = Unknown, 1 = Gray (boundary), 2 = White (exterior)
    std::vector<uint8_t> color(regionSize, 0);

    auto localIdx = [&](int col, int row) {
        return (row - rMin) * regionW + (col - cMin);
    };


    for (int e = 0; e < n; ++e) {
        const Vec2& A = worldVerts[e];
        const Vec2& B = worldVerts[(e + 1) % n];

        // AABB of this edge (clamped to search region)
        int ec0 = std::max(cMin, static_cast<int>(std::floor((std::min(A.x, B.x) - m_startX) / m_cellSize)));
        int ec1 = std::min(cMax, static_cast<int>(std::ceil ((std::max(A.x, B.x) - m_startX) / m_cellSize)));
        int er0 = std::max(rMin, static_cast<int>(std::floor((std::min(A.y, B.y) - m_startY) / m_cellSize)));
        int er1 = std::min(rMax, static_cast<int>(std::ceil ((std::max(A.y, B.y) - m_startY) / m_cellSize)));

        for (int row = er0; row <= er1; ++row) {
            for (int col = ec0; col <= ec1; ++col) {
                AABB cell = {
                    { m_startX + col * m_cellSize,       m_startY + row * m_cellSize },
                    { m_startX + (col + 1) * m_cellSize, m_startY + (row + 1) * m_cellSize }
                };
                if (SegmentIntersectsAABB(A, B, cell)) {
                    color[localIdx(col, row)] = 1; // Gray
                }
            }
        }
    }

    // Seed all border cells that are not Gray
    std::vector<std::pair<int,int>> queue;
    queue.reserve(2 * (regionW + regionH));

    auto tryEnqueue = [&](int col, int row) {
        int li = localIdx(col, row);
        if (color[li] == 0) {
            color[li] = 2; // White
            queue.push_back({col, row});
        }
    };

    for (int col = cMin; col <= cMax; ++col) {
        tryEnqueue(col, rMin);
        tryEnqueue(col, rMax);
    }
    for (int row = rMin + 1; row < rMax; ++row) {
        tryEnqueue(cMin, row);
        tryEnqueue(cMax, row);
    }

    // 4-connected BFS
    const int dc[] = { 1, -1, 0,  0 };
    const int dr[] = { 0,  0, 1, -1 };

    for (size_t qi = 0; qi < queue.size(); ++qi) {
        auto [col, row] = queue[qi];
        for (int d = 0; d < 4; ++d) {
            int nc = col + dc[d];
            int nr = row + dr[d];
            if (nc < cMin || nc > cMax || nr < rMin || nr > rMax) continue;
            int li = localIdx(nc, nr);
            if (color[li] == 0) {
                color[li] = 2; // White
                queue.push_back({nc, nr});
            }
        }
    }


    for (int row = rMin; row <= rMax; ++row) {
        for (int col = cMin; col <= cMax; ++col) {
            uint8_t c = color[localIdx(col, row)];
            if (c == 2) continue; // exterior: nothing to mark

            CellEntry& entry = EnsureCellEntry(col, row);
            if (obj.IsDynamic()) {
                entry.state.dynamicOccupied = true;
                if (c == 0) entry.state.dynamicInner = true;
                if (c == 1) entry.state.dynamicGray = true;
            } else {
                entry.state.staticOccupied = true;
                if (c == 0) entry.state.staticInner = true;
                if (c == 1) entry.state.staticGray = true;
            }
        }
    }
}

// --- Overlap tests ------------------------------------------------------

// Circle vs AABB overlap (closest-point check)
bool Grid::CellOverlapsCircle(const AABB& cell, Vec2 center, float radius)
{
    float cx = std::clamp(center.x, cell.min.x, cell.max.x);
    float cy = std::clamp(center.y, cell.min.y, cell.max.y);
    float dx = center.x - cx;
    float dy = center.y - cy;
    return (dx * dx + dy * dy) <= (radius * radius);
}

// Polygon vs AABB overlap
bool Grid::CellOverlapsPolygon(const AABB& cell, const std::vector<Vec2>& verts)
{
    int n = static_cast<int>(verts.size());
    if (n < 3) return false;

    // Vertex inside cell?
    for (const auto& v : verts) {
        if (v.x >= cell.min.x && v.x <= cell.max.x &&
            v.y >= cell.min.y && v.y <= cell.max.y)
            return true;
    }

    // Cell corner inside polygon?
    Vec2 corners[4] = {
        cell.min,
        { cell.max.x, cell.min.y },
        cell.max,
        { cell.min.x, cell.max.y }
    };
    for (const auto& c : corners) {
        if (PointInPolygon(c, verts))
            return true;
    }

    // Edge-edge intersection?
    for (int i = 0; i < n; ++i) {
        const Vec2& a = verts[i];
        const Vec2& b = verts[(i + 1) % n];
        if (SegmentIntersectsAABB(a, b, cell))
            return true;
    }

    return false;
}

// Segment vs AABB intersection (Liang-Barsky)
bool Grid::SegmentIntersectsAABB(Vec2 a, Vec2 b, const AABB& box)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;

    float p[4] = { -dx,  dx, -dy,  dy };
    float q[4] = { a.x - box.min.x, box.max.x - a.x,
                   a.y - box.min.y, box.max.y - a.y };

    float tMin = 0.0f;
    float tMax = 1.0f;

    for (int i = 0; i < 4; ++i) {
        if (std::abs(p[i]) < 1e-8f) {
            if (q[i] < 0.0f) return false;
        } else {
            float t = q[i] / p[i];
            if (p[i] < 0.0f) { if (t > tMin) tMin = t; }
            else              { if (t < tMax) tMax = t; }
            if (tMin > tMax) return false;
        }
    }

    return true;
}

// True if all 4 cell corners are inside the circle
bool Grid::CellInsideCircle(const AABB& cell, Vec2 center, float radius)
{
    float r2 = radius * radius;
    Vec2 corners[4] = {
        cell.min,
        { cell.max.x, cell.min.y },
        cell.max,
        { cell.min.x, cell.max.y }
    };
    for (const auto& c : corners) {
        float dx = c.x - center.x;
        float dy = c.y - center.y;
        if (dx * dx + dy * dy > r2)
            return false;
    }
    return true;
}

// True if all 4 cell corners are inside the polygon
bool Grid::CellInsidePolygon(const AABB& cell, const std::vector<Vec2>& verts)
{
    Vec2 corners[4] = {
        cell.min,
        { cell.max.x, cell.min.y },
        cell.max,
        { cell.min.x, cell.max.y }
    };
    for (const auto& c : corners) {
        if (!PointInPolygon(c, verts))
            return false;
    }
    return true;
}

// Point-in-polygon (ray casting)
bool Grid::PointInPolygon(Vec2 p, const std::vector<Vec2>& verts)
{
    int n = static_cast<int>(verts.size());
    if (n < 3) return false;

    int crossings = 0;
    for (int i = 0; i < n; ++i) {
        const Vec2& a = verts[i];
        const Vec2& b = verts[(i + 1) % n];

        if ((a.y <= p.y && b.y > p.y) || (b.y <= p.y && a.y > p.y)) {
            float t = (p.y - a.y) / (b.y - a.y);
            if (p.x < a.x + t * (b.x - a.x))
                ++crossings;
        }
    }

    return (crossings & 1) != 0;
}
