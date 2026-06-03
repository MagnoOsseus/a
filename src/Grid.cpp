#include "Grid.h"
#include "Object.h"

#include <SDL2/SDL2_gfxPrimitives.h>
#include <cmath>
#include <algorithm>
#include <cassert>

// --- Helpers ----------------------------------------------------------------

namespace {

// True if outer AABB fully contains inner AABB
inline bool ContainsAABB(const AABB& outer, const AABB& inner)
{
    return outer.min.x <= inner.min.x && outer.min.y <= inner.min.y
        && outer.max.x >= inner.max.x && outer.max.y >= inner.max.y;
}

} // namespace

// --- Public -----------------------------------------------------------------

void Grid::Build(float screenW, float screenH, float minCellSize,
                 const std::vector<Object>& objects, Vec2 origin,
                 float angleDegrees)
{
    m_width       = screenW;
    m_height      = screenH;
    m_minCellSize = minCellSize;
    m_originX     = origin.x;
    m_originY     = origin.y;
    m_hasCSpace   = false;
    m_cspaceTree.Reset();

    // Snap root to the grid aligned with the origin (same logic as before)
    float rx = std::fmod(origin.x, minCellSize);
    if (rx < 0.0f) rx += minCellSize;
    float ry = std::fmod(origin.y, minCellSize);
    if (ry < 0.0f) ry += minCellSize;
    m_startX = rx - minCellSize;
    m_startY = ry - minCellSize;

    // Initialise root node covering the screen area
    m_occupancyTree.Init({ { m_startX, m_startY }, { screenW, screenH } });

    // Rasterize every object into the quadtree using polygon-style tests.
    for (const auto& obj : objects) {
        std::vector<Vec2> worldVerts;
        if (obj.IsDynamic()) {
            Object rotated = obj.GetRotated(angleDegrees);
            worldVerts = rotated.GetWorldVertices();
            RasterizeObject(m_occupancyTree.GetRoot(), rotated, worldVerts);
        } else {
            worldVerts = obj.GetWorldVertices();
            RasterizeObject(m_occupancyTree.GetRoot(), obj, worldVerts);
        }
    }
}

void Grid::Clear()
{
    m_occupancyTree.Reset();
    m_cspaceTree.Reset();
    m_hasCSpace = false;
}

// --- Draw -------------------------------------------------------------------

void Grid::Draw(SDL_Renderer* renderer) const
{
    if (!m_occupancyTree.HasRoot()) return;

    OccTree::ForEachLeaf(m_occupancyTree.GetRoot(),
        [renderer](const OccTree::Node* node)
        {
            const OccupancyData& d = node->data;
            bool sOcc = d.staticOccupied;
            bool dOcc = d.dynamicOccupied;
            if (!sOcc && !dOcc) return; // empty leaf – no drawing

            bool sGray = d.staticGray;
            bool dGray = d.dynamicGray;

            Sint16 x1 = static_cast<Sint16>(node->bounds.min.x);
            Sint16 y1 = static_cast<Sint16>(node->bounds.min.y);
            Sint16 x2 = static_cast<Sint16>(node->bounds.max.x);
            Sint16 y2 = static_cast<Sint16>(node->bounds.max.y);

            if (sOcc && dOcc) {
                if (sGray || dGray) {
                    boxRGBA(renderer, x1, y1, x2, y2, 255, 140, 0, 80);
                    rectangleRGBA(renderer, x1, y1, x2, y2, 255, 140, 0, 220);
                } else {
                    boxRGBA(renderer, x1, y1, x2, y2, 255, 140, 0, 120);
                    rectangleRGBA(renderer, x1, y1, x2, y2, 255, 140, 0, 220);
                }
            } else if (sOcc) {
                if (sGray) {
                    boxRGBA(renderer, x1, y1, x2, y2, 255, 255, 255, 30);
                    rectangleRGBA(renderer, x1, y1, x2, y2, 0, 200, 0, 200);
                } else {
                    boxRGBA(renderer, x1, y1, x2, y2, 0, 255, 0, 60);
                    rectangleRGBA(renderer, x1, y1, x2, y2, 0, 255, 0, 120);
                }
            } else { // dOcc
                if (dGray) {
                    boxRGBA(renderer, x1, y1, x2, y2, 255, 255, 255, 30);
                    rectangleRGBA(renderer, x1, y1, x2, y2, 0, 200, 200, 200);
                } else {
                    boxRGBA(renderer, x1, y1, x2, y2, 0, 255, 255, 60);
                    rectangleRGBA(renderer, x1, y1, x2, y2, 0, 255, 255, 120);
                }
            }
        });
}

// --- Configuration Space ----------------------------------------------------

void Grid::ComputeCSpace()
{
    if (!m_occupancyTree.HasRoot()) return;

    // Collect occupied/inner leaf AABBs from the occupancy tree
    std::vector<AABB> dynOcc, dynInn, statOcc, statInn;
    OccTree::ForEachLeaf(m_occupancyTree.GetRoot(),
        [&](const OccTree::Node* leaf)
        {
            const OccupancyData& d = leaf->data;
            if (d.dynamicOccupied) dynOcc.push_back(leaf->bounds);
            if (d.dynamicInner)    dynInn.push_back(leaf->bounds);
            if (d.staticOccupied)  statOcc.push_back(leaf->bounds);
            if (d.staticInner)     statInn.push_back(leaf->bounds);
        });

    if (dynOcc.empty()) return;

    // Reference point: area-weighted centroid of dynamic occupied leaves
    float totalArea = 0.0f;
    float refX = 0.0f, refY = 0.0f;
    for (const auto& b : dynOcc) {
        float area = b.Width() * b.Height();
        float cx   = (b.min.x + b.max.x) * 0.5f;
        float cy   = (b.min.y + b.max.y) * 0.5f;
        refX += cx * area;
        refY += cy * area;
        totalArea += area;
    }
    if (totalArea > 0.0f) {
        refX /= totalArea;
        refY /= totalArea;
    }
    m_refScreenPos = { refX, refY };

    const auto sameCellSize = [](const AABB& a, const AABB& b) {
        // Quadtree splits by halves, so leaf sizes should match exactly.
        // 1e-4 keeps tolerance far below one pixel while absorbing small
        // float rounding from repeated subdivision/transform steps.
        constexpr float kRelativeSizeEpsilon = 1e-4f;
        const float aw = a.Width();
        const float ah = a.Height();
        const float bw = b.Width();
        const float bh = b.Height();
        const float aScale = std::max(aw, ah);
        const float bScale = std::max(bw, bh);
        const float scale = std::max(1.0f, std::max(aScale, bScale));
        const float eps = kRelativeSizeEpsilon * scale;
        return std::abs(aw - bw) <= eps && std::abs(ah - bh) <= eps;
    };

    // Precompute Minkowski obstacle AABBs, comparing only equal-sized cells.
    std::vector<AABB> minkConservative; // dynOcc × statOcc (same size only)
    minkConservative.reserve(dynOcc.size() * statOcc.size());
    for (const auto& d : dynOcc) {
        for (const auto& s : statOcc) {
            if (!sameCellSize(d, s)) continue;
            AABB mk = {
                { s.min.x - d.max.x + refX, s.min.y - d.max.y + refY },
                { s.max.x - d.min.x + refX, s.max.y - d.min.y + refY }
            };
            if (mk.min.x < mk.max.x && mk.min.y < mk.max.y)
                minkConservative.push_back(mk);
        }
    }

    // Definite collision: dynInn × statInn (same size only)
    std::vector<AABB> minkDefinite;
    minkDefinite.reserve(dynInn.size() * statInn.size());
    for (const auto& d : dynInn) {
        for (const auto& s : statInn) {
            if (!sameCellSize(d, s)) continue;
            AABB mk = {
                { s.min.x - d.max.x + refX, s.min.y - d.max.y + refY },
                { s.max.x - d.min.x + refX, s.max.y - d.min.y + refY }
            };
            if (mk.min.x < mk.max.x && mk.min.y < mk.max.y)
                minkDefinite.push_back(mk);
        }
    }

    // Build CSpace tree with the same root bounds as the occupancy tree
    m_cspaceTree.Init(m_occupancyTree.GetRoot()->bounds);

    if (statOcc.empty()) {
        // No obstacles: the entire C-Space is safe
        m_cspaceTree.GetRoot()->data.safe = true;
    } else {
        BuildCSpaceNode(m_cspaceTree.GetRoot(), minkConservative, minkDefinite);
    }

    m_hasCSpace = true;
}

void Grid::BuildCSpaceNode(CSTree::Node* node,
                           const std::vector<AABB>& minkConservative,
                           const std::vector<AABB>& minkDefinite)
{
    const AABB& b = node->bounds;

    // Are any conservative obstacles relevant for this node?
    bool anyIntersect = false;
    for (const auto& mk : minkConservative) {
        if (b.Intersects(mk)) { anyIntersect = true; break; }
    }
    if (!anyIntersect) {
        node->data.safe = true;
        return;
    }

    // Is this node entirely inside a definite obstacle?
    for (const auto& mk : minkDefinite) {
        if (ContainsAABB(mk, b)) {
            node->data.unsafe = true;
            return;
        }
    }

    // Can we subdivide?
    float halfW = b.Width()  * 0.5f;
    float halfH = b.Height() * 0.5f;
    if (halfW >= m_minCellSize && halfH >= m_minCellSize) {
        CSTree::Subdivide(node); // children start uncertain (default CSpaceData)
        for (auto& child : node->children)
            BuildCSpaceNode(child.get(), minkConservative, minkDefinite);
        return;
    }

    // Minimum size reached: leave as uncertain (neither flag set)
}

void Grid::DrawCSpace(SDL_Renderer* renderer) const
{
    if (!m_cspaceTree.HasRoot()) return;

    CSTree::ForEachLeaf(m_cspaceTree.GetRoot(),
        [renderer](const CSTree::Node* node)
        {
            Sint16 x1 = static_cast<Sint16>(node->bounds.min.x);
            Sint16 y1 = static_cast<Sint16>(node->bounds.min.y);
            Sint16 x2 = static_cast<Sint16>(node->bounds.max.x);
            Sint16 y2 = static_cast<Sint16>(node->bounds.max.y);

            if (node->data.safe) {
                boxRGBA(renderer,       x1, y1, x2, y2,   0, 200,   0, 160);
                rectangleRGBA(renderer, x1, y1, x2, y2,   0, 255,   0, 200);
            } else if (node->data.unsafe) {
                boxRGBA(renderer,       x1, y1, x2, y2, 255,  60,  60, 160);
                rectangleRGBA(renderer, x1, y1, x2, y2, 255, 100, 100, 200);
            } else {
                // Uncertain: faint outline only
                rectangleRGBA(renderer, x1, y1, x2, y2, 50, 50, 50, 40);
            }
        });
}

CSpaceStatus Grid::QueryCSpace(Vec2 t) const
{
    if (!m_cspaceTree.HasRoot()) return CSpaceStatus::Uncertain;
    const CSTree::Node* leaf = CSTree::QueryPoint(m_cspaceTree.GetRoot(), t);
    if (!leaf) return CSpaceStatus::Uncertain;
    if (leaf->data.safe)   return CSpaceStatus::Safe;
    if (leaf->data.unsafe) return CSpaceStatus::Unsafe;
    return CSpaceStatus::Uncertain;
}

// Draw a crosshair at the grid origin
void Grid::DrawOrigin(SDL_Renderer* renderer) const
{
    Sint16 ox  = static_cast<Sint16>(m_originX);
    Sint16 oy  = static_cast<Sint16>(m_originY);
    Sint16 len = static_cast<Sint16>(m_minCellSize * 0.4f);
    thickLineRGBA(renderer, ox - len, oy,       ox + len, oy,       2, 255, 255, 0, 255);
    thickLineRGBA(renderer, ox,       oy - len, ox,       oy + len, 2, 255, 255, 0, 255);
    filledCircleRGBA(renderer, ox, oy, 3, 255, 255, 0, 255);
}

float Grid::GetMaxDynamicInnerDistance(Vec2 origin) const
{
    if (!m_occupancyTree.HasRoot()) return 0.0f;
    float maxDist = 0.0f;
    OccTree::ForEachLeaf(m_occupancyTree.GetRoot(),
        [&](const OccTree::Node* node)
        {
            if (!node->data.dynamicInner) return;
            // Check all four corners of this leaf
            Vec2 corners[4] = {
                node->bounds.min,
                { node->bounds.max.x, node->bounds.min.y },
                node->bounds.max,
                { node->bounds.min.x, node->bounds.max.y }
            };
            for (const auto& c : corners) {
                float dx = c.x - origin.x;
                float dy = c.y - origin.y;
                float d  = std::sqrt(dx * dx + dy * dy);
                if (d > maxDist) maxDist = d;
            }
        });
    return maxDist;
}

// --- Rasterization ----------------------------------------------------------

void Grid::RasterizeObject(OccTree::Node* node, const Object& obj,
                           const std::vector<Vec2>& worldVerts)
{
    if (!node) return;
    // Assert for debug builds; keep runtime guard for release safety.
    assert(worldVerts.size() >= 3 && "Rasterization expects polygonized object vertices.");
    if (worldVerts.size() < 3) return;

    // Quick rejection: does the object's AABB touch this node at all?
    if (!node->bounds.Intersects(obj.GetAABB())) return;

    const bool   isDyn    = obj.IsDynamic();

    float halfW = node->bounds.Width()  * 0.5f;
    float halfH = node->bounds.Height() * 0.5f;
    bool canSubdivide = (halfW >= m_minCellSize && halfH >= m_minCellSize);

    if (node->IsLeaf()) {
        bool hit = CellOverlapsPolygon(node->bounds, worldVerts);
        if (!hit) return;

        bool inside = CellInsidePolygon(node->bounds, worldVerts);

        if (inside) {
            // Leaf is fully inside the object: mark as inner and stop
            if (isDyn) { node->data.dynamicOccupied = true; node->data.dynamicInner = true; }
            else        { node->data.staticOccupied  = true; node->data.staticInner  = true; }
            return;
        }

        if (!canSubdivide) {
            // At minimum cell size: boundary cell
            if (isDyn) { node->data.dynamicOccupied = true; node->data.dynamicGray = true; }
            else        { node->data.staticOccupied  = true; node->data.staticGray  = true; }
            return;
        }

        // Partial overlap: subdivide, children inherit current occupancy state,
        // then clear parent (it becomes an internal node)
        OccTree::Subdivide(node,
            [](OccTree::Node* parent, OccTree::Node* child, int) {
                child->data = parent->data;
            });
        node->data = {};
    }

    // Recurse into children (node is now internal)
    for (auto& child : node->children)
        RasterizeObject(child.get(), obj, worldVerts);
}

// --- Overlap tests ----------------------------------------------------------

bool Grid::CellOverlapsCircle(const AABB& cell, Vec2 center, float radius)
{
    float cx = std::clamp(center.x, cell.min.x, cell.max.x);
    float cy = std::clamp(center.y, cell.min.y, cell.max.y);
    float dx = center.x - cx;
    float dy = center.y - cy;
    return (dx * dx + dy * dy) <= (radius * radius);
}

bool Grid::CellOverlapsPolygon(const AABB& cell, const std::vector<Vec2>& verts)
{
    int n = static_cast<int>(verts.size());
    if (n < 3) return false;

    for (const auto& v : verts) {
        if (v.x >= cell.min.x && v.x <= cell.max.x &&
            v.y >= cell.min.y && v.y <= cell.max.y)
            return true;
    }

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

    for (int i = 0; i < n; ++i) {
        const Vec2& a = verts[i];
        const Vec2& b = verts[(i + 1) % n];
        if (SegmentIntersectsAABB(a, b, cell))
            return true;
    }

    return false;
}

bool Grid::SegmentIntersectsAABB(Vec2 a, Vec2 b, const AABB& box)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;

    float p[4] = { -dx, dx, -dy, dy };
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
