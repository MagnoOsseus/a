#pragma once

#include "AABB.h"
#include "IDrawable.h"
#include "third_party/quadtree/Box.h"
#include "third_party/quadtree/Quadtree.h"

#include <vector>
#include <cstdint>
#include <memory>
#include <unordered_map>

class Object;

// Uniform grid that covers the screen. Rasterizes objects onto cells
class Grid : public IDrawable
{
public:
    Grid() = default;

    // Rebuild the grid for this frame (rasterize objects, no C-Space)
    // Objects marked as dynamic will be rotated by angleDegrees before rasterization
    void Build(float screenW, float screenH, float cellSize, const std::vector<Object>& objects, Vec2 origin, float angleDegrees = 0.0f);

    // Compute configuration space (call separately after Build)
    void ComputeCSpace();

    // Reset everything
    void Clear();

    // Draw every cell colored by occupancy (static / dynamic)
    void Draw(SDL_Renderer* renderer) const override;

    // Draw the C-Space: green dots=safe
    void DrawCSpace(SDL_Renderer* renderer) const;

    // Draw a marker at the grid origin (0,0)
    void DrawOrigin(SDL_Renderer* renderer) const;

    int   GetCols()     const { return m_cols; }
    int   GetRows()     const { return m_rows; }
    float GetCellSize() const { return m_cellSize; }
    float GetStartX()   const { return m_startX; }
    float GetStartY()   const { return m_startY; }
    float GetScreenW()  const { return m_width; }
    float GetScreenH()  const { return m_height; }
    Vec2  GetRefScreenPos()            const { return m_refScreenPos; }
    const std::vector<bool>& GetCSpaceSafe()   const { return m_cspaceSafe; }
    const std::vector<bool>& GetCSpaceUnsafe() const { return m_cspaceUnsafe; }


    float GetMaxDynamicInnerDistance(Vec2 origin) const;

    // --- Cell-vs-shape tests (also used by Quadtree for refinement) ---
    static bool CellOverlapsCircle(const AABB& cell, Vec2 center, float radius);
    static bool CellOverlapsPolygon(const AABB& cell, const std::vector<Vec2>& verts);
    static bool CellInsideCircle(const AABB& cell, Vec2 center, float radius);
    static bool CellInsidePolygon(const AABB& cell, const std::vector<Vec2>& verts);

private:
    struct CellState
    {
        bool staticOccupied = false;
        bool dynamicOccupied = false;
        bool staticInner = false;
        bool dynamicInner = false;
        bool staticGray = false;
        bool dynamicGray = false;
    };

    struct CellEntry
    {
        quadtree::Box<float> box;
        int col = 0;
        int row = 0;
        CellState state;
    };

    struct CellEntryBoxGetter
    {
        quadtree::Box<float> operator()(const CellEntry* entry) const
        {
            return entry->box;
        }
    };

    using OccupancyTree = quadtree::Quadtree<CellEntry*, CellEntryBoxGetter>;

    // Daum 2012: Gray/flood-fill rasterization for arbitrary polygons.
    // Fills occupied/inner arrays directly for the given object.
    void RasterizePolygonDaum(const Object& obj, const std::vector<Vec2>& worldVerts);
    void ResetSpatialIndex();
    CellEntry& EnsureCellEntry(int col, int row);
    CellState QueryCellState(int col, int row) const;
    void SyncOccupancyArrays();
    float m_cellSize = 20.0f;
    int   m_cols     = 0;
    int   m_rows     = 0;
    float m_width    = 0.0f;
    float m_height   = 0.0f;
    float m_startX   = 0.0f;   // top-left corner of the grid (can be negative)
    float m_startY   = 0.0f;
    float m_originX  = 0.0f;   // grid origin position (in pixels)
    float m_originY  = 0.0f;

    // Per-frame occupancy: cells that OVERLAP the shape (conservative)
    std::vector<bool> m_staticOccupied;
    std::vector<bool> m_dynamicOccupied;

    // Per-frame occupancy: cells COMPLETELY INSIDE the shape (strict = Black in Daum)
    std::vector<bool> m_staticInner;
    std::vector<bool> m_dynamicInner;

    // Per-frame occupancy: cells intersected by the boundary (Gray in Daum)
    std::vector<bool> m_staticGray;
    std::vector<bool> m_dynamicGray;

    // C-Space results
    std::vector<bool> m_cspaceSafe;     // true = definitely safe
    std::vector<bool> m_cspaceUnsafe;   // true = definitely colliding
    Vec2 m_refScreenPos = {};            // screen pos of the C-Space reference point

    std::vector<std::unique_ptr<CellEntry>> m_cellEntries;
    std::unordered_map<size_t, CellEntry*> m_cellLookup;
    std::unique_ptr<OccupancyTree> m_occupancyTree;

    // Rasterize a single object onto the grid (marks which cells it covers)
    void RasterizeObject(const Object& obj);

    // --- Internal geometry helpers ---
    static bool SegmentIntersectsAABB(Vec2 a, Vec2 b, const AABB& box);
    static bool PointInPolygon(Vec2 p, const std::vector<Vec2>& verts);
};
