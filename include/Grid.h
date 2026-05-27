#pragma once

#include "AABB.h"
#include "IDrawable.h"

#include <memory>
#include <vector>
#include <cstdint>

class Object;

// Result of a C-Space point query
enum class CSpaceStatus { Safe, Unsafe, Uncertain };

// Pure quadtree decomposition of the screen.
// Space is subdivided adaptively: only regions near obstacle boundaries
// are refined down to minCellSize; empty / fully-interior regions keep
// their larger parent cell.  No flat grid arrays are used.
class Grid : public IDrawable
{
public:
    Grid() = default;

    // Build (rasterize objects into the quadtree, no C-Space yet).
    // Dynamic objects are rotated by angleDegrees before rasterization.
    void Build(float screenW, float screenH, float minCellSize,
               const std::vector<Object>& objects, Vec2 origin,
               float angleDegrees = 0.0f);

    // Compute configuration space (call separately after Build).
    void ComputeCSpace();

    // Reset everything.
    void Clear();

    // Draw every occupied leaf node coloured by occupancy (static / dynamic).
    void Draw(SDL_Renderer* renderer) const override;

    // Draw the C-Space quadtree leaves (green = safe, red = collision).
    void DrawCSpace(SDL_Renderer* renderer) const;

    // Draw a marker at the grid origin.
    void DrawOrigin(SDL_Renderer* renderer) const;

    // Minimum leaf cell size (= minCellSize passed to Build).
    float GetCellSize()  const { return m_minCellSize; }
    float GetStartX()    const { return m_startX; }
    float GetStartY()    const { return m_startY; }
    float GetScreenW()   const { return m_width; }
    float GetScreenH()   const { return m_height; }
    Vec2  GetRefScreenPos() const { return m_refScreenPos; }
    // True once ComputeCSpace() has been called and the tree is built.
    bool  HasCSpace()    const { return m_hasCSpace; }

    // Maximum distance from origin to the farthest dynamic-inner leaf centre.
    float GetMaxDynamicInnerDistance(Vec2 origin) const;

    // Query the C-Space status at a translation point t.
    CSpaceStatus QueryCSpace(Vec2 t) const;

    // --- Cell-vs-shape tests (also used by CSpaceRefiner) ---
    static bool CellOverlapsCircle(const AABB& cell, Vec2 center, float radius);
    static bool CellOverlapsPolygon(const AABB& cell, const std::vector<Vec2>& verts);
    static bool CellInsideCircle(const AABB& cell, Vec2 center, float radius);
    static bool CellInsidePolygon(const AABB& cell, const std::vector<Vec2>& verts);

private:
    // -----------------------------------------------------------------------
    // Quadtree node.  Only leaf nodes carry occupancy / C-Space state.
    // Internal nodes exist solely to partition space; their state flags are
    // always false and should not be read.
    // -----------------------------------------------------------------------
    struct QTNode
    {
        AABB bounds;

        // Occupancy (leaf only)
        bool staticOccupied  = false;
        bool staticInner     = false;
        bool staticGray      = false;
        bool dynamicOccupied = false;
        bool dynamicInner    = false;
        bool dynamicGray     = false;

        // C-Space (leaf only, set by BuildCSpaceNode)
        bool cspaceSafe      = false;
        bool cspaceUnsafe    = false;

        // Children: null ↔ this node is a leaf
        std::unique_ptr<QTNode> children[4]; // NW NE SW SE
        bool IsLeaf() const { return !static_cast<bool>(children[0]); }
    };

    // Occupancy quadtree (built by Build / RasterizeObject)
    std::unique_ptr<QTNode> m_root;
    // C-Space quadtree (built by ComputeCSpace)
    std::unique_ptr<QTNode> m_cspaceRoot;
    bool  m_hasCSpace   = false;

    float m_minCellSize = 20.0f;
    float m_width       = 0.0f;
    float m_height      = 0.0f;
    float m_startX      = 0.0f;   // top-left of the root node (can be negative)
    float m_startY      = 0.0f;
    float m_originX     = 0.0f;   // grid origin in pixels (= dynamic object position)
    float m_originY     = 0.0f;
    Vec2  m_refScreenPos = {};    // C-Space reference point (centroid of dynamic object)

    // --- Quadtree utilities ------------------------------------------------
    // Split node into 4 children.  Each child inherits the parent's occupancy
    // state so that subsequent rasterizations of other objects remain correct.
    // The parent's occupancy flags are cleared (it is now internal).
    static void Subdivide(QTNode* node);
    // Return the AABB of quadrant q (0=NW 1=NE 2=SW 3=SE) of parent.
    static AABB ChildBounds(const AABB& parent, int q);

    // --- Rasterization -----------------------------------------------------
    // Recursively mark the quadtree to reflect one object's footprint.
    void RasterizeObject(QTNode* node, const Object& obj,
                         const std::vector<Vec2>& worldVerts);

    // --- C-Space computation -----------------------------------------------
    // Collect occupied/inner leaf AABBs from the occupancy tree.
    static void CollectLeaves(const QTNode* node,
                              std::vector<AABB>& dynOcc,
                              std::vector<AABB>& dynInn,
                              std::vector<AABB>& statOcc,
                              std::vector<AABB>& statInn);
    // Recursively classify C-Space tree nodes using precomputed Minkowski
    // obstacles (minkConservative = safe-test obstacles,
    //           minkDefinite     = definite-collision obstacles).
    void BuildCSpaceNode(QTNode* node,
                         const std::vector<AABB>& minkConservative,
                         const std::vector<AABB>& minkDefinite);

    // --- Drawing helpers ---------------------------------------------------
    static void TraverseDraw(const QTNode* node, SDL_Renderer* renderer);
    static void TraverseDrawCSpace(const QTNode* node, SDL_Renderer* renderer);

    // --- C-Space query helper ----------------------------------------------
    static CSpaceStatus QueryNode(const QTNode* node, Vec2 t);

    // --- Max-distance helper -----------------------------------------------
    static void TraverseMaxDist(const QTNode* node, Vec2 origin, float& maxDist);

    // --- Internal geometry helpers -----------------------------------------
    static bool SegmentIntersectsAABB(Vec2 a, Vec2 b, const AABB& box);
    static bool PointInPolygon(Vec2 p, const std::vector<Vec2>& verts);
};
