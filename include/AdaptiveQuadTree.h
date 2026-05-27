#pragma once

#include "AABB.h"

#include <cassert>
#include <memory>
#include <utility>

// Generic adaptive region quadtree.
//
// Provides tree *infrastructure* only:
//   - node allocation and ownership (via std::unique_ptr)
//   - subdivision with child-bounds computation (ChildBounds / Subdivide)
//   - depth-first leaf traversal (ForEachLeaf)
//   - deepest-leaf point query (QueryPoint)
//
// All domain logic — when to subdivide, what the per-node data means, geometry
// tests, classification — stays in client code.
//
// NodeData  User-defined type stored in every node.  Only leaf-node data is
//           meaningful; internal nodes carry a default-constructed NodeData.

template<typename NodeData>
class AdaptiveQuadTree
{
public:
    // -----------------------------------------------------------------------
    // Node.  A node is a leaf iff children[0] is null.
    // -----------------------------------------------------------------------
    struct Node
    {
        AABB     bounds;
        NodeData data     = {};

        std::unique_ptr<Node> children[4]; // 0=NW  1=NE  2=SW  3=SE

        bool IsLeaf() const { return !static_cast<bool>(children[0]); }
    };

    // --- Lifecycle ----------------------------------------------------------

    // Initialise (or re-initialise) the tree with a single root leaf.
    void Init(const AABB& rootBounds)
    {
        m_root         = std::make_unique<Node>();
        m_root->bounds = rootBounds;
    }

    // Destroy all nodes.
    void Reset() { m_root.reset(); }

    bool        HasRoot() const { return m_root != nullptr; }
    Node*       GetRoot()       { return m_root.get(); }
    const Node* GetRoot() const { return m_root.get(); }

    // --- Static tree utilities ----------------------------------------------

    // Return the AABB for quadrant q (0=NW 1=NE 2=SW 3=SE) of a parent region.
    static AABB ChildBounds(const AABB& parent, int q)
    {
        const float midX = (parent.min.x + parent.max.x) * 0.5f;
        const float midY = (parent.min.y + parent.max.y) * 0.5f;
        switch (q)
        {
            case 0:  return { parent.min,             { midX, midY }         }; // NW
            case 1:  return { { midX, parent.min.y }, { parent.max.x, midY } }; // NE
            case 2:  return { { parent.min.x, midY }, { midX, parent.max.y } }; // SW
            default: return { { midX, midY },          parent.max            }; // SE
        }
    }

    // Split a leaf into 4 children.
    // initChild(parent*, child*, quadrant) is called for each new child,
    // allowing the caller to initialise child data (e.g. inherit from parent).
    template<typename InitFn>
    static void Subdivide(Node* node, InitFn&& initChild)
    {
        assert(node != nullptr && node->IsLeaf());
        for (int q = 0; q < 4; ++q)
        {
            node->children[q]         = std::make_unique<Node>();
            node->children[q]->bounds = ChildBounds(node->bounds, q);
            initChild(node, node->children[q].get(), q);
        }
    }

    // Split a leaf leaving child data default-constructed.
    static void Subdivide(Node* node)
    {
        Subdivide(node, [](Node*, Node*, int) {});
    }

    // --- Traversal ----------------------------------------------------------

    // Call visit(const Node*) for every leaf.  Const overload.
    template<typename Visitor>
    static void ForEachLeaf(const Node* node, Visitor&& visit)
    {
        if (!node) return;
        if (node->IsLeaf()) { visit(node); return; }
        for (const auto& child : node->children)
            ForEachLeaf(child.get(), std::forward<Visitor>(visit));
    }

    // Call visit(Node*) for every leaf.  Mutable overload.
    template<typename Visitor>
    static void ForEachLeaf(Node* node, Visitor&& visit)
    {
        if (!node) return;
        if (node->IsLeaf()) { visit(node); return; }
        for (auto& child : node->children)
            ForEachLeaf(child.get(), std::forward<Visitor>(visit));
    }

    // --- Spatial query ------------------------------------------------------

    // Return the deepest leaf whose bounds contain point p, or nullptr.
    static const Node* QueryPoint(const Node* node, Vec2 p)
    {
        if (!node || !node->bounds.Contains(p)) return nullptr;
        if (node->IsLeaf()) return node;
        for (const auto& child : node->children)
            if (child && child->bounds.Contains(p))
                return QueryPoint(child.get(), p);
        return nullptr;
    }

private:
    std::unique_ptr<Node> m_root;
};
