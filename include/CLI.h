#pragma once

#include "Object.h"
#include "Grid.h"
#include "CSpaceRefiner.h"

#include <vector>
#include <map>

// Command-line interface for the collision detection app.
// Provides all the same features as the GUI but driven from stdin/stdout.
// Launch with: Collisions --cli
class CLI
{
public:
    void Run();

private:
    void CmdAdd();
    void CmdList() const;
    void CmdRemove();
    void CmdSettings();
    void CmdCompute();
    void CmdQuery() const;
    void CmdClear();
    void PrintHelp() const;

    // --- Scene state ---
    std::vector<Object>          m_objects;
    std::map<int, Grid>          m_layers;
    std::map<int, CSpaceRefiner> m_layerRefiners;

    float            m_cellSize         = 25.0f;
    int              m_refinementLevels = 0;
    float            m_angleDeg         = 0.0f;
    std::vector<int> m_angleSteps;
    Vec2             m_gridOrigin       = {0.0f, 0.0f};
    bool             m_gridOriginSet    = false;

    // Virtual workspace size used for grid computation
    float m_workspaceW = 1000.0f;
    float m_workspaceH = 1000.0f;
};
