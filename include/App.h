#pragma once

#include "Object.h"
#include "Grid.h"
#include "CSpaceRefiner.h"

#include <SDL2/SDL.h>
#include <vector>
#include <map>

// Main application class. Owns the window, renderer, objects and grid.
// Runs the main loop: events -> update -> render.
class App
{
public:
    App();
    ~App();

    App(const App&) = delete;
    App& operator=(const App&) = delete;

    bool Init();   // Create window, renderer and ImGui context
    void Run();    // Main loop

private:
    void ProcessEvents();  // Poll SDL events and forward to ImGui
    void Update();         // Move objects and rebuild the grid
    void Render();         // Draw grid, objects and ImGui
    void DrawUI();         // ImGui panel for creating/managing objects

    // --- Core ---
    SDL_Window*          m_window   = nullptr;
    SDL_Renderer*        m_renderer = nullptr;
    bool                 m_running  = false;
    int                  m_windowW  = 1280;           // current window width
    int                  m_windowH  = 720;            // current window height
    std::vector<Object>  m_objects;

    // --- Multi-layer C-Space (one grid + refiner per angle) ---
    std::map<int, Grid>          m_layers;        // angle in degrees -> Grid
    std::map<int, CSpaceRefiner> m_layerRefiners; // angle in degrees -> CSpaceRefiner
    int                          m_currentAngle = 0;   // current layer angle being viewed
    int                          m_currentLayerIndex = 0; // index into m_angleSteps

    // Angular step computed from cell size and dynamic object geometry
    float             m_angleDeg      = 0.0f;   // computed step in degrees
    std::vector<int>  m_angleSteps;              // list of angles to compute [0, step, 2*step, ...]

    float                m_cellSize      = 25.0f;              // grid cell size in pixels
    Vec2                 m_gridOrigin    = {0.0f, 0.0f};       // grid origin (center of first dynamic object)
    bool                 m_gridOriginSet = false;              // true once a dynamic object has been created

    // --- ImGui widget state ---
    int   m_uiShapeType    = 0;                    // selected shape in the combo box
    float m_uiPosition[2]  = {200.0f, 200.0f};     // position for the next object (panel-relative)
    float m_uiSize         = 80.0f;                // size or radius
    int   m_uiPolySides    = 5;                    // sides for regular polygon
    bool  m_uiDynamic      = false;                // mark object as dynamic (for C-Space)
    int   m_refinementLevels = 0;                  // refinement depth (0 = off)

    // --- Freeform polygon state ---
    std::vector<Vec2> m_bezierControlPoints;
    bool              m_placingBezierPoints = false;
    bool              m_bezierCloseCurve    = false;

    // --- Layout ---
    float m_toolbarH     = 0.0f;    // height of the top toolbar (set each frame)
    bool  m_computing    = false;   // true after pressing Start (triggers C-Space)

    // --- C-Space preview ---
    bool  m_previewTranslation = false;   // true when showing translation preview
    Vec2  m_previewOffset      = {0.0f, 0.0f};  // translation offset from clicked C-Space point
};
