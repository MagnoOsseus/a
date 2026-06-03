#include "App.h"

#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include <glm/ext/scalar_constants.hpp>
#include <SDL2/SDL2_gfxPrimitives.h>

#include <cmath>
#include <numbers>

App::App()  = default;

// Shut down in reverse order: ImGui first, then SDL
App::~App()
{
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    if (m_renderer) SDL_DestroyRenderer(m_renderer);
    if (m_window)   SDL_DestroyWindow(m_window);
    SDL_Quit();
}

// Set up SDL window + renderer, then initialize ImGui on top of it
bool App::Init()
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        SDL_Log("Error al inicializar SDL: %s", SDL_GetError());
        return false;
    }

    m_window = SDL_CreateWindow(
        "Collisions",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        m_windowW,
        m_windowH,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );

    if (!m_window) {
        SDL_Log("Error al crear la ventana: %s", SDL_GetError());
        return false;
    }

    m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_ACCELERATED);
    if (!m_renderer) {
        SDL_Log("Error al crear el renderer: %s", SDL_GetError());
        return false;
    }

    // --- ImGui ---
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForSDLRenderer(m_window, m_renderer);
    ImGui_ImplSDLRenderer2_Init(m_renderer);

    return true;
}

// Main loop: process input, update simulation, draw everything
void App::Run()
{
    m_running = true;

    while (m_running) {
        ProcessEvents();
        Update();
        Render();
    }
}

// Poll SDL events. Forward each one to ImGui so it can handle mouse/keyboard.
void App::ProcessEvents()
{
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);

        switch (event.type) {
        case SDL_QUIT:
            m_running = false;
            break;

        case SDL_WINDOWEVENT:
            if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                m_windowW = event.window.data1;
                m_windowH = event.window.data2;
            }
            break;

        case SDL_KEYDOWN:
            // F11 toggles fullscreen
            if (event.key.keysym.sym == SDLK_F11) {
                Uint32 flags = SDL_GetWindowFlags(m_window);
                if (flags & SDL_WINDOW_FULLSCREEN_DESKTOP)
                    SDL_SetWindowFullscreen(m_window, 0);
                else
                    SDL_SetWindowFullscreen(m_window, SDL_WINDOW_FULLSCREEN_DESKTOP);
            }
            break;

        case SDL_MOUSEBUTTONDOWN:
            if (event.button.button == SDL_BUTTON_LEFT) {
                ImGuiIO& io = ImGui::GetIO();
                if (!io.WantCaptureMouse) {
                    int halfW = m_windowW / 2;
                    int topY = static_cast<int>(m_toolbarH);
                    float mx = static_cast<float>(event.button.x);
                    float my = static_cast<float>(event.button.y);

                    // Check if we're placing Bézier control points (left panel)
                    bool inLeftPanel = mx < static_cast<float>(halfW)
                                    && my > static_cast<float>(topY)
                                    && my < static_cast<float>(m_windowH);

                    if (m_placingBezierPoints && inLeftPanel) {
                        // Add control point in local coordinates
                        Vec2 localPoint = { mx, my - static_cast<float>(topY) };
                        m_bezierControlPoints.push_back(localPoint);
                    }
                    // Check if click is in C-Space panel (right half)
                    else {
                        bool inCSpace = mx > static_cast<float>(halfW + 1)
                                     && my > static_cast<float>(topY)
                                     && mx < static_cast<float>(m_windowW)
                                     && my < static_cast<float>(m_windowH);

                        if (inCSpace && !m_layers.empty()) {
                            auto gridIt = m_layers.find(m_currentAngle);
                            auto refinerIt = m_layerRefiners.find(m_currentAngle);

                            if (gridIt != m_layers.end()) {
                                const Grid& currentGrid = gridIt->second;
                                const CSpaceRefiner& currentRefiner = (refinerIt != m_layerRefiners.end()) 
                                    ? refinerIt->second 
                                    : m_layerRefiners[m_currentAngle];

                                // Convert mouse position to C-Space coordinates
                                Vec2 mousePos = { mx - static_cast<float>(halfW + 1),
                                                  my - static_cast<float>(topY) };

                                // Query refinement level at this position
                                CSpaceQueryResult result = currentRefiner.Query(mousePos, currentGrid);

                                // Get base grid info
                                float sX = currentGrid.GetStartX();
                                float sY = currentGrid.GetStartY();
                                float cs = currentGrid.GetCellSize();

                                // Adjust cell size based on refinement level
                                for (int l = 1; l < result.level; ++l) cs *= 0.5f;

                                // Snap to nearest vertex at the refined level
                                int clickedCol = static_cast<int>(std::round((mousePos.x - sX) / cs));
                                int clickedRow = static_cast<int>(std::round((mousePos.y - sY) / cs));
                                Vec2 clickedPos = { sX + clickedCol * cs, sY + clickedRow * cs };

                                // Calculate offset directly from grid origin (actual object position)
                                m_previewOffset = { clickedPos.x - m_gridOrigin.x, 
                                                   clickedPos.y - m_gridOrigin.y };
                                m_previewTranslation = true;
                            }
                        } else {
                            m_previewTranslation = false;
                        }
                    }
                }
            }
            break;
        }
    }
}

// Rebuild the grid each frame
void App::Update()
{
    if (!m_objects.empty()) {
        // Lock origin to the center of the first dynamic object (once)
        if (!m_gridOriginSet) {
            for (const auto& obj : m_objects) {
                if (obj.IsDynamic()) {
                    m_gridOrigin    = obj.GetPosition();
                    m_gridOriginSet = true;
                    break;
                }
            }
        }

        float panelW = static_cast<float>(m_windowW / 2);
        float panelH = static_cast<float>(m_windowH) - m_toolbarH;

        // Get or create the grid for the current angle.
        // Only rebuild if C-Space hasn't been computed yet; rebuilding would
        // reset m_hasCSpace and erase the C-Space tree.
        float effCellSize = m_cellSize / std::pow(2.0f, static_cast<float>(m_refinementLevels));
        Grid& currentGrid = m_layers[m_currentAngle];
        if (!currentGrid.HasCSpace()) {
            currentGrid.Build(panelW, panelH, effCellSize, m_objects, m_gridOrigin, static_cast<float>(m_currentAngle));
        }

        // Recompute angular step using the smallest cell size
        float d = currentGrid.GetMaxDynamicInnerDistance(m_gridOrigin);
        if (d > 0.0f) {
            float stepRad = currentGrid.GetCellSize() / (2.0f * d);
            m_angleDeg = stepRad * (180.0f / static_cast<float>(std::numbers::pi));
        }

        if (m_computing) {
            // Build angle list and compute all layers
            m_angleSteps.clear();
            if (m_angleDeg > 0.0f) {
                int numLayers = static_cast<int>(std::ceil(360.0f / m_angleDeg));
                for (int i = 0; i < numLayers; ++i) {
                    int angleDeg = static_cast<int>(std::round(i * m_angleDeg)) % 360;
                    m_angleSteps.push_back(angleDeg);
                }
            } else {
                m_angleSteps.push_back(0);
            }

            // Compute C-Space for every angle
            for (int angle : m_angleSteps) {
                Grid& g = m_layers[angle];
                g.Build(panelW, panelH, effCellSize, m_objects, m_gridOrigin, static_cast<float>(angle));
                g.ComputeCSpace();

                CSpaceRefiner& refiner = m_layerRefiners[angle];
                refiner.Clear();
            }

            // Set current angle to first step
            m_currentLayerIndex = 0;
            m_currentAngle = m_angleSteps.empty() ? 0 : m_angleSteps[0];
            m_computing = false;
        }
    } else {
        // Clear all layers when no objects exist
        m_layers.clear();
        m_layerRefiners.clear();
        m_angleSteps.clear();
        m_angleDeg = 0.0f;
    }
}

// Start an ImGui frame, draw the UI, then render the grid, objects, and ImGui
void App::Render()
{
    // --- ImGui new frame ---
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    // --- Panel de creacion ---
    DrawUI();

    // --- Render ---
    SDL_SetRenderDrawColor(m_renderer, 30, 30, 30, 255);
    SDL_RenderClear(m_renderer);

    int halfW  = m_windowW / 2;
    int topY   = static_cast<int>(m_toolbarH);
    int areaH  = m_windowH - topY;

    // Left half: physical space (grid + objects)
    {
        SDL_Rect leftViewport = { 0, topY, halfW, areaH };
        SDL_RenderSetViewport(m_renderer, &leftViewport);

        // Draw current layer's grid
        auto gridIt = m_layers.find(m_currentAngle);
        if (gridIt != m_layers.end()) {
            gridIt->second.Draw(m_renderer);
            gridIt->second.DrawOrigin(m_renderer);
        }

        // Draw objects (dynamic ones rotated according to current angle)
        for (const auto& obj : m_objects) {
            if (obj.IsDynamic()) {
                obj.DrawRotated(m_renderer, static_cast<float>(m_currentAngle));

                // Draw preview of translated position if active
                if (m_previewTranslation) {
                    obj.DrawRotatedTranslated(m_renderer, static_cast<float>(m_currentAngle), m_previewOffset);
                }
            } else {
                obj.Draw(m_renderer);
            }
        }

        // Draw Bézier control points if placing them
        if (m_placingBezierPoints) {
            for (size_t i = 0; i < m_bezierControlPoints.size(); ++i) {
                const Vec2& pt = m_bezierControlPoints[i];

                // Draw control point
                filledCircleRGBA(m_renderer, 
                    static_cast<Sint16>(pt.x), 
                    static_cast<Sint16>(pt.y), 
                    5, 255, 255, 0, 255);

                // Draw line to next control point
                if (i > 0) {
                    const Vec2& prevPt = m_bezierControlPoints[i - 1];
                    aalineRGBA(m_renderer,
                        static_cast<Sint16>(prevPt.x), static_cast<Sint16>(prevPt.y),
                        static_cast<Sint16>(pt.x), static_cast<Sint16>(pt.y),
                        255, 255, 0, 180);
                }
            }

            // Draw preview curve if we have at least 2 points
            if (m_bezierControlPoints.size() >= 2) {
                // Convert to local coordinates (relative to first point)
                Vec2 center = m_bezierControlPoints[0];
                std::vector<Vec2> localPoints;
                for (const auto& pt : m_bezierControlPoints) {
                    localPoints.push_back({ pt.x - center.x, pt.y - center.y });
                }

                // Create temporary preview object
                Object preview = Object::CreateFreeform(center, localPoints, m_bezierCloseCurve);
                auto verts = preview.GetWorldVertices();

                // Draw preview curve
                for (size_t i = 1; i < verts.size(); ++i) {
                    aalineRGBA(m_renderer,
                        static_cast<Sint16>(verts[i-1].x), static_cast<Sint16>(verts[i-1].y),
                        static_cast<Sint16>(verts[i].x), static_cast<Sint16>(verts[i].y),
                        0, 255, 255, 200);
                }
            }
        }

        SDL_RenderSetViewport(m_renderer, nullptr);
    }

    // Separator line
    SDL_SetRenderDrawColor(m_renderer, 100, 100, 100, 255);
    SDL_RenderDrawLine(m_renderer, halfW, topY, halfW, m_windowH);

    // Right half: C-Space
    {
        SDL_Rect rightViewport = { halfW + 1, topY, halfW - 1, areaH };
        SDL_RenderSetViewport(m_renderer, &rightViewport);

        // Draw current layer's C-Space
        auto gridIt = m_layers.find(m_currentAngle);
        if (gridIt != m_layers.end()) {
            gridIt->second.DrawCSpace(m_renderer);
            gridIt->second.DrawOrigin(m_renderer);
        }

        auto refinerIt = m_layerRefiners.find(m_currentAngle);
        if (refinerIt != m_layerRefiners.end()) {
            refinerIt->second.Draw(m_renderer);
        }

        SDL_RenderSetViewport(m_renderer, nullptr);
    }

    // --- C-Space query overlay (mouse hover) ---
    auto gridIt = m_layers.find(m_currentAngle);
    auto refinerIt = m_layerRefiners.find(m_currentAngle);

    if (gridIt != m_layers.end() && gridIt->second.HasCSpace()) {
        const Grid& currentGrid = gridIt->second;
        const CSpaceRefiner& currentRefiner = (refinerIt != m_layerRefiners.end()) 
            ? refinerIt->second 
            : m_layerRefiners[m_currentAngle]; // default-construct if missing

        ImGuiIO& io = ImGui::GetIO();
        float mx = io.MousePos.x;
        float my = io.MousePos.y;

        bool inCSpace = !io.WantCaptureMouse
                      && mx > static_cast<float>(halfW + 1)
                      && my > static_cast<float>(topY)
                      && mx < static_cast<float>(m_windowW)
                      && my < static_cast<float>(m_windowH);

        if (inCSpace) {
            Vec2 mousePos = { mx - static_cast<float>(halfW + 1),
                              my - static_cast<float>(topY) };

            // First query to get refinement level at mouse position (no snap yet)
            CSpaceQueryResult prelimResult = currentRefiner.Query(mousePos, currentGrid);

            // Get base grid info
            float sX = currentGrid.GetStartX();
            float sY = currentGrid.GetStartY();
            float cs = currentGrid.GetCellSize();

            // Adjust cell size based on refinement level at mouse position
            for (int l = 1; l < prelimResult.level; ++l) cs *= 0.5f;

            // Snap to nearest vertex at the refined level
            int nearestCol = static_cast<int>(std::round((mousePos.x - sX) / cs));
            int nearestRow = static_cast<int>(std::round((mousePos.y - sY) / cs));

            // Snapped position using refined cell size
            Vec2 qp = { sX + nearestCol * cs, sY + nearestRow * cs };

            // Final query at the snapped position
            CSpaceQueryResult result = currentRefiner.Query(qp, currentGrid);

            // Compute the cell size for this level (for display purposes)
            float csLevel = currentGrid.GetCellSize();
            for (int l = 1; l < result.level; ++l) csLevel *= 0.5f;

            // Highlight the cell in the C-Space viewport
            {
                SDL_Rect rv = { halfW + 1, topY, halfW - 1, areaH };
                SDL_RenderSetViewport(m_renderer, &rv);

                // Draw crosshair at snapped vertex position
                int cx = static_cast<int>(qp.x);
                int cy = static_cast<int>(qp.y);
                SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 255);
                SDL_RenderDrawLine(m_renderer, cx - 8, cy, cx + 8, cy);
                SDL_RenderDrawLine(m_renderer, cx, cy - 8, cx, cy + 8);

                // Draw filled circle at vertex
                filledCircleRGBA(m_renderer, cx, cy, 4, 255, 255, 255, 255);

                SDL_RenderSetViewport(m_renderer, nullptr);
            }

            // Tooltip
            ImGui::SetNextWindowPos(ImVec2(mx + 15.0f, my + 15.0f));
            ImGui::SetNextWindowBgAlpha(0.85f);
            ImGui::Begin("##CSpaceQuery", nullptr,
                ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                ImGuiWindowFlags_NoNav);

            ImGui::Text("t = (%.1f, %.1f)", qp.x, qp.y);
            switch (result.status) {
            case CSpaceStatus::Safe:
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Non-collision");
                break;
            case CSpaceStatus::Unsafe:
                ImGui::TextColored(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), "Collision");
                break;
            case CSpaceStatus::Uncertain:
                ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Undecided");
                break;
            }
            ImGui::Text("Level: %d", result.level);
            ImGui::End();
        }
    }

    ImGui::Render();
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), m_renderer);

    SDL_RenderPresent(m_renderer);
}

// Horizontal toolbar at the top of the screen (menu-bar style)
void App::DrawUI()
{
    static const char* shapeNames[] = { "Triangle", "Square", "Circle", "Polygon", "Freeform" };
    const float halfW = static_cast<float>(m_windowW) * 0.5f;

    // Full-width toolbar window pinned to the top
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(static_cast<float>(m_windowW), 0.0f), ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::Begin("##Toolbar", nullptr,
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_MenuBar);

    if (ImGui::BeginMenuBar())
    {
        // =================================================================
        // SHAPE menu
        // =================================================================
        if (ImGui::BeginMenu("Shape"))
        {
            ImGui::Combo("##Shape", &m_uiShapeType, shapeNames, IM_ARRAYSIZE(shapeNames));

            float panelH = static_cast<float>(m_windowH) - m_toolbarH;
            if (static_cast<ShapeType>(m_uiShapeType) != ShapeType::Freeform) {
                ImGui::DragFloat("Pos X", &m_uiPosition[0], 1.0f, 0.0f, halfW,  "%.0f");
                ImGui::DragFloat("Pos Y", &m_uiPosition[1], 1.0f, 0.0f, panelH, "%.0f");
            }

            switch (static_cast<ShapeType>(m_uiShapeType)) {
            case ShapeType::Triangle:
            case ShapeType::Square:
                ImGui::DragFloat("Size", &m_uiSize, 1.0f, 10.0f, 400.0f, "%.0f");
                break;
            case ShapeType::Circle:
                ImGui::DragFloat("Radius", &m_uiSize, 1.0f, 5.0f, 300.0f, "%.0f");
                break;
            case ShapeType::Polygon:
                ImGui::SliderInt("Sides", &m_uiPolySides, 3, 20);
                ImGui::DragFloat("Radius", &m_uiSize, 1.0f, 10.0f, 300.0f, "%.0f");
                break;
            case ShapeType::Freeform:
                ImGui::Checkbox("Close Shape", &m_bezierCloseCurve);
                ImGui::Text("Points: %d", static_cast<int>(m_bezierControlPoints.size()));
                if (m_placingBezierPoints) {
                    ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), 
                        "Click on left panel to place points");
                }
                break;
            }

            ImGui::Separator();
            ImGui::Checkbox("Dynamic", &m_uiDynamic);

            ImGui::Separator();

            // Green "Create" button
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.15f, 0.55f, 0.15f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,  ImVec4(0.20f, 0.70f, 0.20f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,   ImVec4(0.10f, 0.45f, 0.10f, 1.0f));

            // Special handling for Bézier curves
            if (static_cast<ShapeType>(m_uiShapeType) == ShapeType::Freeform) {
                if (!m_placingBezierPoints) {
                    // Start placing points
                    if (ImGui::Button("Start Placing Points", ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
                        m_placingBezierPoints = true;
                        m_bezierControlPoints.clear();
                    }
                } else {
                    // Finish and create the curve
                    if (ImGui::Button("Finish Curve", ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
                        if (m_bezierControlPoints.size() >= 2) {
                            // Convert to local coordinates
                            Vec2 center = m_bezierControlPoints[0];
                            std::vector<Vec2> localPoints;
                            for (const auto& pt : m_bezierControlPoints) {
                                localPoints.push_back({ pt.x - center.x, pt.y - center.y });
                            }

                            m_objects.push_back(Object::CreateFreeform(center, localPoints, m_bezierCloseCurve));

                            if (m_uiDynamic) {
                                m_objects.back().SetDynamic(true);
                            }
                        }
                        m_placingBezierPoints = false;
                        m_bezierControlPoints.clear();
                    }

                    ImGui::SameLine();

                    // Cancel button
                    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.55f, 0.15f, 0.15f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,  ImVec4(0.70f, 0.20f, 0.20f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,   ImVec4(0.45f, 0.10f, 0.10f, 1.0f));
                    if (ImGui::Button("Cancel", ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
                        m_placingBezierPoints = false;
                        m_bezierControlPoints.clear();
                    }
                    ImGui::PopStyleColor(3);
                }
            } else if (ImGui::Button("+ Create", ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
                Vec2 pos = { m_uiPosition[0], m_uiPosition[1] };

                switch (static_cast<ShapeType>(m_uiShapeType)) {
                case ShapeType::Triangle:
                    m_objects.push_back(Object::CreateTriangle(pos, m_uiSize));
                    break;
                case ShapeType::Square:
                    m_objects.push_back(Object::CreateSquare(pos, m_uiSize));
                    break;
                case ShapeType::Circle:
                    m_objects.push_back(Object::CreateCircle(pos, m_uiSize));
                    break;
                case ShapeType::Polygon: {
                    std::vector<Vec2> verts;
                    const float step = 2.0f * glm::pi<float>() / static_cast<float>(m_uiPolySides);
                    for (int i = 0; i < m_uiPolySides; ++i) {
                        float angle = step * static_cast<float>(i) - static_cast<float>(std::numbers::pi) / 2.0f;
                        verts.push_back({ m_uiSize * std::cos(angle),
                                          m_uiSize * std::sin(angle) });
                    }
                    m_objects.push_back(Object::CreatePolygon(pos, verts));
                    break;
                }
                }

                if (m_uiDynamic) {
                    m_objects.back().SetDynamic(true);
                }
            }
            ImGui::PopStyleColor(3);

            ImGui::EndMenu();
        }

        // =================================================================
        // SETTINGS menu
        // =================================================================
        if (ImGui::BeginMenu("Settings"))
        {
            ImGui::SliderFloat("Cell Size", &m_cellSize, 5.0f, 100.0f, "%.0f px");
            ImGui::SliderInt("Refinement", &m_refinementLevels, 0, 5);

            ImGui::Separator();

            // Angular step info (read-only)
            if (m_angleDeg > 0.0f) {
                float smallestCell = m_cellSize / std::pow(2.0f, static_cast<float>(m_refinementLevels));
                int numLayers = static_cast<int>(std::ceil(360.0f / m_angleDeg));
                ImGui::Text("Min cell size: %.2f px", smallestCell);
                ImGui::Text("Angular step:  %.2f deg", m_angleDeg);
                ImGui::Text("Layers: %d", numLayers);
            } else {
                ImGui::TextDisabled("Angular step: (no dynamic object)");
            }

            ImGui::Separator();

            // Layer navigation
            ImGui::Text("Current layer: %d deg  [%d / %d]",
                m_currentAngle,
                m_angleSteps.empty() ? 0 : m_currentLayerIndex,
                static_cast<int>(m_angleSteps.size()));

            if (!m_angleSteps.empty()) {
                if (ImGui::ArrowButton("##prev", ImGuiDir_Left)) {
                    m_currentLayerIndex = (m_currentLayerIndex - 1 + static_cast<int>(m_angleSteps.size())) 
                                         % static_cast<int>(m_angleSteps.size());
                    m_currentAngle = m_angleSteps[m_currentLayerIndex];
                }
                ImGui::SameLine();
                if (ImGui::ArrowButton("##next", ImGuiDir_Right)) {
                    m_currentLayerIndex = (m_currentLayerIndex + 1) % static_cast<int>(m_angleSteps.size());
                    m_currentAngle = m_angleSteps[m_currentLayerIndex];
                }
            }

            ImGui::Separator();

            // Red "Clear All" button
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.55f, 0.15f, 0.15f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,  ImVec4(0.70f, 0.20f, 0.20f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,   ImVec4(0.45f, 0.10f, 0.10f, 1.0f));
            if (ImGui::Button("Clear All", ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
                m_objects.clear();
                m_layers.clear();
                m_layerRefiners.clear();
                m_angleSteps.clear();
                m_angleDeg = 0.0f;
                m_currentAngle = 0;
                m_currentLayerIndex = 0;
                m_gridOriginSet = false;
            }
            ImGui::PopStyleColor(3);

            ImGui::EndMenu();
        }

        // =================================================================
        // OBJECTS menu
        // =================================================================
        if (ImGui::BeginMenu("Objects"))
        {
            ImGui::Text("Count: %d", static_cast<int>(m_objects.size()));
            ImGui::Separator();

            for (int i = 0; i < static_cast<int>(m_objects.size()); ++i) {
                const auto& obj = m_objects[i];
                const char* name = shapeNames[static_cast<int>(obj.GetShapeType())];
                Vec2 p = obj.GetPosition();

                ImGui::PushID(i);

                if (obj.IsDynamic())
                    ImGui::TextColored(ImVec4(0.3f, 1.0f, 1.0f, 1.0f), ">");
                else
                    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "-");

                ImGui::SameLine();
                ImGui::Text("#%d %s (%.0f,%.0f)", i, name, p.x, p.y);

                ImGui::SameLine(ImGui::GetContentRegionAvail().x + ImGui::GetCursorPosX() - 20.0f);
                ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.50f, 0.10f, 0.10f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered,  ImVec4(0.70f, 0.20f, 0.20f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive,   ImVec4(0.40f, 0.08f, 0.08f, 1.0f));
                if (ImGui::SmallButton("X")) {
                    m_objects.erase(m_objects.begin() + i);
                    ImGui::PopStyleColor(3);
                    ImGui::PopID();
                    break;
                }
                ImGui::PopStyleColor(3);

                ImGui::PopID();
            }

            ImGui::EndMenu();
        }

        // =================================================================
        // Centered Start button
        // =================================================================
        {
            float buttonW = 80.0f;
            float barW = ImGui::GetWindowWidth();
            float cursorX = (barW - buttonW) * 0.5f;
            float menuEnd = ImGui::GetCursorPosX();
            if (cursorX > menuEnd)
                ImGui::SetCursorPosX(cursorX);

            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.15f, 0.45f, 0.65f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,  ImVec4(0.20f, 0.55f, 0.80f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,   ImVec4(0.10f, 0.35f, 0.50f, 1.0f));
            if (ImGui::Button("Start", ImVec2(buttonW, 0))) {
                m_computing = true;
            }
            ImGui::PopStyleColor(3);
        }

        ImGui::EndMenuBar();
    }

    // Store toolbar height so Render() can offset the scene below it
    m_toolbarH = ImGui::GetWindowSize().y;

    ImGui::End();
    ImGui::PopStyleVar(2);
}
