#include "CLI.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <sstream>
#include <string>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static const char* kShapeNames[] = { "Triangle", "Square", "Circle", "Polygon", "Freeform" };
static constexpr int kNumShapes  = 5;

// Read one trimmed line from stdin; returns false on EOF.
static bool ReadLine(const std::string& prompt, std::string& out)
{
    std::cout << prompt;
    std::cout.flush();
    if (!std::getline(std::cin, out))
        return false;
    // trim leading/trailing whitespace
    auto first = out.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) { out.clear(); return true; }
    auto last  = out.find_last_not_of(" \t\r\n");
    out = out.substr(first, last - first + 1);
    return true;
}

// Parse a float from s; if empty or invalid, return defaultVal.
static float ParseFloat(const std::string& s, float defaultVal)
{
    if (s.empty()) return defaultVal;
    try { return std::stof(s); } catch (...) { return defaultVal; }
}

// Parse an int from s; if empty or invalid, return defaultVal.
static int ParseInt(const std::string& s, int defaultVal)
{
    if (s.empty()) return defaultVal;
    try { return std::stoi(s); } catch (...) { return defaultVal; }
}

// ---------------------------------------------------------------------------
// PrintHelp
// ---------------------------------------------------------------------------

void CLI::PrintHelp() const
{
    std::cout <<
        "\nCommands:\n"
        "  add      - Add a new shape\n"
        "  list     - List all shapes\n"
        "  remove   - Remove a shape by index\n"
        "  settings - Configure cell size, refinement and workspace\n"
        "  compute  - Compute C-Space for all angle layers\n"        "  query    - Query collision at a C-Space position\n"
        "  clear    - Remove all shapes\n"
        "  help     - Show this help\n"
        "  quit     - Exit\n\n";
}

// ---------------------------------------------------------------------------
// CmdAdd
// ---------------------------------------------------------------------------

void CLI::CmdAdd()
{
    std::string line;

    // --- shape type ---
    std::cout << "Shape types:\n";
    for (int i = 0; i < kNumShapes; ++i)
        std::cout << "  " << i << " = " << kShapeNames[i] << "\n";
    ReadLine("Shape type [0]: ", line);
    int shapeType = std::clamp(ParseInt(line, 0), 0, kNumShapes - 1);

    auto st = static_cast<ShapeType>(shapeType);

    float posX = 200.0f, posY = 200.0f;
    float size  = 80.0f;
    int   sides = 5;

    // --- position (not needed for Freeform; user enters raw points) ---
    if (st != ShapeType::Freeform) {
        ReadLine("Position X [200]: ", line);
        posX = ParseFloat(line, 200.0f);
        ReadLine("Position Y [200]: ", line);
        posY = ParseFloat(line, 200.0f);
    }

    // --- shape-specific parameters ---
    switch (st) {
    case ShapeType::Triangle:
    case ShapeType::Square:
        ReadLine("Size [80]: ", line);
        size = std::clamp(ParseFloat(line, 80.0f), 1.0f, 10000.0f);
        break;
    case ShapeType::Circle:
        ReadLine("Radius [80]: ", line);
        size = std::clamp(ParseFloat(line, 80.0f), 1.0f, 10000.0f);
        break;
    case ShapeType::Polygon:
        ReadLine("Sides [5]: ", line);
        sides = std::clamp(ParseInt(line, 5), 3, 20);
        ReadLine("Radius [80]: ", line);
        size = std::clamp(ParseFloat(line, 80.0f), 1.0f, 10000.0f);
        break;
    case ShapeType::Freeform:
        break;
    }

    // --- dynamic flag ---
    ReadLine("Dynamic? (y/n) [n]: ", line);
    bool dynamic = (!line.empty() && (line[0] == 'y' || line[0] == 'Y'));

    // --- create the object ---
    if (st == ShapeType::Freeform) {
        ReadLine("Close curve? (y/n) [n]: ", line);
        bool closeCurve = (!line.empty() && (line[0] == 'y' || line[0] == 'Y'));

        std::cout << "Enter control points as \"x y\", empty line when done (need >= 2):\n";
        std::vector<Vec2> localPoints;
        Vec2 center = {};
        bool firstPoint = true;

        while (true) {
            std::ostringstream prompt;
            prompt << "  Point " << localPoints.size() << ": ";
            if (!ReadLine(prompt.str().c_str(), line)) break;

            if (line.empty()) {
                if (localPoints.size() >= 2) break;
                std::cout << "  Need at least 2 points.\n";
                continue;
            }

            std::istringstream iss(line);
            float x = 0.0f, y = 0.0f;
            if (!(iss >> x >> y)) {
                std::cout << "  Invalid format. Enter two numbers separated by a space.\n";
                continue;
            }

            if (firstPoint) {
                center = { x, y };
                localPoints.push_back({ 0.0f, 0.0f });
                firstPoint = false;
            } else {
                localPoints.push_back({ x - center.x, y - center.y });
            }
        }

        m_objects.push_back(Object::CreateFreeform(center, localPoints, closeCurve));
    } else {
        Vec2 pos = { posX, posY };
        switch (st) {
        case ShapeType::Triangle:
            m_objects.push_back(Object::CreateTriangle(pos, size));
            break;
        case ShapeType::Square:
            m_objects.push_back(Object::CreateSquare(pos, size));
            break;
        case ShapeType::Circle:
            m_objects.push_back(Object::CreateCircle(pos, size));
            break;
        case ShapeType::Polygon: {
            std::vector<Vec2> verts;
            const float step = 2.0f * static_cast<float>(std::numbers::pi) / static_cast<float>(sides);
            for (int i = 0; i < sides; ++i) {
                float angle = step * static_cast<float>(i)
                            - static_cast<float>(std::numbers::pi) / 2.0f;
                verts.push_back({ size * std::cos(angle), size * std::sin(angle) });
            }
            m_objects.push_back(Object::CreatePolygon(pos, verts));
            break;
        }
        default: break;
        }
    }

    if (dynamic) {
        m_objects.back().SetDynamic(true);
        if (!m_gridOriginSet) {
            m_gridOrigin    = m_objects.back().GetPosition();
            m_gridOriginSet = true;
        }
    }

    std::cout << "Shape #" << (m_objects.size() - 1)
              << " (" << kShapeNames[shapeType] << ") added.\n";

    // Invalidate previously computed C-Space
    m_layers.clear();
    m_layerRefiners.clear();
    m_angleSteps.clear();
}

// ---------------------------------------------------------------------------
// CmdList
// ---------------------------------------------------------------------------

void CLI::CmdList() const
{
    if (m_objects.empty()) {
        std::cout << "No shapes.\n";
        return;
    }
    for (int i = 0; i < static_cast<int>(m_objects.size()); ++i) {
        const auto& obj = m_objects[i];
        Vec2 p = obj.GetPosition();
        std::cout << "  #" << i
                  << " " << kShapeNames[static_cast<int>(obj.GetShapeType())]
                  << "  pos=(" << p.x << ", " << p.y << ")"
                  << (obj.IsDynamic() ? "  [dynamic]" : "  [static]")
                  << "\n";
    }
}

// ---------------------------------------------------------------------------
// CmdRemove
// ---------------------------------------------------------------------------

void CLI::CmdRemove()
{
    CmdList();
    if (m_objects.empty()) return;

    std::string line;
    ReadLine("Index to remove: ", line);
    int idx = ParseInt(line, -1);

    if (idx < 0 || idx >= static_cast<int>(m_objects.size())) {
        std::cout << "Invalid index.\n";
        return;
    }

    m_objects.erase(m_objects.begin() + idx);
    std::cout << "Shape #" << idx << " removed.\n";

    // Re-derive grid origin from remaining dynamic objects
    m_gridOriginSet = false;
    for (const auto& obj : m_objects) {
        if (obj.IsDynamic()) {
            m_gridOrigin    = obj.GetPosition();
            m_gridOriginSet = true;
            break;
        }
    }

    // Invalidate C-Space
    m_layers.clear();
    m_layerRefiners.clear();
    m_angleSteps.clear();
}

// ---------------------------------------------------------------------------
// CmdSettings
// ---------------------------------------------------------------------------

void CLI::CmdSettings()
{
    std::string line;

    std::cout << "--- Cell size (5..100) [" << m_cellSize << "]: ";
    std::getline(std::cin, line);
    if (!line.empty())
        m_cellSize = std::clamp(ParseFloat(line, m_cellSize), 5.0f, 100.0f);

    std::cout << "--- Refinement levels (0..5) [" << m_refinementLevels << "]: ";
    std::getline(std::cin, line);
    if (!line.empty())
        m_refinementLevels = std::clamp(ParseInt(line, m_refinementLevels), 0, 5);

    std::cout << "--- Workspace width [" << m_workspaceW << "]: ";
    std::getline(std::cin, line);
    if (!line.empty())
        m_workspaceW = std::max(ParseFloat(line, m_workspaceW), 1.0f);

    std::cout << "--- Workspace height [" << m_workspaceH << "]: ";
    std::getline(std::cin, line);
    if (!line.empty())
        m_workspaceH = std::max(ParseFloat(line, m_workspaceH), 1.0f);

    std::cout << "Settings updated: cell=" << m_cellSize
              << " px, refinement=" << m_refinementLevels
              << ", workspace=" << m_workspaceW << "x" << m_workspaceH << "\n";

    // Invalidate C-Space
    m_layers.clear();
    m_layerRefiners.clear();
    m_angleSteps.clear();
}

// ---------------------------------------------------------------------------
// CmdCompute
// ---------------------------------------------------------------------------

void CLI::CmdCompute()
{
    bool hasDynamic = false;
    for (const auto& obj : m_objects)
        if (obj.IsDynamic()) { hasDynamic = true; break; }

    if (!hasDynamic) {
        std::cout << "No dynamic object. Add at least one shape marked as dynamic.\n";
        return;
    }

    // Lock grid origin to center of first dynamic object if not yet set
    if (!m_gridOriginSet) {
        for (const auto& obj : m_objects) {
            if (obj.IsDynamic()) {
                m_gridOrigin    = obj.GetPosition();
                m_gridOriginSet = true;
                break;
            }
        }
    }

    // Compute angular step via the base layer (angle = 0)
    {
        Grid baseGrid;
        baseGrid.Build(m_workspaceW, m_workspaceH, m_cellSize,
                       m_objects, m_gridOrigin, 0.0f);

        float d = baseGrid.GetMaxDynamicInnerDistance(m_gridOrigin);
        if (d > 0.0f) {
            float smallestCell = m_cellSize
                               / std::pow(2.0f, static_cast<float>(m_refinementLevels));
            float stepRad = smallestCell / (2.0f * d);
            m_angleDeg = stepRad * (180.0f / static_cast<float>(std::numbers::pi));
        } else {
            m_angleDeg = 0.0f;
        }
    }

    // Build the list of angle steps
    m_angleSteps.clear();
    if (m_angleDeg > 0.0f) {
        int numLayers = static_cast<int>(std::ceil(360.0f / m_angleDeg));
        for (int i = 0; i < numLayers; ++i) {
            int a = static_cast<int>(std::round(i * m_angleDeg)) % 360;
            m_angleSteps.push_back(a);
        }
    } else {
        m_angleSteps.push_back(0);
    }

    std::cout << "Computing " << m_angleSteps.size() << " layer(s)";
    if (m_angleDeg > 0.0f)
        std::cout << " (angular step: " << m_angleDeg << " deg)";
    std::cout << "...\n";

    m_layers.clear();
    m_layerRefiners.clear();

    for (int angle : m_angleSteps) {
        Grid& g = m_layers[angle];
        g.Build(m_workspaceW, m_workspaceH, m_cellSize,
                m_objects, m_gridOrigin, static_cast<float>(angle));
        g.ComputeCSpace();

        CSpaceRefiner& refiner = m_layerRefiners[angle];
        if (m_refinementLevels > 0)
            refiner.Refine(g, m_objects, m_refinementLevels, static_cast<float>(angle));
        else
            refiner.Clear();
    }

    std::cout << "Done. Use 'query' to check collision at a C-Space position.\n";
}

// ---------------------------------------------------------------------------
// CmdQuery
// ---------------------------------------------------------------------------

void CLI::CmdQuery() const
{
    if (m_layers.empty()) {
        std::cout << "C-Space not computed yet. Run 'compute' first.\n";
        return;
    }

    std::string line;

    // --- choose angle layer ---
    int angle = m_angleSteps.empty() ? 0 : m_angleSteps[0];

    if (m_angleSteps.size() > 1) {
        std::cout << "Available angles:";
        for (int a : m_angleSteps) std::cout << " " << a;
        std::cout << "\n";

        ReadLine("Angle (deg) [" + std::to_string(angle) + "]: ", line);
        int requested = ParseInt(line, angle);

        // Snap to nearest available angle
        int best = m_angleSteps[0];
        int bestDiff = std::abs(requested - best);
        for (int a : m_angleSteps) {
            int diff = std::abs(requested - a);
            if (diff < bestDiff) { bestDiff = diff; best = a; }
        }
        angle = best;
    }

    auto layerIt = m_layers.find(angle);
    if (layerIt == m_layers.end()) {
        std::cout << "Layer not found for angle " << angle << ".\n";
        return;
    }
    const Grid& grid = layerIt->second;

    // --- query position ---
    ReadLine("Query position X [" + std::to_string(static_cast<int>(m_gridOrigin.x)) + "]: ", line);
    float qx = ParseFloat(line, m_gridOrigin.x);
    ReadLine("Query position Y [" + std::to_string(static_cast<int>(m_gridOrigin.y)) + "]: ", line);
    float qy = ParseFloat(line, m_gridOrigin.y);

    Vec2 queryPos = { qx, qy };

    // --- run query ---
    auto refinerIt = m_layerRefiners.find(angle);
    CSpaceQueryResult result;
    if (refinerIt != m_layerRefiners.end()) {
        result = refinerIt->second.Query(queryPos, grid);
    } else {
        CSpaceRefiner tmp;
        result = tmp.Query(queryPos, grid);
    }

    // --- report ---
    std::cout << "\n=== C-Space Query Result ===\n";
    std::cout << "Position : (" << qx << ", " << qy << ")  angle=" << angle << " deg\n";
    std::cout << "Status   : ";
    switch (result.status) {
    case CSpaceStatus::Safe:
        std::cout << "NON-COLLISION (Safe)\n";
        break;
    case CSpaceStatus::Unsafe:
        std::cout << "COLLISION (Unsafe)\n";
        break;
    case CSpaceStatus::Uncertain:
        std::cout << "UNCERTAIN (Undecided — try increasing refinement)\n";
        break;
    }
    std::cout << "Level    : " << result.level << "\n\n";
}

// ---------------------------------------------------------------------------
// CmdClear
// ---------------------------------------------------------------------------

void CLI::CmdClear()
{
    m_objects.clear();
    m_layers.clear();
    m_layerRefiners.clear();
    m_angleSteps.clear();
    m_angleDeg      = 0.0f;
    m_gridOriginSet = false;
    std::cout << "All shapes cleared.\n";
}

// ---------------------------------------------------------------------------
// Run — main interactive loop
// ---------------------------------------------------------------------------

void CLI::Run()
{
    std::cout << "=== Collisions CLI ===\n";
    std::cout << "Type 'help' for a list of commands.\n\n";

    std::string cmd;
    while (true) {
        std::cout << "> ";
        std::cout.flush();
        if (!std::getline(std::cin, cmd)) break;  // EOF

        // trim
        auto first = cmd.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) continue;
        auto last = cmd.find_last_not_of(" \t\r\n");
        cmd = cmd.substr(first, last - first + 1);

        if (cmd.empty()) continue;

        if (cmd == "quit" || cmd == "exit" || cmd == "q") {
            break;
        } else if (cmd == "help" || cmd == "h" || cmd == "?") {
            PrintHelp();
        } else if (cmd == "add" || cmd == "a") {
            CmdAdd();
        } else if (cmd == "list" || cmd == "ls" || cmd == "l") {
            CmdList();
        } else if (cmd == "remove" || cmd == "rm" || cmd == "r") {
            CmdRemove();
        } else if (cmd == "settings" || cmd == "s") {
            CmdSettings();
        } else if (cmd == "compute" || cmd == "comp") {
            CmdCompute();
        } else if (cmd == "query" || cmd == "qr") {
            CmdQuery();
        } else if (cmd == "clear") {
            CmdClear();
        } else {
            std::cout << "Unknown command: '" << cmd << "'. Type 'help' for help.\n";
        }
    }
}
