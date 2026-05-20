#include "Object.h"

#include <SDL2/SDL2_gfxPrimitives.h>
#include <numbers>

// --- Factory methods ----------------------------------------------------
// Each one builds an Object with the right shape type and local vertices.
// Vertices are centered around (0,0) so moving the object is just changing m_position.

// Equilateral triangle centered at the origin
Object Object::CreateTriangle(Vec2 position, float size)
{
	Object obj;
	obj.m_shapeType = ShapeType::Triangle;
	obj.m_position  = position;

	const float h = size * std::sqrt(3.0f) / 2.0f;
	obj.m_vertices = {
		{  0.0f,        -h * 2.0f / 3.0f },   // top
		{ -size / 2.0f,  h * 1.0f / 3.0f },   // bottom-left
		{  size / 2.0f,  h * 1.0f / 3.0f }    // bottom-right
	};

	return obj;
}

// Square centered at the origin
Object Object::CreateSquare(Vec2 position, float size)
{
    Object obj;
    obj.m_shapeType = ShapeType::Square;
    obj.m_position  = position;

	const float half = size / 2.0f;
	obj.m_vertices = {
		{ -half, -half },   // top-left
		{  half, -half },   // top-right
		{  half,  half },   // bottom-right
		{ -half,  half }    // bottom-left
	};

    return obj;
}

// Circle approximated with N vertices placed around the circumference
Object Object::CreateCircle(Vec2 position, float radius, int segments)
{
    Object obj;
    obj.m_shapeType = ShapeType::Circle;
    obj.m_position  = position;
    obj.m_radius    = radius;

    const float step = 2.0f * static_cast<float>(std::numbers::pi) / static_cast<float>(segments);
    obj.m_vertices.reserve(segments);

    for (int i = 0; i < segments; ++i) {
        const float angle = step * static_cast<float>(i);
        obj.m_vertices.push_back({ radius * std::cos(angle),
                                   radius * std::sin(angle) });
    }

    return obj;
}

// Arbitrary polygon from user-provided local vertices
Object Object::CreatePolygon(Vec2 position, const std::vector<Vec2>& vertices)
{
	Object obj;
	obj.m_shapeType = ShapeType::Polygon;
	obj.m_position  = position;
	obj.m_vertices  = vertices;
	return obj;
}


// Freeform polygon from user-placed points (straight lines between them)
Object Object::CreateFreeform(Vec2 position, const std::vector<Vec2>& points, bool closeCurve)
{
	Object obj;
	obj.m_shapeType = ShapeType::Freeform;
	obj.m_position  = position;
	obj.m_vertices  = points;
	// closeCurve: last point connects back to first — the draw loop handles this automatically
	// since the vertex list already forms a closed polygon when drawn as a polyline
	if (closeCurve && points.size() >= 2) {
		obj.m_vertices.push_back(points[0]);
	}
	return obj;
}

// --- Utilities ----------------------------------------------------------

// Translate every local vertex by the object's position to get world coordinates
std::vector<Vec2> Object::GetWorldVertices() const
{
    std::vector<Vec2> world;
    world.reserve(m_vertices.size());
    for (const auto& v : m_vertices)
        world.push_back(v + m_position);
	return world;
}

// Build the smallest axis-aligned box that contains the whole object
AABB Object::GetAABB() const
{
	if (m_shapeType == ShapeType::Circle) {
		return { m_position - Vec2(m_radius), m_position + Vec2(m_radius) };
	}

	auto world = GetWorldVertices();
	if (world.empty()) return {};

	AABB box{ world[0], world[0] };
	for (const auto& v : world) {
		box.min.x = std::min(box.min.x, v.x);
		box.min.y = std::min(box.min.y, v.y);
		box.max.x = std::max(box.max.x, v.x);
		box.max.y = std::max(box.max.y, v.y);
	}
	return box;
}

// Creates a rotated copy of this object (rotates vertices around the center)
Object Object::GetRotated(float angleDegrees) const
{
	Object rotated = *this;

	// Circles don't need vertex rotation
	if (m_shapeType == ShapeType::Circle) {
		return rotated;
	}

	// Convert degrees to radians (negate for clockwise rotation)
	const float angleRad = -angleDegrees * static_cast<float>(std::numbers::pi) / 180.0f;
	const float cosA = std::cos(angleRad);
	const float sinA = std::sin(angleRad);

	// Rotate each local vertex around (0,0)
	for (auto& v : rotated.m_vertices) {
		float x = v.x * cosA - v.y * sinA;
		float y = v.x * sinA + v.y * cosA;
		v.x = x;
		v.y = y;
	}

	return rotated;
}

// --- Simulation --------------------------------------------------------

// Move the object along its direction. Only does something for dynamic objects.
void Object::Update(float dt)
{
	if (!m_dynamic || m_speed == 0.0f) return;
	m_position += m_direction * m_speed * dt;
}

// --- Drawing ------------------------------------------------------------

// Pick a color based on state and draw the shape using SDL2_gfx primitives
void Object::Draw(SDL_Renderer* renderer) const
{
	// Red = colliding, Cyan = dynamic, Green = static
	Uint8 r, g, b;
	if (m_colliding)    { r = 255; g = 60;  b = 60;  }
	else if (m_dynamic) { r = 0;   g = 255; b = 255; }
	else                { r = 0;   g = 255; b = 0;   }
	constexpr Uint8 a = 255;

	switch (m_shapeType) {
	case ShapeType::Circle:
		aacircleRGBA(renderer,
			static_cast<Sint16>(m_position.x),
			static_cast<Sint16>(m_position.y),
			static_cast<Sint16>(m_radius),
			r, g, b, a);
		break;

	case ShapeType::Triangle: {
		const auto v0 = m_vertices[0] + m_position;
		const auto v1 = m_vertices[1] + m_position;
		const auto v2 = m_vertices[2] + m_position;
		aatrigonRGBA(renderer,
			static_cast<Sint16>(v0.x), static_cast<Sint16>(v0.y),
			static_cast<Sint16>(v1.x), static_cast<Sint16>(v1.y),
			static_cast<Sint16>(v2.x), static_cast<Sint16>(v2.y),
			r, g, b, a);
		break;
	}

	case ShapeType::Square: {
		const auto tl = m_vertices[0] + m_position;
		const auto br = m_vertices[2] + m_position;
		rectangleRGBA(renderer,
			static_cast<Sint16>(tl.x), static_cast<Sint16>(tl.y),
			static_cast<Sint16>(br.x), static_cast<Sint16>(br.y),
			r, g, b, a);
		break;
	}

	case ShapeType::Polygon: {
		const auto world = GetWorldVertices();
		const auto n     = static_cast<int>(world.size());
		if (n == 0) return;

		std::vector<Sint16> vx(n);
		std::vector<Sint16> vy(n);
		for (int i = 0; i < n; ++i) {
			vx[i] = static_cast<Sint16>(world[i].x);
			vy[i] = static_cast<Sint16>(world[i].y);
		}
		aapolygonRGBA(renderer, vx.data(), vy.data(), n, r, g, b, a);
		break;
	}

	case ShapeType::Freeform: {
		const auto world = GetWorldVertices();
		const auto n     = static_cast<int>(world.size());
		if (n < 2) return;

		for (int i = 1; i < n; ++i) {
			aalineRGBA(renderer,
				static_cast<Sint16>(world[i-1].x), static_cast<Sint16>(world[i-1].y),
				static_cast<Sint16>(world[i].x), static_cast<Sint16>(world[i].y),
				r, g, b, a);
		}
		break;
	}
	}
}

// Draw the object rotated by angleDegrees
void Object::DrawRotated(SDL_Renderer* renderer, float angleDegrees) const
{
	// For circles, rotation doesn't change appearance
	if (m_shapeType == ShapeType::Circle) {
		Draw(renderer);
		return;
	}

	// Red = colliding, Cyan = dynamic, Green = static
	Uint8 r, g, b;
	if (m_colliding)    { r = 255; g = 60;  b = 60;  }
	else if (m_dynamic) { r = 0;   g = 255; b = 255; }
	else                { r = 0;   g = 255; b = 0;   }
	constexpr Uint8 a = 255;

	// Get rotated vertices (negate angle for clockwise rotation)
	const float angleRad = -angleDegrees * static_cast<float>(std::numbers::pi) / 180.0f;
	const float cosA = std::cos(angleRad);
	const float sinA = std::sin(angleRad);

	std::vector<Vec2> rotatedVerts;
	rotatedVerts.reserve(m_vertices.size());
	for (const auto& v : m_vertices) {
		float x = v.x * cosA - v.y * sinA;
		float y = v.x * sinA + v.y * cosA;
		rotatedVerts.push_back({ x + m_position.x, y + m_position.y });
	}

	// Draw based on shape type
	switch (m_shapeType) {
	case ShapeType::Triangle:
		if (rotatedVerts.size() >= 3) {
			aatrigonRGBA(renderer,
				static_cast<Sint16>(rotatedVerts[0].x), static_cast<Sint16>(rotatedVerts[0].y),
				static_cast<Sint16>(rotatedVerts[1].x), static_cast<Sint16>(rotatedVerts[1].y),
				static_cast<Sint16>(rotatedVerts[2].x), static_cast<Sint16>(rotatedVerts[2].y),
				r, g, b, a);
		}
		break;

	case ShapeType::Square:
	case ShapeType::Polygon: {
		const auto n = static_cast<int>(rotatedVerts.size());
		if (n == 0) return;

		std::vector<Sint16> vx(n);
		std::vector<Sint16> vy(n);
		for (int i = 0; i < n; ++i) {
			vx[i] = static_cast<Sint16>(rotatedVerts[i].x);
			vy[i] = static_cast<Sint16>(rotatedVerts[i].y);
		}
		aapolygonRGBA(renderer, vx.data(), vy.data(), n, r, g, b, a);
		break;
	}

	case ShapeType::Freeform: {
		const auto n = static_cast<int>(rotatedVerts.size());
		if (n < 2) return;

		for (int i = 1; i < n; ++i) {
			aalineRGBA(renderer,
				static_cast<Sint16>(rotatedVerts[i-1].x), static_cast<Sint16>(rotatedVerts[i-1].y),
				static_cast<Sint16>(rotatedVerts[i].x), static_cast<Sint16>(rotatedVerts[i].y),
				r, g, b, a);
		}
		break;
	}

	default:
		break;
	}
}

// Draw the object rotated and translated (for C-Space preview)
void Object::DrawRotatedTranslated(SDL_Renderer* renderer, float angleDegrees, Vec2 offset) const
{
	// For circles, rotation doesn't change appearance, just translation
	if (m_shapeType == ShapeType::Circle) {
		// Semi-transparent white for preview
		aacircleRGBA(renderer,
			static_cast<Sint16>(m_position.x + offset.x),
			static_cast<Sint16>(m_position.y + offset.y),
			static_cast<Sint16>(m_radius),
			255, 255, 255, 180);
		return;
	}

	// Semi-transparent white for preview
	constexpr Uint8 r = 255, g = 255, b = 255, a = 180;

	// Get rotated vertices
	const float angleRad = -angleDegrees * static_cast<float>(std::numbers::pi) / 180.0f;
	const float cosA = std::cos(angleRad);
	const float sinA = std::sin(angleRad);

	std::vector<Vec2> rotatedVerts;
	rotatedVerts.reserve(m_vertices.size());
	for (const auto& v : m_vertices) {
		float x = v.x * cosA - v.y * sinA;
		float y = v.x * sinA + v.y * cosA;
		// Apply both position and translation offset
		rotatedVerts.push_back({ x + m_position.x + offset.x, y + m_position.y + offset.y });
	}

	// Draw based on shape type
	switch (m_shapeType) {
	case ShapeType::Triangle:
		if (rotatedVerts.size() >= 3) {
			aatrigonRGBA(renderer,
				static_cast<Sint16>(rotatedVerts[0].x), static_cast<Sint16>(rotatedVerts[0].y),
				static_cast<Sint16>(rotatedVerts[1].x), static_cast<Sint16>(rotatedVerts[1].y),
				static_cast<Sint16>(rotatedVerts[2].x), static_cast<Sint16>(rotatedVerts[2].y),
				r, g, b, a);
		}
		break;

	case ShapeType::Square:
	case ShapeType::Polygon: {
		const auto n = static_cast<int>(rotatedVerts.size());
		if (n == 0) return;

		std::vector<Sint16> vx(n);
		std::vector<Sint16> vy(n);
		for (int i = 0; i < n; ++i) {
			vx[i] = static_cast<Sint16>(rotatedVerts[i].x);
			vy[i] = static_cast<Sint16>(rotatedVerts[i].y);
		}
		aapolygonRGBA(renderer, vx.data(), vy.data(), n, r, g, b, a);
		break;
	}

	case ShapeType::Freeform: {
		const auto n = static_cast<int>(rotatedVerts.size());
		if (n < 2) return;

		for (int i = 1; i < n; ++i) {
			aalineRGBA(renderer,
				static_cast<Sint16>(rotatedVerts[i-1].x), static_cast<Sint16>(rotatedVerts[i-1].y),
				static_cast<Sint16>(rotatedVerts[i].x), static_cast<Sint16>(rotatedVerts[i].y),
				r, g, b, a);
		}
		break;
	}

	default:
		break;
	}
}
