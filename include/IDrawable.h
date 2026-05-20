#pragma once

struct SDL_Renderer;

// Interface for anything that can be drawn on screen.
// Any class that inherits from this must implement Draw().
class IDrawable
{
public:
    virtual ~IDrawable() = default;
    virtual void Draw(SDL_Renderer* renderer) const = 0;
};
