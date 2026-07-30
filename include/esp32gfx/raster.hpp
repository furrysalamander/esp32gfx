#pragma once

#include "surface.hpp"
#include <cstdint>

namespace esp32gfx {

template<Pixel P>
void fill_triangle_gouraud(
    Surface<P>& surf,
    int16_t x0, int16_t y0, Color c0, int16_t z0, float iw0,
    int16_t x1, int16_t y1, Color c1, int16_t z1, float iw1,
    int16_t x2, int16_t y2, Color c2, int16_t z2, float iw2);

template<Pixel P>
inline void fill_triangle_flat(
    Surface<P>& surf,
    int16_t x0, int16_t y0,
    int16_t x1, int16_t y1,
    int16_t x2, int16_t y2,
    Color color,
    int16_t z0 = 0, int16_t z1 = 0, int16_t z2 = 0)
{
    fill_triangle_gouraud(surf, x0, y0, color, z0, 1.0f, x1, y1, color, z1, 1.0f, x2, y2, color, z2, 1.0f);
}

template<Pixel P>
void draw_line(
    Surface<P>& surf,
    int16_t x0, int16_t y0,
    int16_t x1, int16_t y1,
    Color color);

extern template void fill_triangle_gouraud(SurfaceRGBA32&, int16_t, int16_t, Color, int16_t, float, int16_t, int16_t, Color, int16_t, float, int16_t, int16_t, Color, int16_t, float);
extern template void fill_triangle_gouraud(SurfaceGray8&, int16_t, int16_t, Color, int16_t, float, int16_t, int16_t, Color, int16_t, float, int16_t, int16_t, Color, int16_t, float);
extern template void draw_line(SurfaceRGBA32&, int16_t, int16_t, int16_t, int16_t, Color);
extern template void draw_line(SurfaceGray8&, int16_t, int16_t, int16_t, int16_t, Color);

} // namespace esp32gfx
