#include "esp32gfx/raster.hpp"

namespace esp32gfx {

template<Pixel P>
void fill_triangle_gouraud(
    Surface<P>& surf,
    int16_t x0, int16_t y0, Color c0, int16_t z0,
    int16_t x1, int16_t y1, Color c1, int16_t z1,
    int16_t x2, int16_t y2, Color c2, int16_t z2)
{
    auto swp = [](int16_t& a, int16_t& b) { int16_t t = a; a = b; b = t; };
    auto swc = [](Color& a, Color& b) { Color t = a; a = b; b = t; };
    auto swz = [](int16_t& a, int16_t& b) { int16_t t = a; a = b; b = t; };

    if (y0 > y1) { swp(x0, x1); swp(y0, y1); swc(c0, c1); swz(z0, z1); }
    if (y0 > y2) { swp(x0, x2); swp(y0, y2); swc(c0, c2); swz(z0, z2); }
    if (y1 > y2) { swp(x1, x2); swp(y1, y2); swc(c1, c2); swz(z1, z2); }

    int h = surf.height();
    if (y2 < 0 || y0 >= h) return;

    int pre_step = y0 < 0 ? -y0 : 0;
    int ys = y0 + pre_step;

    auto slope = [](int16_t xa, int16_t ya, int16_t xb, int16_t yb) {
        int d = yb - ya;
        return d > 0 ? float(xb - xa) / float(d) : 0.0f;
    };

    auto cslp = [](float ca, int ya, float cb, int yb) {
        int d = yb - ya;
        return d > 0 ? (cb - ca) / float(d) : 0.0f;
    };

    float dx02 = slope(x0, y0, x2, y2);
    float dx01 = slope(x0, y0, x1, y1);
    float dx12 = slope(x1, y1, x2, y2);

    float z02_s = cslp(float(z0), y0, float(z2), y2);
    float z01_s = cslp(float(z0), y0, float(z1), y1);
    float z12_s = cslp(float(z1), y1, float(z2), y2);

    float c02_r = cslp(c0.r, y0, c2.r, y2);
    float c02_g = cslp(c0.g, y0, c2.g, y2);
    float c02_b = cslp(c0.b, y0, c2.b, y2);
    float c01_r = cslp(c0.r, y0, c1.r, y1);
    float c01_g = cslp(c0.g, y0, c1.g, y1);
    float c01_b = cslp(c0.b, y0, c1.b, y1);
    float c12_r = cslp(c1.r, y1, c2.r, y2);
    float c12_g = cslp(c1.g, y1, c2.g, y2);
    float c12_b = cslp(c1.b, y1, c2.b, y2);

    float ex02 = float(x0) + dx02 * float(pre_step);
    float ex01 = float(x0) + dx01 * float(pre_step);
    float ez02 = float(z0) + z02_s * float(pre_step);
    float ez01 = float(z0) + z01_s * float(pre_step);
    float ec02_r = c0.r + c02_r * float(pre_step);
    float ec02_g = c0.g + c02_g * float(pre_step);
    float ec02_b = c0.b + c02_b * float(pre_step);
    float ec01_r = c0.r + c01_r * float(pre_step);
    float ec01_g = c0.g + c01_g * float(pre_step);
    float ec01_b = c0.b + c01_b * float(pre_step);

    auto fill_span = [&](int y, int xs, int xe,
                         float cl_r, float cl_g, float cl_b, float zl,
                         float cr_r, float cr_g, float cr_b, float zr)
    {
        if (y < 0 || y >= h) return;
        int w = surf.width();

        if (xs < 0 && xe > xs) {
            float t = float(-xs) / float(xe - xs);
            cl_r += (cr_r - cl_r) * t;
            cl_g += (cr_g - cl_g) * t;
            cl_b += (cr_b - cl_b) * t;
            zl += (zr - zl) * t;
            xs = 0;
        }
        if (xe >= w) xe = w - 1;
        if (xs > xe) return;

        int dx = xe - xs;
        float dr = dx > 0 ? (cr_r - cl_r) / float(dx) : 0;
        float dg = dx > 0 ? (cr_g - cl_g) / float(dx) : 0;
        float db = dx > 0 ? (cr_b - cl_b) / float(dx) : 0;
        float dz = dx > 0 ? (zr - zl) / float(dx) : 0;

        float cr = cl_r, cg = cl_g, cb = cl_b, cz = zl;
        for (int x = xs; x <= xe; x++) {
            surf.pixel(x, y, {cr, cg, cb}, int16_t(cz));
            cr += dr; cg += dg; cb += db; cz += dz;
        }
    };

    int end_top = y1;
    if (end_top >= h) end_top = h - 1;

    if (y0 == y1 && ys <= end_top) {
        int xs = x0, xe = x1;
        if (xs > xe) { int t = xs; xs = xe; xe = t; }
        fill_span(ys, xs, xe, c0.r, c0.g, c0.b, float(z0), c1.r, c1.g, c1.b, float(z1));
        ex02 += dx02;
        ez02 += z02_s;
        ec02_r += c02_r; ec02_g += c02_g; ec02_b += c02_b;
    } else {
        for (int y = ys; y <= end_top; y++) {
            int xs = int(ex01), xe = int(ex02);
            if (xs > xe) {
                int t = xs; xs = xe; xe = t;
                fill_span(y, xs, xe,
                          ec02_r, ec02_g, ec02_b, ez02,
                          ec01_r, ec01_g, ec01_b, ez01);
            } else {
                fill_span(y, xs, xe,
                          ec01_r, ec01_g, ec01_b, ez01,
                          ec02_r, ec02_g, ec02_b, ez02);
            }
            ex01 += dx01; ex02 += dx02;
            ez01 += z01_s; ez02 += z02_s;
            ec01_r += c01_r; ec01_g += c01_g; ec01_b += c01_b;
            ec02_r += c02_r; ec02_g += c02_g; ec02_b += c02_b;
        }
    }

    float ex12 = float(x1);
    float ez12 = float(z1);
    float ec12_r = c1.r, ec12_g = c1.g, ec12_b = c1.b;
    if (y1 < 0) {
        float t = float(-y1);
        ex12 += dx12 * t;
        ez12 += z12_s * t;
        ec12_r += c12_r * t; ec12_g += c12_g * t; ec12_b += c12_b * t;
    } else if (y1 < h) {
        ex12 += dx12;
        ez12 += z12_s;
        ec12_r += c12_r; ec12_g += c12_g; ec12_b += c12_b;
    }

    int start_bot = y1 + 1;
    if (start_bot < 0) start_bot = 0;
    int end_bot = y2;
    if (end_bot >= h) end_bot = h - 1;

    for (int y = start_bot; y <= end_bot; y++) {
        int xs = int(ex12), xe = int(ex02);
        if (xs > xe) {
            int t = xs; xs = xe; xe = t;
            fill_span(y, xs, xe,
                      ec02_r, ec02_g, ec02_b, ez02,
                      ec12_r, ec12_g, ec12_b, ez12);
        } else {
            fill_span(y, xs, xe,
                      ec12_r, ec12_g, ec12_b, ez12,
                      ec02_r, ec02_g, ec02_b, ez02);
        }
        ex12 += dx12; ex02 += dx02;
        ez12 += z12_s; ez02 += z02_s;
        ec12_r += c12_r; ec12_g += c12_g; ec12_b += c12_b;
        ec02_r += c02_r; ec02_g += c02_g; ec02_b += c02_b;
    }
}

template void fill_triangle_gouraud(SurfaceRGBA32&, int16_t, int16_t, Color, int16_t, int16_t, int16_t, Color, int16_t, int16_t, int16_t, Color, int16_t);
template void fill_triangle_gouraud(SurfaceGray8&, int16_t, int16_t, Color, int16_t, int16_t, int16_t, Color, int16_t, int16_t, int16_t, Color, int16_t);

template<Pixel P>
void draw_line(Surface<P>& surf,
               int16_t x0, int16_t y0,
               int16_t x1, int16_t y1,
               Color color)
{
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        surf.pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx)  { err += dx; y0 += sy; }
    }
}

template void draw_line(SurfaceRGBA32&, int16_t, int16_t, int16_t, int16_t, Color);
template void draw_line(SurfaceGray8&, int16_t, int16_t, int16_t, int16_t, Color);

} // namespace esp32gfx
