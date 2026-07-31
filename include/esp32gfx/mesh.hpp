#pragma once

#include "math.hpp"
#include "color.hpp"
#include <array>
#include <vector>

namespace esp32gfx {

struct Vertex {
    vec3f pos;
    Color color;
    vec3f normal{0, 0, 0};
};

struct ScreenVertex {
    int32_t sx, sy;
    int16_t sz;
    Color color;
    float clip_x, clip_y, clip_z, clip_w;
};

using Tri = std::array<uint16_t, 3>;

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<Tri> triangles;
};

} // namespace esp32gfx
