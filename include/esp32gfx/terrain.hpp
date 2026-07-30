#pragma once

#include "mesh.hpp"

namespace esp32gfx {

using HeightFn = float (*)(float wx, float wz);

void generate_terrain_grid(
    Mesh& mesh,
    float cx, float cz,
    float half_w, float half_d,
    int cols, int rows,
    HeightFn height_fn,
    Color base_color = Color(0, 0.5f, 0));

void generate_torus(
    Mesh& mesh,
    int cols, int rows,
    float major_radius, float minor_radius,
    Color base_color = Color(0.7f, 0.3f, 0.8f));

} // namespace esp32gfx
