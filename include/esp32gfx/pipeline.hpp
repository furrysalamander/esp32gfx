#pragma once

#include "math.hpp"
#include "mesh.hpp"
#include "surface.hpp"
#include <vector>

namespace esp32gfx {

void transform_and_project(
    const Vertex* verts, int n,
    ScreenVertex* out,
    const mat4f& model,
    const mat4f& view,
    const mat4f& proj,
    int vp_x, int vp_y, int vp_w, int vp_h,
    const vec3f& light_dir = vec3f(0, 0, 0));

template<Pixel P>
void draw_mesh(
    Surface<P>& surf,
    const ScreenVertex* verts,
    const Tri* tris, int nt,
    int vp_x, int vp_y, int vp_w, int vp_h);

extern template void draw_mesh(SurfaceRGBA32&, const ScreenVertex*, const Tri*, int, int, int, int, int);
extern template void draw_mesh(SurfaceGray8&, const ScreenVertex*, const Tri*, int, int, int, int, int);

} // namespace esp32gfx
