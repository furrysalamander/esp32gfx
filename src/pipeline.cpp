#include "esp32gfx/pipeline.hpp"
#include "esp32gfx/raster.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace esp32gfx {

void transform_and_project(
    const Vertex* verts, int n,
    ScreenVertex* out,
    const mat4f& model,
    const mat4f& view,
    const mat4f& proj,
    int vp_x, int vp_y, int vp_w, int vp_h,
    const vec3f& light_dir)
{
    mat4f mv = view * model;
    mat4f mvp = proj * mv;

    float len = light_dir.x * light_dir.x + light_dir.y * light_dir.y + light_dir.z * light_dir.z;
    bool has_dynamic_light = len > 0;
    vec3f norm_light = has_dynamic_light ? light_dir * (1.0f / std::sqrt(len)) : light_dir;

    for (int i = 0; i < n; i++) {
        vec4f clip = mvp * vec4f(verts[i].pos.x, verts[i].pos.y, verts[i].pos.z, 1);

        out[i].clip_x = clip.x;
        out[i].clip_y = clip.y;
        out[i].clip_z = clip.z;
        out[i].clip_w = clip.w;

        // Dynamic lighting: transform normal to world space, compute ndotl
        if (has_dynamic_light && (verts[i].normal.x != 0 || verts[i].normal.y != 0 || verts[i].normal.z != 0)) {
            vec4f wn = model * vec4f(verts[i].normal.x, verts[i].normal.y, verts[i].normal.z, 0);
            vec3f world_norm = vec3f(wn.x, wn.y, wn.z).normalized();
            float ndotl = world_norm.dot(norm_light);
            float intensity = ndotl * 0.5f + 0.5f;
            out[i].color = verts[i].color * intensity;
        } else {
            out[i].color = verts[i].color;
        }

        // Cull vertices outside any frustum plane.
        if (clip.w <= 0 ||
            clip.z <= -clip.w || clip.z >= clip.w ||
            clip.x <= -clip.w || clip.x >= clip.w ||
            clip.y <= -clip.w || clip.y >= clip.w) {
            out[i].sx = -9999;
            out[i].sz = 0;
            continue;
        }

        float inv_w = 1.0f / clip.w;

        out[i].sx = int16_t(vp_x + vp_w * (clip.x * inv_w * 0.5f + 0.5f));
        out[i].sy = int16_t(vp_y + vp_h * (-clip.y * inv_w * 0.5f + 0.5f));
        out[i].sz = int16_t(std::clamp(clip.z * inv_w, -1.0f, 1.0f) * 32767.0f);
    }
}

// Interpolate a vertex along an edge that crosses the near plane.
// va is inside (sx != -9999), vb is outside (sx == -9999).
static ScreenVertex clip_edge_near(
    const ScreenVertex& va, const ScreenVertex& vb,
    int vp_x, int vp_y, int vp_w, int vp_h)
{
    // The near plane in clip space: clip.z + clip.w = 0
    float da = va.clip_z + va.clip_w; // >= 0 (inside)
    float db = vb.clip_z + vb.clip_w; // < 0 (outside)
    float t = da / (da - db);         // t in [0, 1]

    ScreenVertex v;
    v.clip_x = va.clip_x + t * (vb.clip_x - va.clip_x);
    v.clip_y = va.clip_y + t * (vb.clip_y - va.clip_y);
    v.clip_z = va.clip_z + t * (vb.clip_z - va.clip_z);
    v.clip_w = va.clip_w + t * (vb.clip_w - va.clip_w);
    v.color.r = va.color.r + t * (vb.color.r - va.color.r);
    v.color.g = va.color.g + t * (vb.color.g - va.color.g);
    v.color.b = va.color.b + t * (vb.color.b - va.color.b);

    float inv_w = 1.0f / v.clip_w;
    v.sx = int16_t(vp_x + vp_w * (v.clip_x * inv_w * 0.5f + 0.5f));
    v.sy = int16_t(vp_y + vp_h * (-v.clip_y * inv_w * 0.5f + 0.5f));
    v.sz = int16_t(v.clip_z * inv_w * 32767.0f);
    return v;
}

// Clip a triangle against the near plane. Returns number of output triangles
// (0, 1, or 2). Output vertices are written to out_verts starting at out_idx.
// Output triangle indices index into the out_verts array (not absolute).
struct ClipResult { int ntris; int nverts; };
static ClipResult clip_triangle_near(
    const ScreenVertex& v0, const ScreenVertex& v1, const ScreenVertex& v2,
    ScreenVertex* out_verts, int out_idx,
    int vp_x, int vp_y, int vp_w, int vp_h)
{
    auto outside = [](const ScreenVertex& v) { return v.sx == -9999 && v.clip_z + v.clip_w < 0; };
    bool o0 = outside(v0), o1 = outside(v1), o2 = outside(v2);

    // All inside: pass through
    if (!o0 && !o1 && !o2) {
        out_verts[0] = v0;
        out_verts[1] = v1;
        out_verts[2] = v2;
        return { 1, 3 };
    }

    // All outside: discard
    if (o0 && o1 && o2) return { 0, 0 };

    // 1 vertex inside, 2 outside → 1 output triangle
    if (!o0 && o1 && o2) {
        ScreenVertex p0 = clip_edge_near(v0, v1, vp_x, vp_y, vp_w, vp_h);
        ScreenVertex p1 = clip_edge_near(v0, v2, vp_x, vp_y, vp_w, vp_h);
        out_verts[0] = v0;
        out_verts[1] = p0;
        out_verts[2] = p1;
        return { 1, 3 };
    }
    if (o0 && !o1 && o2) {
        ScreenVertex p0 = clip_edge_near(v1, v0, vp_x, vp_y, vp_w, vp_h);
        ScreenVertex p1 = clip_edge_near(v1, v2, vp_x, vp_y, vp_w, vp_h);
        out_verts[0] = v1;
        out_verts[1] = p0;
        out_verts[2] = p1;
        return { 1, 3 };
    }
    if (o0 && o1 && !o2) {
        ScreenVertex p0 = clip_edge_near(v2, v0, vp_x, vp_y, vp_w, vp_h);
        ScreenVertex p1 = clip_edge_near(v2, v1, vp_x, vp_y, vp_w, vp_h);
        out_verts[0] = v2;
        out_verts[1] = p0;
        out_verts[2] = p1;
        return { 1, 3 };
    }

    // 2 vertices inside, 1 outside → 1 quad → 2 output triangles
    if (o0 && !o1 && !o2) {
        ScreenVertex p0 = clip_edge_near(v1, v0, vp_x, vp_y, vp_w, vp_h);
        ScreenVertex p1 = clip_edge_near(v2, v0, vp_x, vp_y, vp_w, vp_h);
        out_verts[0] = v1;
        out_verts[1] = p0;
        out_verts[2] = p1;
        out_verts[3] = v2;
        return { 2, 4 };
    }
    if (!o0 && o1 && !o2) {
        ScreenVertex p0 = clip_edge_near(v0, v1, vp_x, vp_y, vp_w, vp_h);
        ScreenVertex p1 = clip_edge_near(v2, v1, vp_x, vp_y, vp_w, vp_h);
        out_verts[0] = v0;
        out_verts[1] = p0;
        out_verts[2] = p1;
        out_verts[3] = v2;
        return { 2, 4 };
    }
    if (!o0 && !o1 && o2) {
        ScreenVertex p0 = clip_edge_near(v0, v2, vp_x, vp_y, vp_w, vp_h);
        ScreenVertex p1 = clip_edge_near(v1, v2, vp_x, vp_y, vp_w, vp_h);
        out_verts[0] = v0;
        out_verts[1] = p0;
        out_verts[2] = p1;
        out_verts[3] = v1;
        return { 2, 4 };
    }

    return { 0, 0 };
}

template<Pixel P>
void draw_mesh(
    Surface<P>& surf,
    const ScreenVertex* verts,
    const Tri* tris, int nt,
    int vp_x, int vp_y, int vp_w, int vp_h)
{
    surf.clear_depth();

    // Process each triangle: clip against near plane, then draw with Z-buffer
    for (int i = 0; i < nt; i++) {
        const auto& t = tris[i];
        const auto& v0 = verts[t[0]];
        const auto& v1 = verts[t[1]];
        const auto& v2 = verts[t[2]];

        // Vertex is culled specifically because it's behind the near plane
        auto near_culled = [](const ScreenVertex& v) {
            return v.sx == -9999 && v.clip_z + v.clip_w < 0;
        };

        // Guard band: skip triangles with vertices outside the frustum
        // that AREN'T near-plane straddlers (side/far-plane culled, or
        // valid vertices projecting to extreme screen coordinates).
        auto guard = [vp_w, vp_h, &near_culled](const ScreenVertex& v) {
            if (v.sx == -9999) return !near_culled(v);
            return v.sx < -vp_w || v.sx >= vp_w * 2 ||
                   v.sy < -vp_h || v.sy >= vp_h * 2;
        };
        if (guard(v0) || guard(v1) || guard(v2)) continue;

        // Check if any vertices need near-plane clipping
        auto outside = near_culled;
        if (!outside(v0) && !outside(v1) && !outside(v2)) {
            // All inside: draw directly
            fill_triangle_gouraud(surf,
                v0.sx, v0.sy, v0.color, v0.sz,
                v1.sx, v1.sy, v1.color, v1.sz,
                v2.sx, v2.sy, v2.color, v2.sz);
            continue;
        }

        // Some vertices outside near plane: clip
        ScreenVertex buf[4];
        auto r = clip_triangle_near(v0, v1, v2, buf, 0, vp_x, vp_y, vp_w, vp_h);
        if (r.ntris == 0) continue;

        // Guard band check for clipped vertices too
        {
            bool g = false;
            for (int j = 0; j < r.nverts; j++)
                if (buf[j].sx < -vp_w || buf[j].sx >= vp_w * 2 ||
                    buf[j].sy < -vp_h || buf[j].sy >= vp_h * 2)
                    { g = true; break; }
            if (g) continue;
        }

        // Draw first clipped triangle
        fill_triangle_gouraud(surf,
            buf[0].sx, buf[0].sy, buf[0].color, buf[0].sz,
            buf[1].sx, buf[1].sy, buf[1].color, buf[1].sz,
            buf[2].sx, buf[2].sy, buf[2].color, buf[2].sz);

        // Draw second clipped triangle if quad was split
        if (r.ntris == 2) {
            fill_triangle_gouraud(surf,
                buf[0].sx, buf[0].sy, buf[0].color, buf[0].sz,
                buf[2].sx, buf[2].sy, buf[2].color, buf[2].sz,
                buf[3].sx, buf[3].sy, buf[3].color, buf[3].sz);
        }
    }
}

template void draw_mesh(SurfaceRGBA32&, const ScreenVertex*, const Tri*, int, int, int, int, int);
template void draw_mesh(SurfaceGray8&, const ScreenVertex*, const Tri*, int, int, int, int, int);

} // namespace esp32gfx
