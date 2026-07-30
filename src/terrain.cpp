#include "esp32gfx/terrain.hpp"
#include <cmath>

namespace esp32gfx {

void generate_terrain_grid(
    Mesh& mesh,
    float cx, float cz,
    float half_w, float half_d,
    int cols, int rows,
    HeightFn height_fn,
    Color base_color)
{
    mesh.vertices.clear();
    mesh.triangles.clear();
    mesh.vertices.reserve(cols * rows);
    mesh.triangles.reserve((cols - 1) * (rows - 1) * 2);

    float eps = 0.5f;

    for (int zi = 0; zi < rows; zi++) {
        float tz = float(zi) / float(rows - 1);
        float wz = cz - half_d + tz * half_d * 2;

        for (int xi = 0; xi < cols; xi++) {
            float tx = float(xi) / float(cols - 1);
            float wx = cx - half_w + tx * half_w * 2;

            float h = height_fn(wx, wz);
            float hx = height_fn(wx + eps, wz);
            float hz = height_fn(wx, wz + eps);

            vec3f normal = vec3f(h - hx, eps, h - hz).normalized();
            mesh.vertices.push_back({{wx, h, wz}, base_color, normal});
        }
    }

    for (int zi = 0; zi < rows - 1; zi++) {
        for (int xi = 0; xi < cols - 1; xi++) {
            uint16_t i00 = uint16_t(zi * cols + xi);
            uint16_t i10 = uint16_t(zi * cols + xi + 1);
            uint16_t i01 = uint16_t((zi + 1) * cols + xi);
            uint16_t i11 = uint16_t((zi + 1) * cols + xi + 1);

            mesh.triangles.push_back({i00, i10, i11});
            mesh.triangles.push_back({i00, i11, i01});
        }
    }
}

static float torus_u(int i, int cols) {
    return float(i) / float(cols) * 2.0f * 3.14159265f;
}

static float torus_v(int j, int rows) {
    return float(j) / float(rows) * 2.0f * 3.14159265f;
}

void generate_torus(
    Mesh& mesh,
    int cols, int rows,
    float major_radius, float minor_radius,
    Color base_color)
{
    mesh.vertices.clear();
    mesh.triangles.clear();
    mesh.vertices.reserve((rows + 1) * (cols + 1));
    mesh.triangles.reserve(rows * cols * 2);

    for (int j = 0; j <= rows; j++) {
        float v = torus_v(j, rows);
        float cv = std::cos(v), sv = std::sin(v);

        for (int i = 0; i <= cols; i++) {
            float u = torus_u(i, cols);
            float cu = std::cos(u), su = std::sin(u);

            float Rrcv = major_radius + minor_radius * cv;

            vec3f pos(Rrcv * cu, minor_radius * sv, Rrcv * su);

            // Analytic normal: cross(dP/du, dP/dv)
            vec3f ddu(-Rrcv * su, 0, Rrcv * cu);
            vec3f ddv(-minor_radius * sv * cu, minor_radius * cv, -minor_radius * sv * su);
            vec3f normal = ddu.cross(ddv).normalized();

            mesh.vertices.push_back({pos, base_color, normal});
        }
    }

    for (int j = 0; j < rows; j++) {
        for (int i = 0; i < cols; i++) {
            uint16_t i00 = uint16_t(j * (cols + 1) + i);
            uint16_t i10 = uint16_t(j * (cols + 1) + i + 1);
            uint16_t i01 = uint16_t((j + 1) * (cols + 1) + i);
            uint16_t i11 = uint16_t((j + 1) * (cols + 1) + i + 1);

            mesh.triangles.push_back({i00, i10, i11});
            mesh.triangles.push_back({i00, i11, i01});
        }
    }
}

} // namespace esp32gfx
