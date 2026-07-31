#include "esp32gfx/esp32gfx.hpp"
#include <SDL2/SDL.h>
#include <cstdio>
#include <cmath>

using namespace esp32gfx;

static constexpr uint8_t NOISE_TILE = 64;
static constexpr BlueNoise64 blue_noise;

// Screen-space tiled blue noise offset by camera rotation.
// DitherOffset = ScreenSize * CameraRotation / CameraFov
static void apply_obra_dither(SurfaceGray8& surf, float fov_y,
                              float yaw_offset, float pitch_offset) {
    int w = surf.width(), h = surf.height();
    uint8_t* data = surf.data();

    float aspect = float(w) / float(h);
    float fov_x = 2.0f * std::atan(std::tan(fov_y * 0.5f) * aspect);

    int ox = int(w * yaw_offset / fov_x);
    int oy = int(h * pitch_offset / fov_y);

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int tx = (x - ox) & (NOISE_TILE - 1);
            int ty = (y - oy) & (NOISE_TILE - 1);
            data[y * w + x] = data[y * w + x] > blue_noise.data[ty][tx] ? 255 : 0;
        }
    }
}

// World-space sphere-mapped dither: the noise texture is looked up from a
// sphere centered at the camera, oriented to world space.  During camera
// rotation the pattern stays pinned to scene geometry.  The 64x64 noise
// tiles at ~screen pixel resolution: each radian of horizontal angle maps
// to w/fov_x texels, so the pattern wraps every 64px at screen center, but
// the tile origin is pinned to world directions.
static void apply_obra_dither_sphere(SurfaceGray8& surf, float fov_y,
                                     const mat4f& view) {
    int w = surf.width(), h = surf.height();
    uint8_t* data = surf.data();
    float tan_half_fov = std::tan(fov_y * 0.5f);
    float aspect = float(w) / float(h);
    float fov_x = 2.0f * std::atan(std::tan(fov_y * 0.5f) * aspect);

    for (int y = 0; y < h; y++) {
        float ndc_y = 1.0f - 2.0f * (float(y) + 0.5f) / float(h);
        float view_y = ndc_y * tan_half_fov;

        for (int x = 0; x < w; x++) {
            float ndc_x = 2.0f * (float(x) + 0.5f) / float(w) - 1.0f;
            float view_x = ndc_x * tan_half_fov * aspect;

            // view->world direction = transpose of the view rotation.
            // The view matrix is column-major: column 0 = right, column 1 = up,
            // column 2 = -forward, so (M^T * v) gives s.x*vx+u.x*vy+f.x, etc.
            float dx = view.m[0] * view_x + view.m[1] * view_y - view.m[2];
            float dy = view.m[4] * view_x + view.m[5] * view_y - view.m[6];
            float dz = view.m[8] * view_x + view.m[9] * view_y - view.m[10];

            float theta = std::atan2(dz, dx);
            float phi = std::asin(dy / std::sqrt(dx * dx + dy * dy + dz * dz));

            int tx = int(theta * float(w) / fov_x) & (NOISE_TILE - 1);
            int ty = int(phi * float(h) / fov_y) & (NOISE_TILE - 1);

            data[y * w + x] = data[y * w + x] > blue_noise.data[ty][tx] ? 255 : 0;
        }
    }
}

static float terrain_height(float wx, float wz) {
    float h = 0;
    h += std::sin(wx * 0.125f) * std::cos(wz * 0.125f + 0.83f) * 15.0f;
    h += std::sin(wx * 0.5f + 1.67f) * std::cos(wz * 0.5f + 3.11f) * 8.0f;
    h += std::sin(wx + 0.58f) * std::cos(wz + 2.70f) * 3.0f;
    h += 30;
    if (h < 0) h = 0;
    if (h > 60) h = 60;
    return h;
}

struct Camera {
    vec3f pos{0, 50, -120};
    float yaw = 0, pitch = -0.25f;
    float speed = 60.0f;
    float sens = 0.003f;

    vec3f forward() const {
        return {std::sin(yaw) * std::cos(pitch),
                std::sin(pitch),
                std::cos(yaw) * std::cos(pitch)};
    }

    vec3f right() const {
        return {std::cos(yaw), 0, -std::sin(yaw)};
    }

    mat4f view() const {
        return mat4f::look_at(pos, pos + forward(), {0, 1, 0});
    }
};

int main(int argc, char** argv) {
    int win_w = 800, win_h = 600;
    int surf_w = 800, surf_h = 600;

    for (int i = 1; i < argc; i++) {
        if (argv[i][0] == '-') {
            switch (argv[i][1]) {
            case 'w': win_w = std::atoi(argv[++i]); break;
            case 'h': win_h = std::atoi(argv[++i]); break;
            case 'r':
                surf_w = std::atoi(argv[++i]);
                surf_h = std::atoi(argv[++i]);
                break;
            }
        }
    }

    SDL_SetHint(SDL_HINT_VIDEODRIVER, "wayland,x11");
    if (SDL_Init(SDL_INIT_VIDEO) < 0) { return 1; }

    SDL_Window* window = SDL_CreateWindow(
        "esp32gfx", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        win_w, win_h, SDL_WINDOW_SHOWN);

    SDL_Renderer* renderer = SDL_CreateRenderer(
        window, -1, SDL_RENDERER_SOFTWARE);
    if (!renderer) {
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    }

    SDL_Texture* texture = SDL_CreateTexture(
        renderer, SDL_PIXELFORMAT_RGBA32,
        SDL_TEXTUREACCESS_STREAMING, surf_w, surf_h);

    SurfaceRGBA32 surface(surf_w, surf_h);
    Camera cam;
    Mesh mesh, torus;
    vec3f light_dir = vec3f(0.6f, 1.0f, 0.3f).normalized();
    generate_terrain_grid(mesh, 0, 300, 200, 400, 24, 18, terrain_height,
                          Color{0.2f, 0.5f, 0.12f});
    generate_torus(torus, 32, 20, 15, 6,
                   Color{0.8f, 0.3f, 0.2f});

    std::vector<ScreenVertex> screen(mesh.vertices.size());

    mat4f proj = mat4f::perspective(1.0f, float(surf_w) / float(surf_h), 5, 800);

    bool running = true;
    bool wireframe = false;
    bool gray_mode = false;
    enum DitherMode { DITHER_OFF, DITHER_SCREEN, DITHER_SPHERE };
    int dither_mode = DITHER_OFF;
    bool show_torus = false;
    float torus_angle = 0;
    float yaw_base = cam.yaw, pitch_base = cam.pitch;
    uint32_t last_time = SDL_GetTicks();
    int frames = 0, fps = 0;
    SDL_WarpMouseInWindow(window, win_w / 2, win_h / 2);
    SDL_SetRelativeMouseMode(SDL_TRUE);

    while (running) {
        uint32_t now = SDL_GetTicks();
        float dt = float(now - last_time) / 1000.0f;
        last_time = now;
        frames++;
        if (now % 1000 < 16) fps = frames, frames = 0;

        int mx, my;
        SDL_GetRelativeMouseState(&mx, &my);
        cam.yaw -= float(mx) * cam.sens;
        cam.pitch = std::clamp(cam.pitch + float(-my) * cam.sens, -1.4f, 1.4f);

        const uint8_t* keys = SDL_GetKeyboardState(nullptr);
        if (keys[SDL_SCANCODE_W]) cam.pos = cam.pos + cam.forward() * cam.speed * dt;
        if (keys[SDL_SCANCODE_S]) cam.pos = cam.pos - cam.forward() * cam.speed * dt;
        if (keys[SDL_SCANCODE_A]) cam.pos = cam.pos + cam.right() * cam.speed * dt;
        if (keys[SDL_SCANCODE_D]) cam.pos = cam.pos - cam.right() * cam.speed * dt;
        if (keys[SDL_SCANCODE_SPACE]) cam.pos.y += cam.speed * dt;
        if (keys[SDL_SCANCODE_LSHIFT]) cam.pos.y -= cam.speed * dt;
        if (keys[SDL_SCANCODE_ESCAPE]) running = false;

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            if (e.type == SDL_KEYDOWN) {
                if (e.key.keysym.sym == SDLK_TAB) wireframe = !wireframe;
                if (e.key.keysym.sym == SDLK_g) {
                    gray_mode = !gray_mode;
                    if (gray_mode) {
                        SDL_DestroyTexture(texture);
                        texture = SDL_CreateTexture(
                            renderer, SDL_PIXELFORMAT_RGBA32,
                            SDL_TEXTUREACCESS_STREAMING, surf_w, surf_h);
                    }
                }
                if (e.key.keysym.sym == SDLK_b) {
                    dither_mode = (dither_mode + 1) % 3;
                    if (dither_mode == DITHER_SCREEN || dither_mode == DITHER_SPHERE) {
                        SDL_DestroyTexture(texture);
                        texture = SDL_CreateTexture(
                            renderer, SDL_PIXELFORMAT_RGBA32,
                            SDL_TEXTUREACCESS_STREAMING, surf_w, surf_h);
                    }
                }
                if (e.key.keysym.sym == SDLK_t) {
                    show_torus = !show_torus;
                    screen.clear();
                    auto& m = show_torus ? torus : mesh;
                    screen.resize(m.vertices.size(), ScreenVertex{});
                }
                if (e.key.keysym.sym == SDLK_F1) {
                    auto& cur_mesh = show_torus ? torus : mesh;
                    int valid = 0, invalid = 0;
                    for (auto& s : screen) {
                        if (s.sx == -9999) invalid++; else valid++;
                    }

                    fprintf(stderr, "=== DEBUG DUMP ===\n");
                    fprintf(stderr, "camera: pos=(%.1f,%.1f,%.1f) yaw=%.2f pitch=%.2f\n",
                            cam.pos.x, cam.pos.y, cam.pos.z, cam.yaw, cam.pitch);
                    fprintf(stderr, "frustum: fov_y=%.2f near=%.0f far=%.0f aspect=%.3f model=%s\n",
                            1.0f, 5.0f, 800.0f, float(surf_w)/float(surf_h),
                            show_torus ? "torus" : "terrain");
                    fprintf(stderr, "viewport: w=%d h=%d\n", surf_w, surf_h);
                    fprintf(stderr, "mesh: nverts=%zu valid=%d invalid=%d ntris=%zu\n",
                            screen.size(), valid, invalid, cur_mesh.triangles.size());

                    // Triangle classification + detailed dump for interesting triangles
                    int all_in_cnt = 0, partial_cnt = 0, all_out_cnt = 0;
                    int on_screen_cnt = 0, huge_cnt = 0, large_z_cnt = 0, tiny_w_cnt = 0, huge_printed = 0;

                    // First pass: classify and collect interesting triangles
                    struct TriDump { size_t idx; int flags; }; // flags: 1=partial, 2=huge, 4=large_z, 8=tiny_w
                    std::vector<TriDump> interesting;

                    for (size_t ti = 0; ti < cur_mesh.triangles.size(); ti++) {
                        auto& t = cur_mesh.triangles[ti];
                        if (t[0] >= screen.size() || t[1] >= screen.size() || t[2] >= screen.size())
                            continue;
                        auto& v0 = screen[t[0]];
                        auto& v1 = screen[t[1]];
                        auto& v2 = screen[t[2]];

                        bool o0 = v0.sx == -9999, o1 = v1.sx == -9999, o2 = v2.sx == -9999;
                        int n_out = (o0?1:0) + (o1?1:0) + (o2?1:0);

                        if (n_out == 0) all_in_cnt++;
                        else if (n_out == 3) all_out_cnt++;
                        else partial_cnt++;

                        if (n_out > 0 && n_out < 3) continue; // we want ALL partial clip triangles

                        if (n_out == 3) continue;

                        // All vertices inside: check for anomalous properties
                        int32_t vsx[3] = {v0.sx, v1.sx, v2.sx};
                        int32_t vsy[3] = {v0.sy, v1.sy, v2.sy};
                        int32_t vsz[3] = {v0.sz, v1.sz, v2.sz};

                        int minx = std::min({vsx[0], vsx[1], vsx[2]});
                        int maxx = std::max({vsx[0], vsx[1], vsx[2]});
                        int miny = std::min({vsy[0], vsy[1], vsy[2]});
                        int maxy = std::max({vsy[0], vsy[1], vsy[2]});
                        int minz = std::min({vsz[0], vsz[1], vsz[2]});
                        int maxz = std::max({vsz[0], vsz[1], vsz[2]});

                        bool on_scr = minx < surf_w && maxx >= 0 && miny < surf_h && maxy >= 0;
                        bool huge = on_scr && (maxx - minx > surf_w / 2 || maxy - miny > surf_h / 2);
                        bool lz = maxz - minz > 20000;
                        bool tw = (v0.clip_w < 12 || v1.clip_w < 12 || v2.clip_w < 12);

                        int flags = 0;
                        if (huge) flags |= 2;
                        if (lz)   flags |= 4;
                        if (tw)   flags |= 8;
                        if (on_scr) on_screen_cnt++;
                        if (huge) huge_cnt++;
                        if (lz) large_z_cnt++;
                        if (tw) tiny_w_cnt++;

                        if (flags) interesting.push_back({ti, flags});
                    }

                    // Second pass: dump partial-clip triangles (all of them)
                    fprintf(stderr, "\n-- triangle summary --\n");
                    fprintf(stderr, "all_in=%d partial=%d all_out=%d\n", all_in_cnt, partial_cnt, all_out_cnt);
                    fprintf(stderr, "on_screen=%d huge=%d large_z=%d tiny_w=%d\n",
                            on_screen_cnt, huge_cnt, large_z_cnt, tiny_w_cnt);
                    fprintf(stderr, "interesting_tris=%zu\n", interesting.size());

                    if (partial_cnt > 0) {
                        fprintf(stderr, "\n-- partial clip --\n");
                        for (size_t ti = 0; ti < cur_mesh.triangles.size(); ti++) {
                            auto& t = cur_mesh.triangles[ti];
                            auto& v0 = screen[t[0]];
                            auto& v1 = screen[t[1]];
                            auto& v2 = screen[t[2]];
                            bool o0 = v0.sx == -9999, o1 = v1.sx == -9999, o2 = v2.sx == -9999;
                            int n_out = (o0?1:0) + (o1?1:0) + (o2?1:0);
                            if (n_out == 0 || n_out == 3) continue;

                            int n_in = 3 - n_out;
                            int32_t vsz[3] = {v0.sz, v1.sz, v2.sz};
                            int minz = std::min({vsz[0], vsz[1], vsz[2]});
                            int maxz = std::max({vsz[0], vsz[1], vsz[2]});

                            fprintf(stderr, "tri[%zu]: out=%d in=%d z_range=%d\n", ti, n_out, n_in, maxz - minz);
                            auto pv = [](const ScreenVertex& v) {
                                if (v.sx == -9999)
                                    fprintf(stderr, "  CULL clip=(%.2f,%.2f,%.2f,%.2f)\n",
                                            v.clip_x, v.clip_y, v.clip_z, v.clip_w);
                                else
                                    fprintf(stderr, "  OK sx=%d sy=%d sz=%d clip=(%.2f,%.2f,%.2f,%.2f)\n",
                                            v.sx, v.sy, v.sz, v.clip_x, v.clip_y, v.clip_z, v.clip_w);
                            };
                            pv(v0); pv(v1); pv(v2);

                            // Simulate clipping to show what draw_mesh will produce
                            auto clip_edge = [&](const ScreenVertex& va, const ScreenVertex& vb) {
                                float da = va.clip_z + va.clip_w;
                                float db = vb.clip_z + vb.clip_w;
                                float t = da / (da - db);
                                ScreenVertex out;
                                out.clip_x = va.clip_x + t * (vb.clip_x - va.clip_x);
                                out.clip_y = va.clip_y + t * (vb.clip_y - va.clip_y);
                                out.clip_z = va.clip_z + t * (vb.clip_z - va.clip_z);
                                out.clip_w = va.clip_w + t * (vb.clip_w - va.clip_w);
                                float inv_w = 1.0f / out.clip_w;
                                out.sx = int32_t(out.clip_x * inv_w * 0.5f * surf_w + surf_w * 0.5f);
                                out.sy = int32_t(-out.clip_y * inv_w * 0.5f * surf_h + surf_h * 0.5f);
                                out.sz = int16_t(std::clamp(out.clip_z * inv_w, -1.0f, 1.0f) * 32767.0f);
                                return out;
                            };

                            // 1 vertex inside, 2 outside
                            auto dump_clip_1in = [&](const ScreenVertex& inside,
                                                      const ScreenVertex& out1,
                                                      const ScreenVertex& out2) {
                                auto p = clip_edge(inside, out1);
                                auto q = clip_edge(inside, out2);
                                fprintf(stderr, "  clip1in: p=(%d,%d,%d) q=(%d,%d,%d)\n",
                                        p.sx, p.sy, p.sz, q.sx, q.sy, q.sz);
                            };
                            // 2 vertices inside, 1 outside
                            auto dump_clip_2in = [&](const ScreenVertex& in1,
                                                      const ScreenVertex& in2,
                                                      const ScreenVertex& out) {
                                auto p = clip_edge(in1, out);
                                auto q = clip_edge(in2, out);
                                fprintf(stderr, "  clip2in: p=(%d,%d,%d) q=(%d,%d,%d)\n",
                                        p.sx, p.sy, p.sz, q.sx, q.sy, q.sz);
                            };

                            if (n_in == 1 && n_out == 2) {
                                if (!o0) dump_clip_1in(v0, v1, v2);
                                else if (!o1) dump_clip_1in(v1, v0, v2);
                                else dump_clip_1in(v2, v0, v1);
                            } else {
                                if (o0) dump_clip_2in(v1, v2, v0);
                                else if (o1) dump_clip_2in(v0, v2, v1);
                                else dump_clip_2in(v0, v1, v2);
                            }
                        }
                    }

                    if (!interesting.empty()) {
                        fprintf(stderr, "\n-- anomalous all-inside triangles --\n");
                        for (auto& d : interesting) {
                            auto& t = cur_mesh.triangles[d.idx];
                            auto& v0 = screen[t[0]];
                            auto& v1 = screen[t[1]];
                            auto& v2 = screen[t[2]];
                            int32_t vsz[3] = {v0.sz, v1.sz, v2.sz};
                            int minz = std::min({vsz[0], vsz[1], vsz[2]});
                            int maxz = std::max({vsz[0], vsz[1], vsz[2]});
                            fprintf(stderr, "tri[%zu] flg=%d sx=(%d,%d,%d) sy=(%d,%d,%d) sz=(%d,%d,%d) zr=%d clip_w=(%.1f,%.1f,%.1f)\n",
                                    d.idx, d.flags,
                                    v0.sx, v1.sx, v2.sx,
                                    v0.sy, v1.sy, v2.sy,
                                    v0.sz, v1.sz, v2.sz,
                                    maxz - minz,
                                    v0.clip_w, v1.clip_w, v2.clip_w);
                        }
                    }

                    fprintf(stderr, "=== END ===\n");
                }
            }
        }

        torus_angle += dt * 0.8f;
        auto& cur_mesh = show_torus ? torus : mesh;
        mat4f model = show_torus ? mat4f::rotate(torus_angle, 0.0f, 1.0f, 0.0f) * mat4f::rotate(1.570796f, 1.0f, 0.0f, 0.0f) : mat4f::identity();

        if (dither_mode != DITHER_OFF) {
            SurfaceGray8 gray_surf(surf_w, surf_h);
            gray_surf.clear(Color::black());

            transform_and_project(cur_mesh.vertices.data(), (int)cur_mesh.vertices.size(),
                                  screen.data(), model, cam.view(), proj,
                                  0, 0, surf_w, surf_h, light_dir);

            if (wireframe) {
                for (const auto& t : cur_mesh.triangles) {
                    const auto& v0 = screen[t[0]];
                    const auto& v1 = screen[t[1]];
                    const auto& v2 = screen[t[2]];
                    draw_line(gray_surf, v0.sx, v0.sy, v1.sx, v1.sy, Color::white());
                    draw_line(gray_surf, v1.sx, v1.sy, v2.sx, v2.sy, Color::white());
                    draw_line(gray_surf, v2.sx, v2.sy, v0.sx, v0.sy, Color::white());
                }
            } else {
                draw_mesh(gray_surf, screen.data(), cur_mesh.triangles.data(),
                          (int)cur_mesh.triangles.size(), 0, 0, surf_w, surf_h);
            }

            if (dither_mode == DITHER_SCREEN) {
                float dyaw = cam.yaw - yaw_base;
                float dpitch = cam.pitch - pitch_base;
                apply_obra_dither(gray_surf, 1.0f, dyaw, dpitch);
            } else {
                apply_obra_dither_sphere(gray_surf, 1.0f, cam.view());
            }

            uint8_t* src = gray_surf.data();
            for (int i = 0; i < surf_w * surf_h; i++)
                surface.data()[i] = src[i] ? 0xFFFFFFFF : 0xFF000000;

            SDL_UpdateTexture(texture, nullptr, surface.data(), surf_w * 4);
        } else if (gray_mode) {
            SurfaceGray8 gray_surf(surf_w, surf_h);
            gray_surf.clear(Color::sky());

            transform_and_project(cur_mesh.vertices.data(), (int)cur_mesh.vertices.size(),
                                  screen.data(), model, cam.view(), proj,
                                  0, 0, surf_w, surf_h, light_dir);

            if (wireframe) {
                for (const auto& t : cur_mesh.triangles) {
                    const auto& v0 = screen[t[0]];
                    const auto& v1 = screen[t[1]];
                    const auto& v2 = screen[t[2]];
                    draw_line(gray_surf, v0.sx, v0.sy, v1.sx, v1.sy, Color::white());
                    draw_line(gray_surf, v1.sx, v1.sy, v2.sx, v2.sy, Color::white());
                    draw_line(gray_surf, v2.sx, v2.sy, v0.sx, v0.sy, Color::white());
                }
            } else {
                draw_mesh(gray_surf, screen.data(), cur_mesh.triangles.data(), (int)cur_mesh.triangles.size(), 0, 0, surf_w, surf_h);
            }

            uint8_t* src = gray_surf.data();
            uint32_t* dst = surface.data();
            for (int i = 0; i < surf_w * surf_h; i++) {
                uint8_t v = src[i];
                dst[i] = 0xFF000000 | (v << 16) | (v << 8) | v;
            }
            SDL_UpdateTexture(texture, nullptr, surface.data(), surf_w * 4);
        } else {
            surface.clear(Color::sky());

            transform_and_project(cur_mesh.vertices.data(), (int)cur_mesh.vertices.size(),
                                  screen.data(), model, cam.view(), proj,
                                  0, 0, surf_w, surf_h, light_dir);

            if (wireframe) {
                for (const auto& t : cur_mesh.triangles) {
                    const auto& v0 = screen[t[0]];
                    const auto& v1 = screen[t[1]];
                    const auto& v2 = screen[t[2]];
                    draw_line(surface, v0.sx, v0.sy, v1.sx, v1.sy, Color::white());
                    draw_line(surface, v1.sx, v1.sy, v2.sx, v2.sy, Color::white());
                    draw_line(surface, v2.sx, v2.sy, v0.sx, v0.sy, Color::white());
                }
            } else {
                draw_mesh(surface, screen.data(), cur_mesh.triangles.data(), (int)cur_mesh.triangles.size(), 0, 0, surf_w, surf_h);
            }

            SDL_UpdateTexture(texture, nullptr, surface.data(), surf_w * 4);
        }

        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    SDL_SetRelativeMouseMode(SDL_FALSE);
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
