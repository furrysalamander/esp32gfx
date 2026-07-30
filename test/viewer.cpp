#include "esp32gfx/esp32gfx.hpp"
#include <SDL2/SDL.h>
#include <cstdio>
#include <cmath>

using namespace esp32gfx;

// Screen-space tiled blue noise with camera-rotation offset.
// Each screen pixel maps 1:1 to a noise texel (crisp dots).
// The tile origin shifts as the camera rotates so the repeating
// 64×64 pattern doesn't stay visibly locked to the screen.
static constexpr uint8_t NOISE_TILE = 64;
static constexpr BlueNoise64 blue_noise;

static void apply_obra_dither(SurfaceGray8& surf, float fov_y,
                              float yaw_offset, float pitch_offset) {
    int w = surf.width(), h = surf.height();
    uint8_t* data = surf.data();

    int ox = int(w * yaw_offset / fov_y);
    int oy = int(h * pitch_offset * float(w) / float(h) / fov_y);

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int tx = (x - ox) & (NOISE_TILE - 1);
            int ty = (y - oy) & (NOISE_TILE - 1);
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
    bool dither_mode = false;
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
                    dither_mode = !dither_mode;
                    if (dither_mode) {
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
                    int valid = 0, invalid = 0, huge = 0;
                    for (auto& s : screen) {
                        if (s.sx == -9999) invalid++; else valid++;
                    }
                    fprintf(stderr, "=== DEBUG DUMP ===\n");
                    fprintf(stderr, "camera pos=(%.1f, %.1f, %.1f) yaw=%.2f pitch=%.2f\n",
                            cam.pos.x, cam.pos.y, cam.pos.z, cam.yaw, cam.pitch);
                    fprintf(stderr, "verts: %zu valid=%d invalid=%d\n", screen.size(), valid, invalid);
                    int nreport = 0;
                    for (size_t ti = 0; ti < mesh.triangles.size(); ti++) {
                        auto& t = mesh.triangles[ti];
                        auto& v0 = screen[t[0]];
                        auto& v1 = screen[t[1]];
                        auto& v2 = screen[t[2]];
                        if (v0.sx == -9999 || v1.sx == -9999 || v2.sx == -9999) continue;
                        int minx = std::min({v0.sx, v1.sx, v2.sx});
                        int maxx = std::max({v0.sx, v1.sx, v2.sx});
                        int miny = std::min({v0.sy, v1.sy, v2.sy});
                        int maxy = std::max({v0.sy, v1.sy, v2.sy});
                        if (minx < surf_w && maxx >= 0 && miny < surf_h && maxy >= 0 &&
                            (maxx - minx > surf_w / 2 || maxy - miny > surf_h / 2)) {
                            int32_t d = int32_t(v0.sz) + int32_t(v1.sz) + int32_t(v2.sz);
                            fprintf(stderr, "HUGE tri[%zu] sx=(%d,%d,%d) sy=(%d,%d,%d) sz=(%d,%d,%d) d=%d\n",
                                    ti, v0.sx, v1.sx, v2.sx, v0.sy, v1.sy, v2.sy,
                                    v0.sz, v1.sz, v2.sz, d);
                            if (++huge >= 5) break;
                        }
                    }
                    if (!huge) fprintf(stderr, "no huge triangles found\n");
                    fprintf(stderr, "==================\n");
                }
            }
        }

        torus_angle += dt * 0.8f;
        auto& cur_mesh = show_torus ? torus : mesh;
        mat4f model = show_torus ? mat4f::rotate(torus_angle, 0.0f, 1.0f, 0.0f) * mat4f::rotate(1.570796f, 1.0f, 0.0f, 0.0f) : mat4f::identity();

        if (dither_mode) {
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

            float dyaw = cam.yaw - yaw_base;
            float dpitch = cam.pitch - pitch_base;
            apply_obra_dither(gray_surf, 1.0f, dyaw, dpitch);

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

            SDL_UpdateTexture(texture, nullptr, gray_surf.data(), surf_w);
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
