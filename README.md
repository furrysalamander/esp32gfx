# esp32gfx

Software rasterization for microcontrollers. Per-pixel Z-buffer, Gouraud shading,
near-plane clipping, fixed-function MVP pipeline. Targets ESP32-S3. The desktop
viewer builds with SDL2 for development.

```sh
cmake -B build -S .
cmake --build build
./build/test/esp32gfx_viewer
```

Debian: `apt install libsdl2-dev`. Arch: `pacman -S sdl2`.

| Key | Action |
|-----|--------|
| WASD | Move |
| Space / Shift | Up / down |
| Mouse | Look |
| Tab | Wireframe |
| G | Grayscale |
| T | Terrain / torus |
| Escape | Quit |
| F1 | Debug dump |

## Pipeline

Vertices have a position, a base color, and a surface normal. On projection the
pipeline rotates the normal by the model matrix and dots it with the light
direction. The result multiplies the base color and lands in the screen vertex.

Lighting is not baked at mesh creation time. Pass `light_dir = (0,0,0)` and the
pipeline uses vertex colors as-is.

Triangles that straddle the near plane get split into screen-space triangles,
not culled. `clip_triangle_near` handles all six cases with a fixed 4-vertex
stack buffer. No allocation.

Every pixel write checks depth. `clear_depth` resets the buffer each frame.
Draw order does not matter.

## API

### Math

`vec3<T>`, `vec4<T>`, `mat4<T>`, `quat<T>`. Right-handed Y-up, looking down
−Z (OpenGL convention).

```cpp
mat4f::perspective(fov_y, aspect, near, far);
mat4f::look_at(eye, target, up);
mat4f::rotate(angle, ax, ay, az) * mat4f::translate(x, y, z);
```

### Surface

Row-major pixel buffer with a parallel depth buffer. `PixelTraits` lets the same
rasterizer work on `uint32_t` (RGBA) and `uint8_t` (grayscale).

```cpp
surface.clear(Color::sky());
surface.clear_depth();
surface.pixel(x, y, color, z);
```

### Rasterizer

One triangle function. Gouraud with Z interpolation.

```cpp
fill_triangle_gouraud<P>(surface, x0, y0, z0, c0,
                                   x1, y1, z1, c1,
                                   x2, y2, z2, c2);
```

Plus `draw_line<P>` — Bresenham.

### Pipeline

```cpp
transform_and_project(verts, n, screen, model, view, proj,
                      vp_x, vp_y, vp_w, vp_h, light_dir);

draw_mesh<P>(surface, screen, tris, nt, vp_x, vp_y, vp_w, vp_h);
```

### Terrain

Two mesh generators. Both write `Vertex` records with position, base color, and
surface normal.

```cpp
generate_terrain_grid(mesh, cx, cz, half_w, half_d, cols, rows,
                      height_fn, base_color);

generate_torus(mesh, cols, rows, major_r, minor_r, base_color);
```

## What It Does Not Do

- No texture mapping. No UV coordinates. No samplers.
- No fragment shaders. No programmable pipeline.
- No dynamic memory in the render loop.
- No painter's algorithm. The Z-buffer is mandatory.
- No GL, Vulkan, or Metal. Writes pixels to memory.

## Design

The rasterizer is nested integer loops over span bounds. `clip_triangle_near`
uses a fixed 4-vertex buffer. The Z-buffer is a pre-allocated `std::vector<int16_t>`
alongside the pixel buffer. No allocation in the hot path.

Far-plane and screen-edge vertices project normally. The rasterizer and Z-buffer
handle extents. `transform_and_project` only marks vertices behind the near plane
(`sx = -9999`).

`ScreenVertex` carries a `Color`. The rasterizer interpolates it as floats across
spans. No texture coordinates, no UVs, no samplers.

## Project Structure

```
esp32gfx/
├── CMakeLists.txt              # Static library + optional test viewer
├── include/esp32gfx/
│   ├── math.hpp                # vec3, vec4, mat4, quat
│   ├── color.hpp               # Color, PixelTraits<uint32_t/uint8_t>
│   ├── surface.hpp             # Surface<P> with depth buffer
│   ├── mesh.hpp                # Vertex, ScreenVertex, Tri, Mesh
│   ├── raster.hpp              # fill_triangle_gouraud, draw_line
│   ├── pipeline.hpp            # transform_and_project, draw_mesh
│   └── terrain.hpp             # generate_terrain_grid, generate_torus
├── src/
│   ├── raster.cpp              # Scanline Gouraud rasterizer
│   ├── pipeline.cpp            # Clipping, Z-buffer draw, clip helpers
│   └── terrain.cpp             # Mesh generators
└── test/
    └── viewer.cpp              # SDL2 interactive viewer
```

## Requirements

C++23. SDL2 for the desktop viewer (optional). The library has no dependency
other than the standard library.

## License

MIT
