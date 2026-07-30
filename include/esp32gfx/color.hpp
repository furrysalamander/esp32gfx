#pragma once

#include <cstdint>
#include <algorithm>
#include <concepts>

namespace esp32gfx {

struct Color {
    float r{}, g{}, b{}, a{1};

    constexpr Color() = default;
    constexpr Color(float r, float g, float b, float a = 1) : r(r), g(g), b(b), a(a) {}

    static constexpr Color black()   { return {0, 0, 0}; }
    static constexpr Color white()   { return {1, 1, 1}; }
    static constexpr Color red()     { return {1, 0, 0}; }
    static constexpr Color green()   { return {0, 1, 0}; }
    static constexpr Color blue()    { return {0, 0, 1}; }
    static constexpr Color cyan()    { return {0, 1, 1}; }
    static constexpr Color yellow()  { return {1, 1, 0}; }
    static constexpr Color magenta() { return {1, 0, 1}; }
    static constexpr Color sky()     { return {0.53f, 0.81f, 0.92f}; }
    static constexpr Color grass()   { return {0.2f, 0.5f, 0.1f}; }
    static constexpr Color gray()    { return {0.5f, 0.5f, 0.5f}; }

    uint32_t to_rgba32() const {
        auto clamp = [](float v) -> uint8_t {
            return uint8_t(std::clamp(int(v * 255.0f), 0, 255));
        };
        return uint32_t(clamp(a)) << 24 |
               uint32_t(clamp(r)) << 16 |
               uint32_t(clamp(g)) << 8  |
               uint32_t(clamp(b));
    }

    static Color from_rgba32(uint32_t rgba) {
        return {
            ((rgba >> 16) & 0xFF) / 255.0f,
            ((rgba >> 8)  & 0xFF) / 255.0f,
            ( rgba        & 0xFF) / 255.0f,
            ((rgba >> 24) & 0xFF) / 255.0f
        };
    }

    float luminance() const {
        return r * 0.299f + g * 0.587f + b * 0.114f;
    }

    Color operator*(float s) const { return {r * s, g * s, b * s, a}; }
    Color operator+(Color c) const { return {r + c.r, g + c.g, b + c.b, a}; }
    Color operator*(Color c) const { return {r * c.r, g * c.g, b * c.b, a * c.a}; }
};

template<typename T>
concept Pixel = requires(T p, float f) {
    { T(0) };
};

template<Pixel P>
struct PixelTraits;

template<>
struct PixelTraits<uint32_t> {
    static uint32_t pack(const Color& c) { return c.to_rgba32(); }
    static Color unpack(uint32_t p) { return Color::from_rgba32(p); }
    static constexpr uint32_t black_value() { return 0xFF000000; }
    static constexpr int bit_depth() { return 32; }
};

template<>
struct PixelTraits<uint8_t> {
    static uint8_t pack(const Color& c) {
        return uint8_t(std::clamp(c.luminance(), 0.0f, 1.0f) * 255.0f);
    }
    static Color unpack(uint8_t p) {
        float g = p / 255.0f;
        return {g, g, g, 1};
    }
    static constexpr uint8_t black_value() { return 0; }
    static constexpr int bit_depth() { return 8; }
};

} // namespace esp32gfx
