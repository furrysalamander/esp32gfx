#pragma once

#include <cstdint>
#include <array>
#include <cstddef>
#include <cmath>
#include <limits>
#include <utility>

namespace esp32gfx {

namespace detail {

struct LCG {
    uint64_t state;
    constexpr explicit LCG(uint64_t seed) : state(seed) {}
    constexpr uint64_t next() {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return state;
    }
};

// Superfast void-and-cluster blue noise (Bart Wronski, 2021):
//   https://bartwronski.com/2021/04/21/superfast-void-and-cluster-blue-noise-in-python-numpy-jax/
// Seed a jittered grid, then repeatedly place a point at the lowest-energy
// empty pixel.  The energy field is the sum of toroidally-wrapped Gaussian
// bells; each placed point leaves +inf exactly at its center, so set pixels
// can never be re-picked and no bookkeeping is needed.
//
// Everything is accessed through raw pointers on std::array::data() because
// libstdc++ builds with _GLIBCXX_ASSERTIONS make operator[] non-constexpr.
template <std::size_t N>
struct BlueNoiseImpl {
    static constexpr std::size_t TOTAL = N * N;

    std::array<uint8_t, TOTAL> data{};

    constexpr BlueNoiseImpl(double sigma = 1.9, uint64_t seed = 42) {
        static_assert(N >= 2 && (N & (N - 1)) == 0, "N must be a power of two");

        // Seed points per dimension: 1 in 8 per axis, like the original.
        constexpr std::size_t SPPD = N >= 8 ? N / 8 : 1;
        constexpr std::size_t NUM_SEEDS = SPPD * SPPD;
        constexpr std::size_t BUCKET = N / SPPD;
        constexpr std::size_t MASK = N - 1;

        // Toroidally-wrapped Gaussian energy LUT, +inf at the center.
        std::array<double, TOTAL> lut{};
        std::array<double, N> gauss{};
        for (std::size_t i = 0; i < N; ++i) {
            double d = double(i < N / 2 ? i : N - i);
            gauss[i] = std::exp(-0.5 * d * d / (sigma * sigma));
        }
        for (std::size_t i = 0; i < N; ++i) {
            double* lut_row = lut.data() + i * N;
            double gi = gauss[i];
            for (std::size_t j = 0; j < N; ++j)
                lut_row[j] = gi * gauss[j];
        }
        lut[0] = std::numeric_limits<double>::infinity();

        // Jittered-grid seed points, shuffled into a random order.
        std::array<std::pair<std::size_t, std::size_t>, NUM_SEEDS> seeds{};
        LCG rng(seed);
        for (std::size_t x = 0; x < SPPD; ++x)
            for (std::size_t y = 0; y < SPPD; ++y)
                seeds[x * SPPD + y] = {
                    x * BUCKET + std::size_t(rng.next() % BUCKET),
                    y * BUCKET + std::size_t(rng.next() % BUCKET)};
        for (std::size_t i = NUM_SEEDS; i-- > 1;) {
            std::size_t j = std::size_t(rng.next() % (i + 1));
            std::swap(seeds[i], seeds[j]);
        }

        double* energy = energy_buf.data();
        double* lutp = lut.data();
        std::size_t* value = value_buf.data();

        auto add_energy = [&](std::size_t px, std::size_t py) {
            for (std::size_t i = 0; i < N; ++i) {
                std::size_t ii = (i + px) & MASK;
                double* e_row = energy + ii * N;
                double* l_row = lutp + i * N;
                for (std::size_t j = 0; j < N; ++j)
                    e_row[(j + py) & MASK] += l_row[j];
            }
        };

        std::size_t count = 0;
        for (std::size_t s = 0; s < NUM_SEEDS; ++s) {
            std::size_t sx = seeds[s].first, sy = seeds[s].second;
            value[sx * N + sy] = count++;
            add_energy(sx, sy);
        }

        // Place the remaining points at the smallest-energy empty pixel.
        for (; count < TOTAL; ++count) {
            std::size_t best = 0;
            double best_e = std::numeric_limits<double>::infinity();
            for (std::size_t k = 0; k < TOTAL; ++k)
                if (energy[k] < best_e) {
                    best_e = energy[k];
                    best = k;
                }
            value[best] = count;
            add_energy(best / N, best & MASK);
        }

        // Values are insertion ranks in [0, TOTAL): map onto [0, 255].
        uint8_t* dp = data.data();
        for (std::size_t i = 0; i < TOTAL; ++i)
            dp[i] = uint8_t(0.5 + 255.0 * double(value[i]) / double(TOTAL - 1));
    }

private:
    std::array<double, TOTAL> energy_buf{};
    std::array<std::size_t, TOTAL> value_buf{};
};

} // namespace detail

template <std::size_t Size>
struct BlueNoise {
    static_assert(Size > 0 && (Size & (Size - 1)) == 0, "Size must be power of two");
    static constexpr std::size_t N = Size;
    static constexpr std::size_t TOTAL = N * N;
    std::array<uint8_t, N * N> data{};

    constexpr BlueNoise() {
        detail::BlueNoiseImpl<N> impl;
        data = impl.data;
    }
};

using BlueNoise64 = BlueNoise<64>;
using BlueNoise128 = BlueNoise<128>;
using BlueNoise256 = BlueNoise<256>;

} // namespace esp32gfx
