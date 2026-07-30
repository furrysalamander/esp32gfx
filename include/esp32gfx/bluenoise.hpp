#pragma once

#include <cstdint>
#include <array>
#include <cstddef>

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

template <std::size_t N>
struct BlueNoiseImpl {
    static constexpr std::size_t TOTAL = N * N;

    std::array<uint8_t, TOTAL> data{};

    constexpr void generate() {
        LCG rng(42);
        std::array<uint64_t, TOTAL> vals{};

        uint64_t* vp = vals.data();
        for (std::size_t i = 0; i < TOTAL; ++i)
            vp[i] = rng.next();

        uint8_t* dp = data.data();
        for (std::size_t i = 0; i < TOTAL; ++i) {
            int rank = 0;
            uint64_t vi = vp[i];
            for (std::size_t j = 0; j < TOTAL; ++j)
                if (vp[j] < vi) ++rank;
            dp[i] = uint8_t(rank * 255 / (TOTAL - 1));
        }
    }
};

} // namespace detail

template <std::size_t Size>
struct BlueNoise {
    static_assert(Size > 0 && (Size & (Size - 1)) == 0, "Size must be power of two");
    static constexpr std::size_t N = Size;
    std::array<std::array<uint8_t, N>, N> data{};

    constexpr BlueNoise() {
        detail::BlueNoiseImpl<N> impl;
        impl.generate();
        for (std::size_t i = 0; i < N * N; ++i)
            data[i / N][i % N] = impl.data[i];
    }
};

using BlueNoise64 = BlueNoise<64>;
using BlueNoise128 = BlueNoise<128>;

} // namespace esp32gfx
