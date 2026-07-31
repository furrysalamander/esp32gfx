#pragma once

#include "color.hpp"
#include <vector>
#include <concepts>

namespace esp32gfx {

template<Pixel P>
class Surface {
public:
    Surface() = default;
    Surface(int w, int h) : w_(w), h_(h), buf_(w * h, PixelTraits<P>::black_value()) {}

    int width()  const { return w_; }
    int height() const { return h_; }
    P* data()             { return buf_.data(); }
    const P* data() const { return buf_.data(); }

    void clear(Color c = Color::black()) {
        P v = PixelTraits<P>::pack(c);
        std::fill(buf_.begin(), buf_.end(), v);
    }

    void clear_depth() {
        if (!depth_.empty())
            depth_.assign(w_ * h_, 32767);
        else if (w_ > 0 && h_ > 0)
            depth_.assign(w_ * h_, 32767);
    }

    void pixel(int x, int y, Color c) {
        if (x < 0 || x >= w_ || y < 0 || y >= h_) return;
        buf_[y * w_ + x] = PixelTraits<P>::pack(c);
    }

    void pixel(int x, int y, Color c, int16_t z) {
        if (x < 0 || x >= w_ || y < 0 || y >= h_) return;
        int idx = y * w_ + x;
        if (!depth_.empty()) {
            if (z >= depth_[idx]) return;
            depth_[idx] = z;
        }
        buf_[idx] = PixelTraits<P>::pack(c);
    }

    Color pixel(int x, int y) const {
        if (x < 0 || x >= w_ || y < 0 || y >= h_) return {};
        return PixelTraits<P>::unpack(buf_[y * w_ + x]);
    }

    int16_t depth_at(int x, int y) const {
        if (x < 0 || x >= w_ || y < 0 || y >= h_ || depth_.empty()) return 32767;
        return depth_[y * w_ + x];
    }

private:
    int w_ = 0, h_ = 0;
    std::vector<P> buf_;
    std::vector<int16_t> depth_;
};

using SurfaceRGBA32 = Surface<uint32_t>;
using SurfaceGray8 = Surface<uint8_t>;

} // namespace esp32gfx
