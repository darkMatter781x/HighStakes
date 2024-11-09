#pragma once

#include <cfloat>
#include <cmath>
#include <cstdint>

using HexRGB = uint32_t;

#define COLOR_OPERATOR(color_class, op, channel1, channel2, channel3)          \
  template <typename U, typename R = decltype(channel1 op std::declval<U>())>  \
  color_class<R> operator op(const U & operand) const {                        \
    return {.channel1 = channel1 op operand,                                   \
            .channel2 = channel2 op operand,                                   \
            .channel3 = channel3 op operand};                                  \
  }                                                                            \
  template <typename U, typename R = decltype(channel1 op std::declval<U>())>  \
  color_class<R> operator op(const color_class<U>& other) const {              \
    return {.channel1 = channel1 op other.channel1,                            \
            .channel2 = channel2 op other.channel2,                            \
            .channel3 = channel3 op other.channel3};                           \
  }

#define BASIC_COLOR_OPERATORS(color_class, channel1, channel2, channel3)       \
  COLOR_OPERATOR(color_class, +, channel1, channel2, channel3)                 \
  COLOR_OPERATOR(color_class, -, channel1, channel2, channel3)                 \
  COLOR_OPERATOR(color_class, *, channel1, channel2, channel3)                 \
  COLOR_OPERATOR(color_class, /, channel1, channel2, channel3)

#define RGB_BASIC_OPERATORS() BASIC_COLOR_OPERATORS(RGB, r, g, b)
#define HSV_BASIC_OPERATORS() BASIC_COLOR_OPERATORS(HSV, h, s, v)

template <typename T = uint8_t> struct RGB {
    T r;
    T g;
    T b;

    static RGB fromHex(HexRGB hex) {
      return RGB {.r = T((hex >> 16) & 0xFF),
                  .g = T((hex >> 8) & 0xFF),
                  .b = T(hex & 0xFF)};
    }

    RGB_BASIC_OPERATORS()

    template <typename U> explicit operator RGB<U>() const {
      return RGB<U> {.r = U(r), .g = U(g), .b = U(b)};
    }

    /** @return the largest of the three color channels */
    T max() const { return std::max({r, g, b}); }

    /** @return the smallest of the three color channels */
    T min() const { return std::min({r, g, b}); }
};

static HexRGB rgbToHex(RGB<uint8_t> rgb) {
  return (rgb.r << 16) | (rgb.g << 8) | rgb.b;
}

template <typename T> struct HSV {
    T h;
    T s;
    T v;

    HSV_BASIC_OPERATORS()

    static HSV<T> fromRGB(RGB<uint8_t> rgb) {
      // formula from https://math.stackexchange.com/a/3954976
      auto normalized = RGB<T>(rgb / 255.0f);

      const T max = normalized.max();
      const T min = normalized.min();
      const T delta = max - min;

      T h = 0;
      if (std::abs(delta) < FLT_EPSILON) {
        h = 0;
      } else if (std::abs(normalized.r - max) < FLT_EPSILON) {
        h = (normalized.g - normalized.b) / delta;
      } else if (std::abs(normalized.g - max) < FLT_EPSILON) {
        h = 2.0f + (normalized.b - normalized.r) / delta;
      } else {
        h = 4.0f + (normalized.r - normalized.g) / delta;
      }
      h /= 6.0f;
      h = std::fmod(h, 1.0f);
      h *= 360;

      const T s = max == 0 ? 0 : delta / max;
      const T v = max;

      return HSV<T> {.h = h, .s = s, .v = v};
    }
};

template <typename T> RGB<T> hsvToRgb(const HSV<T>& hsv) {
  // formula from https://en.wikipedia.org/wiki/HSL_and_HSV#From_HSV
  const T positiveH = std::fmod(360.0 * 10.0f + hsv.h, 360);

  const T c = hsv.v * hsv.s;
  const T x = c * (1 - std::abs(std::fmod(positiveH / 60, 2) - 1));
  const T m = hsv.v - c;

  RGB<T> rgb;
  switch (int(floor(positiveH / 60))) {
    case 0: rgb = {.r = c, .g = x, .b = 0}; break;
    case 1: rgb = {.r = x, .g = c, .b = 0}; break;
    case 2: rgb = {.r = 0, .g = c, .b = x}; break;
    case 3: rgb = {.r = 0, .g = x, .b = c}; break;
    case 4: rgb = {.r = x, .g = 0, .b = c}; break;
    case 5: rgb = {.r = c, .g = 0, .b = x}; break;
  }

  return rgb + m;
}