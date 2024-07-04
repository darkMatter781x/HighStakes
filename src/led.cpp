#include "led.h"
#include "pros/adi.hpp"
#include <cstddef>
#include <cstdint>
#include <cstdio>

LedStrip::LedStrip(pros::adi::LED& led) : pros::adi::LED(led) {
  super::clear();
}

size_t LedStrip::getLength() const { return super::_buffer.size(); }

void LedStrip::setPixel(size_t index, uint32_t color) {
  super::set_pixel(color, index);
}

void LedStrip::setAll(uint32_t color) { super::set_all(color); }

void LedStrip::clear() { super::clear(); }

void LedStrip::setGradient(HexRGB startColor, HexRGB endColor) {
  const size_t length = this->getLength();
  const HSV<float> startHSV =
      HSV<float>::fromRGB(RGB<uint8_t>::fromHex(startColor));
  const HSV<float> endHSV =
      HSV<float>::fromRGB(RGB<uint8_t>::fromHex(endColor));

  // the delta between each pixel
  const HSV<float> delta = (endHSV - startHSV) / float(length - 1);

  printf("start: %f %f %f\n", startHSV.h, startHSV.s, startHSV.v);
  printf("end: %f %f %f\n", endHSV.h, endHSV.s, endHSV.v);
  printf("delta: %f %f %f\n", delta.h, delta.s, delta.v);

  for (size_t i = 0; i < length; ++i) {
    const auto hsv = startHSV + delta * float(i);
    printf("hsv: %f %f %f\n", hsv.h, hsv.s, hsv.v);
    const auto rgb = hsvToRgb(hsv) * 255;
    printf("rgb: %f %f %f\n", rgb.r, rgb.g, rgb.b);
    _buffer[i] = rgbToHex(RGB<uint8_t>(rgb));
  }

  for (auto color : _buffer) { printf("%x\n", color); }

  super::update();
}

void LedStrip::shift(size_t distance) {
  const size_t length = this->getLength();
  distance %= length;

  uint32_t prevColor = _buffer[length - 1];
  for (auto it = _buffer.begin(); it != _buffer.end(); ++it) {
    uint32_t temp = *it;
    *it = prevColor;
    prevColor = temp;
  }

  super::update();
}