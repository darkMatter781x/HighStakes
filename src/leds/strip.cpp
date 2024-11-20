#include "leds/strip.h"
#include "leds/power.h"
#include "pros/rtos.hpp"

RGBBuffer::RGBBuffer(size_t length) : m_buffer(length) {}

void RGBBuffer::setPixel(size_t index, HexRGB color) {
  if (m_buffer.at(index) != color) updateHash();
  m_buffer.at(index) = color;
}

void RGBBuffer::setAll(HexRGB color) {
  updateHash();
  std::fill(m_buffer.begin(), m_buffer.end(), color);
}

void RGBBuffer::clear() { setAll(0x000000); }

void RGBBuffer::setGradient(HexRGB startColor, HexRGB endColor) {
  const size_t length = this->size();
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
    m_buffer[i] = rgbToHex(RGB<uint8_t>(rgb));
  }

  for (auto color : m_buffer) { printf("%x\n", color); }
  updateHash();
}

void RGBBuffer::shift(size_t distance) {
  const size_t length = this->size();
  distance %= length;

  uint32_t prevColor = m_buffer[length - 1];
  for (auto it = m_buffer.begin(); it != m_buffer.end(); ++it) {
    uint32_t temp = *it;
    *it = prevColor;
    prevColor = temp;
  }
  updateHash();
}

size_t RGBBuffer::size() const { return m_buffer.size(); }

HexRGB& RGBBuffer::operator[](size_t index) { return m_buffer[index]; }

HexRGB& RGBBuffer::at(size_t index) { return m_buffer.at(index); };

const HexRGB& RGBBuffer::operator[](size_t index) const {
  return m_buffer[index];
}

const HexRGB& RGBBuffer::at(size_t index) const { return m_buffer.at(index); };

std::vector<HexRGB>& RGBBuffer::getBuffer() { return m_buffer; };

const std::vector<HexRGB>& RGBBuffer::getBuffer() const { return m_buffer; };

const size_t& RGBBuffer::getHash() const { return m_hash; }

void RGBBuffer::update() { updateHash(); }

std::shared_ptr<LedStrip>
LedStrip::create(pros::adi::ext_adi_port_pair_t port_pair, size_t length,
                 float wattsPerPixelChannel) {
  std::shared_ptr<LedStrip> strip {
      new LedStrip {port_pair, length, wattsPerPixelChannel}};
  LedPowerHandler::get().registerStrip(strip);
  return strip;
}

size_t LedStrip::size() const { return buf().size(); }

float LedStrip::getCurrent() const {
  // If hash is the same, then use the cached current.
  if (buf().getHash() == m_prevCurrent.first) {
    // if (pros::millis() % 100 < 10) printf("current calc: cache hit!\n");
    return m_prevCurrent.second;
  }
  printf("current calc: cache miss!\n");

  float current = 0;
  for (auto pixel : buf().getBuffer()) {
    auto rgb = RGB<size_t>::fromHex(pixel) / 255.0f;
    current += (rgb.r + rgb.g + rgb.b) * m_wattsPerPixelChannel / (10.f / 3);
    // printf("%#08X, ", pixel);
  }
  // printf("\n");
  m_prevCurrent = std::pair<size_t, float> {buf().getHash(), current};
  return current;
}

uint8_t LedStrip::getADIExpander() const {
  return std::get<0>(super::get_port());
}

RGBBuffer& LedStrip::buf() { return m_desiredBuffer; }

const RGBBuffer& LedStrip::buf() const { return m_desiredBuffer; }

void LedStrip::applyReduction(float reduction) {
  // If hash is the same and the reduction factor is the same as the previous
  // time, then we don't need to update the led strip.
  if (m_prevReduction.first == buf().getHash() &&
      m_prevReduction.second == reduction)
    return;

  if (reduction == 1) super::_buffer = buf().getBuffer();
  else
    for (size_t i = 0; i < size(); ++i) {
      const auto desired = RGB<float>::fromHex(buf()[i]);
      const auto reduced = RGB<uint8_t>(desired * reduction);
      super::_buffer[i] = rgbToHex(reduced);
    }
  m_prevReduction = std::pair<size_t, float> {buf().getHash(), reduction};

  super::update();
}

const std::vector<HexRGB>& LedStrip::realBuffer() const {
  return super::_buffer;
};

LedStrip::LedStrip(pros::adi::ext_adi_port_pair_t port_pair, size_t length,
                   float wattsPerPixelChannel)
  : pros::adi::LED(port_pair, length),
    m_wattsPerPixelChannel(wattsPerPixelChannel), m_desiredBuffer(length) {
  super::clear();
}