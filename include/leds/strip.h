#pragma once

#include "pros/adi.hpp"
#include "leds/colors.h"
#include <memory>

class RGBBuffer {
  public:
    RGBBuffer(size_t length);

    /**
     * @brief Sets the color of the specified pixel in the buffer.
     *
     * @param index Zero based index of desired pixel.
     * @param color Desired color.
     */
    void setPixel(size_t index, HexRGB color);

    /**
     * @brief Sets all the pixels to the specified color.
     */
    void setAll(HexRGB color);

    /**
     * @brief Sets all the pixels to 0.
     */
    void clear();

    /**
     * @brief Sets all the pixels to be a gradient between startColor and
     * endColor.
     * @param startColor The desired color of the first pixel.
     * @param endColor The desired color of the last pixel.
     */
    void setGradient(uint32_t startColor, uint32_t endColor);

    /**
     * @brief Shifts colors of all pixels by distance pixel positions.
     * So, if the buffer was [red, green, blue], then after this a shift
     * with distance 1, it would then be [blue, red, green].
     * @param distance How far the pixels should be shifted.
     */
    void shift(size_t distance = 1);

    /** @return The length of the buffer. */
    size_t size() const;

    /** @brief Gets mutable color at index without bounds checking. */
    HexRGB& operator[](size_t index);
    /** @brief Gets mutable color at index with bounds checking. */
    HexRGB& at(size_t index);

    /** @brief Gets const color at index without bounds checking. */
    const HexRGB& operator[](size_t index) const;
    /** @brief Gets const color at index with bounds checking. */
    const HexRGB& at(size_t index) const;

    /** @return Raw mutable buffer */
    std::vector<HexRGB>& getBuffer();
    /** @return Raw const buffer */
    const std::vector<HexRGB>& getBuffer() const;

    /**
     * @brief Force updates the buffer's hash (indirectly updates the associated LedStrip).
     */
    void update();

    const size_t& getHash() const;
  protected:
    /** @brief Fakes hashing. Incremented when the buffer is modified. */
    size_t m_hash = 0;

    inline void updateHash() { ++m_hash; }

    std::vector<HexRGB> m_buffer;
};

/**
 * @brief Provides additional functionality to the LED object
 */
class LedStrip : protected pros::adi::LED {
  public:
    /**
     * @brief Creates an LedStrip and registers it to the LedPowerHandler, which
     * prevents over-currents, by reducing the brightness of the pixels.
     */
    static std::shared_ptr<LedStrip>
    create(pros::adi::ext_adi_port_pair_t port_pair, size_t length,
           float wattsPerPixelChannel);

    /** @return The length of the strip. */
    size_t size() const;

    /**
     * @return Estimation of current that will be drawn if desired colors are
     * applied.
     */
    float getCurrent() const;

    /** @return Port number of the ADI expander this led strip uses. */
    uint8_t getADIExpander() const;

    /**
     * @return Mutable buffer of pixel colors, before power scaling is applied.
     */
    RGBBuffer& buf();

    /** @return Const buffer of pixel colors, before power scaling is applied.
     */
    const RGBBuffer& buf() const;

    /**
     * @brief Multiplies reduction by the desired pixel colors, and applies the
     * result to the led strip.
     * @param reduction The amount to reduce the brightness of the pixels in the
     * interval (0, 1].
     */
    void applyReduction(float reduction);

    /**
     * @return Const buffer of pixel colors, after power scaling is applied.
     * Updated every 10ms.
     */
    const std::vector<HexRGB>& realBuffer() const;
  protected:
    LedStrip(pros::adi::ext_adi_port_pair_t port_pair, size_t length,
             float wattsPerPixelChannel);

    /** @brief The pixel colors before power scaling */
    RGBBuffer m_desiredBuffer;

    /**
     * @brief Previously applied reduction. Used to prevent unecessary writes
     * to the led strip.
     * first  - Hash of the rgb buffer when reduction was applied.
     * second - The most recently applied reduction.
     */
    std::pair<size_t, float> m_prevReduction {-1, -1};

    /**
     * @brief Previously calculated current. Used to prevent unnecessary
     * computation via caching.
     * first  - Hash of the rgb buffer when current was calculate.
     * second - The most recently calculated current.
     */
    mutable std::pair<size_t, float> m_prevCurrent {-1, -1};

    /**
     * @brief Force updates the led strip.
     */
    void update();

    /**
     * @brief The power used by a pixel at full brightness on a single color
     * channel.
     */
    const float m_wattsPerPixelChannel;
  private:
    using super = pros::adi::LED;
};