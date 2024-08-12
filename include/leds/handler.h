#pragma once

#include "subsystems.h"
#include "leds/led.h"

/**
 * @brief Responsible for ensuring LEDs don't over-current by reducing pixel
 * brightnesses.
 * Uses singleton pattern to ensure only one is ever constructed.
 */
class LedPowerHandler : Subsystem {
  public:
    /**
     * @brief Ran every 10ms. Updates the led strips with the power constrained
     * RGB values.
     */
    void update() override;

    /** @brief Registers the strip to this handler. */
    void registerStrip(std::shared_ptr<LedStrip> strip);

    /**
     * @brief Gets the LedPowerHandler instance.
     * If it has not been previously constructed (instance == nullptr), then
     * this method will construct it.
     */
    static LedPowerHandler& get();
  private:
    LedPowerHandler();

    /**
     * @brief Should ever be one instance of LedPowerHandler, and that's this
     * one.
     */
    static LedPowerHandler* instance;

    /** @brief Map of ADI expander port numbers to handled strips. */
    std::unordered_multimap<uint8_t, std::shared_ptr<LedStrip>> m_strips;

    /** @brief The max current in Amps that an expander can provide. */
    static const float currentLimit;
};