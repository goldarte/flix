// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Copyright (c) 2026 Arthur Golubtsov <goldartt@gmail.com>
// Repository: https://github.com/goldarte/flix

// LED control

#include <LiteLED.h>

// Board's LED params
#define BLINK_PERIOD 500000

#ifndef LED_BUILTIN
#define LED_BUILTIN 2 // for ESP32 Dev Module
#endif

const int LED_OFF = 0, LED_ON = 1, LED_BLINK = 2;
int led_mode = LED_OFF;

// LED strip parameters
#define LED_TYPE    LED_STRIP_WS2812
#define LED_PIN     6
#define NUM_LEDS    68
#define LED_IS_RGBW 0
#define BRIGHTNESS  50      // 0 to 255

const int LEDSTRIP_OFF = 0, LEDSTRIP_RAINBOW = 1, LEDSTRIP_RUSSIAN = 2;
int ledstrip_mode = LEDSTRIP_RUSSIAN;

// LiteLED strip
LiteLED strip(LED_TYPE, LED_IS_RGBW);

// Helper functions for led strip
void fillStrip(rgb_t color);
void fillStripPart(rgb_t color, int start_led, int num_leds);
crgb_t HSVtoRGB(uint8_t h, uint8_t s, uint8_t v);

void setupLED() {
	pinMode(LED_BUILTIN, OUTPUT);
}

void setLED(bool on) {
	static bool state = false;
	if (on == state) {
		return; // don't call digitalWrite if the state is the same
	}
	digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
	state = on;
}

void blinkLED() {
	setLED(micros() / BLINK_PERIOD % 2);
}

void setLEDMode(String mode) {
	if (mode == "off") {
		led_mode = LED_OFF;
	} else if (mode == "on") {
		led_mode = LED_ON;
	} else if (mode == "blink") {
		led_mode = LED_BLINK;
	}
}

void handleLED() {
	switch (led_mode) {
		case LED_OFF:
			setLED(false);
			break;
		case LED_ON:
			setLED(true);
			break;
		case LED_BLINK:
			blinkLED();
			break;
		default:
			setLED(false);
	}
}

void fillStrip(rgb_t color) {
  strip.fill(color, true);
}

void fillStripPart(rgb_t color, int start_led, int num_leds) {
  for (int i = start_led; i < start_led + num_leds; i++) {
    strip.setPixel(i, color);
  }
}

crgb_t HSVtoRGB(uint8_t h, uint8_t s, uint8_t v) {
    uint8_t region, remainder, p, q, t;
    uint8_t r, g, b;

    if (s == 0) {
        return ((uint32_t)v << 16) | ((uint32_t)v << 8) | v;
    }

    region = h / 43;
    remainder = (h - (region * 43)) * 6;

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }

    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

void setupLEDStrip() {
	strip.begin(LED_PIN, NUM_LEDS);

	// Limit brightness to save power
	strip.brightness(BRIGHTNESS);

	rgb_t green = rgb_from_values(0, 255, 0);
	fillStrip(green);
}

void setLEDStripRussian() {
	static Rate rate(1);
	if (!rate) return;
	// red
	rgb_t red = rgb_from_values(255, 0, 0);
	fillStripPart(red, 0, 14);
	fillStripPart(red, 54, 14);
	// blue
	rgb_t blue = rgb_from_values(0, 0, 255);
	fillStripPart(blue, 14, 6);
	fillStripPart(blue, 48, 6);
	// white
	rgb_t white = rgb_from_values(255, 255, 255);
	fillStripPart(white, 20, 28);
	strip.show();
}

void setLEDStripRainbow() {
	static Rate rate(50);
	if (!rate) return;
	static uint8_t hue = 0;
	for (size_t i = 0; i < NUM_LEDS; i++) {
		// Create rainbow effect
		uint8_t pixelHue = hue + (i * 255 / NUM_LEDS);
		strip.setPixel(i, HSVtoRGB(pixelHue, 255, 255));
	}
	strip.show();
	hue += 1;
}

void setLEDStripOff() {
	static Rate rate(1);
	if (!rate) return;
	fillStrip(rgb_from_values(0, 0, 0));
}

bool setLEDStripMode(String mode) {
	if (mode == "off") {
		return setParameter("LEDS_MODE", LEDSTRIP_OFF);
	} else if (mode == "rainbow" || mode == "lgbt") {
		return setParameter("LEDS_MODE", LEDSTRIP_RAINBOW);
	} else if (mode == "russian" || mode == "rus") {
		return setParameter("LEDS_MODE", LEDSTRIP_RUSSIAN);
	}
	return false;
}

void handleLEDStrip() {
	switch (ledstrip_mode) {
		case LEDSTRIP_OFF:
			setLEDStripOff();
			break;
		case LEDSTRIP_RAINBOW:
			setLEDStripRainbow();
			break;
		case LEDSTRIP_RUSSIAN:
			setLEDStripRussian();
			break;
		default:
			setLEDStripOff();
	}
}
