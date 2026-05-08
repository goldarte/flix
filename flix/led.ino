// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Board's LED control

#include <FastLED.h>

#define BLINK_PERIOD 500000

#ifndef LED_BUILTIN
#define LED_BUILTIN 2 // for ESP32 Dev Module
#endif

#define LED_PIN     5      	// GPIO pin connected to Data In
#define NUM_LEDS    68      // Total number of LEDs
#define BRIGHTNESS  64      // 0 to 255
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB     // WS2812B is usually GRB

CRGB leds[NUM_LEDS];
uint8_t hue = 0;

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


void setupLEDStrip() {
	FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

	// Limit brightness to save power
	FastLED.setBrightness(BRIGHTNESS);

	fill_solid(leds, NUM_LEDS, CRGB::Green);
  	FastLED.show();
}

void setLEDStripRainbow(int led_period, int speed) {
	static Rate rate(50);
	if (!rate) return;
	fill_rainbow(leds, NUM_LEDS, hue-=speed, led_period);
	FastLED.show();
}
