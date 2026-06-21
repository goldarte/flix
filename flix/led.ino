// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Board's LED control

#include <FastLED.h>

#define BLINK_PERIOD 500000

#ifndef LED_BUILTIN
#define LED_BUILTIN 2 // for ESP32 Dev Module
#endif

const int LED_OFF = 0, LED_ON = 1, LED_BLINK = 2;
int led_mode = LED_OFF;

#define LED_PIN     5      	// GPIO pin connected to Data In
#define NUM_LEDS    68      // Total number of LEDs
#define BRIGHTNESS  64      // 0 to 255
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB     // WS2812B is usually GRB

CRGB leds[NUM_LEDS];
uint8_t hue = 0;
const int LEDSTRIP_OFF = 0, LEDSTRIP_RAINBOW = 1, LEDSTRIP_RUSSIAN = 2;
int ledstrip_mode = LEDSTRIP_RUSSIAN;

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

void setupLEDStrip() {
	FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

	// Limit brightness to save power
	FastLED.setBrightness(BRIGHTNESS);

	fill_solid(leds, NUM_LEDS, CRGB::Green);
	FastLED.show();
}

void setLEDStripRussian() {
	static Rate rate(1);
	if (!rate) return;
	// red
	fill_solid(leds, 14, CRGB::Red);
	fill_solid(leds+54, 14, CRGB::Red);
	// blue
	fill_solid(leds+14, 6, CRGB::Blue);
	fill_solid(leds+48, 6, CRGB::Blue);
	// white
	fill_solid(leds+20, 28, CRGB::White);
	FastLED.show();
}

void setLEDStripRainbow(int led_period, int speed) {
	static Rate rate(50);
	if (!rate) return;
	fill_rainbow(leds, NUM_LEDS, hue-=speed, led_period);
	FastLED.show();
}

void setLEDStripOff() {
	static Rate rate(1);
	if (!rate) return;
	fill_solid(leds, NUM_LEDS, 0);
	FastLED.show();
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
			setLEDStripRainbow(7,5);
			break;
		case LEDSTRIP_RUSSIAN:
			setLEDStripRussian();
			break;
		default:
			setLEDStripOff();
	}
}
