// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// LED control

#include <FastLED.h>

#define BLINK_PERIOD 500000
#include <FlixPeriph.h>
#include "util.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 2 // for ESP32 Dev Module
#endif

const int LED_OFF = 0, LED_ON = 1, LED_BLINK = 2;
int led_mode = LED_OFF;

#define LED_PIN     6      	// GPIO pin connected to Data In
#define NUM_LEDS    68      // Total number of LEDs
#define BRIGHTNESS  64      // 0 to 255
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB     // WS2812B is usually GRB

CRGB leds[NUM_LEDS];
uint8_t hue = 0;
const int LEDSTRIP_OFF = 0, LEDSTRIP_RAINBOW = 1, LEDSTRIP_RUSSIAN = 2;
int ledstrip_mode = LEDSTRIP_RUSSIAN;
///////
int ledType = 0; // 0 - single LED, 1 - WS2812, -1 - disabled
int ledPin = LED_BUILTIN;
int ledCount = 1;
float ledBrightness = 1;

RGB *led; // array of RGB values
int ledAnimation;
int ledColor[3]; // animation color
float ledTime; // time of the last command

WS2812 ws2812;
const int FILL = 0, BLINK = 1, RAINBOW = 2;

float modeAnimation[4];

void setupLED() {
	delete[] led;

	if (ledType == 0) {
		pinMode(ledPin, OUTPUT);
		led = new RGB[1];
	} else if (ledType == 1) {
		led = new RGB[ledCount];
		ws2812.begin(ledPin, ledCount);
		ws2812.setBrightness(ledBrightness);
	}
}

void loopLED() {
	if (ledType == -1) return;
	static Rate rate(30);
	if (!rate) return;

	// Handle mode change
	static int prevMode = -1;
	if (prevMode != mode) {
		unpackAnimation(modeAnimation[mode], ledAnimation, ledColor[0], ledColor[1], ledColor[2]);
		ledTime = t;
		prevMode = mode;
	}

	handleAnimation();

	if (ledType == 0) {
		digitalWrite(ledPin, ledColor[0] || ledColor[1] || ledColor[2] ? HIGH : LOW); // fall back to single LED
	} else if (ledType == 1) {
		ws2812.write(led);
	}
}

void handleAnimation() {
	if (ledAnimation == FILL && t - ledTime < 0.1) {
		for (int i = 0; i < ledCount; ++i) {
			led[i] = RGB(ledColor[0], ledColor[1], ledColor[2]);
		}
	}

	if (ledAnimation == RAINBOW) {
		const float period = ledColor[0] > 0 ? ledColor[0] : 10;
		float cyclePhase = fmod(t / period, 1.0f);
		for (int i = 0; i < ledCount; ++i) {
			float ledPhase = fmod(cyclePhase + (float)i / ledCount, 1.0f) * 3.0f;
			int segment = (int)ledPhase;
			uint8_t segmentLevel = (ledPhase - segment) * 255;
			led[i] = segment == 0 ? RGB(segmentLevel, (uint8_t)(255 - segmentLevel), 0)
				: segment == 1 ? RGB((uint8_t)(255 - segmentLevel), 0, segmentLevel)
				: RGB(0, segmentLevel, (uint8_t)(255 - segmentLevel));
		}
	}

	if (ledAnimation == BLINK) {
		const float period = 2;
		int state = fmod(t, period) < period / 2;
		for (int i = 0; i < ledCount; ++i) {
			led[i] = RGB(state ? ledColor[0] : 0, state ? ledColor[1] : 0, state ? ledColor[2] : 0);
		}
	}
}

float packAnimation(uint8_t animation, uint8_t r, uint8_t g, uint8_t b) {
	float value;
	uint8_t bytes[4] = {animation, r, g, b};
	memcpy(&value, bytes, sizeof(value));
	return value;
}

void unpackAnimation(float value, int& animation, int& r, int& g, int& b) {
	uint8_t bytes[4];
	memcpy(bytes, &value, sizeof(value));
	animation = bytes[0];
	r = bytes[1];
	g = bytes[2];
	b = bytes[3];
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
