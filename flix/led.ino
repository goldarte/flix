// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// LED control

#define BLINK_PERIOD 500000
#include <FlixPeriph.h>
#include "util.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 2 // for ESP32 Dev Module
#endif

const int LED_OFF = 0, LED_ON = 1, LED_BLINK = 2;
int led_mode = LED_OFF;

int ledPin = 6;
int ledCount = 68;
float ledBrightness = 0.4;

RGB *led; // array of RGB values
int ledAnimation;
int ledColor[3] = {0, 0, 0};
float ledTime; // time of the last command
int ledType = 1;

WS2812 ws2812;
const int LEDSTRIP_OFF = 0, LEDSTRIP_RAINBOW = 1, LEDSTRIP_RUSSIAN = 2;
int ledstrip_mode = LEDSTRIP_RUSSIAN;

const int FILL = 0, BLINK = 1, RAINBOW = 2;

float modeAnimation[4];

void fill_solid(RGB *target, int numToFill, const RGB &color) {
	for (int i = 0; i < numToFill; ++i) {
			target[i] = color;
	}
}

void setupLEDStrip() {
	delete[] led;
	led = new RGB[ledCount];
	ws2812.begin(ledPin, ledCount);
	ws2812.setBrightness(ledBrightness);
	fill_solid(led, ledCount, RGB(0,255,0));
	ws2812.write(led);
}

void loopLEDStrip() {
	static Rate rate(30);
	if (!rate) return;
	handleLEDStrip();
	ws2812.write(led);
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

void setLEDStripOff() {
	fill_solid(led, ledCount, RGB(0,0,0));
}

void setLEDStripRainbow() {
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

void setLEDStripRussian() {
	static Rate rate(1);
	if (!rate) return;
	// red
	fill_solid(led, 14, RGB(255, 0, 0));
	fill_solid(led+54, 14, RGB(255, 0, 0));
	// blue
	fill_solid(led+14, 6, RGB(0, 0, 255));
	fill_solid(led+48, 6, RGB(0, 0, 255));
	// white
	fill_solid(led+20, 28, RGB(255, 255, 255));
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
