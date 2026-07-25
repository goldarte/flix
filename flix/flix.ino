// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Main firmware file

#include "vector.h"
#include "quaternion.h"
#include "util.h"
#include <MAVLink.h>

class PID;

extern float t, dt;
extern float controlRoll, controlPitch, controlYaw, controlThrottle, controlMode;
extern Vector gyro, acc;
extern Vector rates;
extern Quaternion attitude;
extern Vector position, velocity;
extern Quaternion odometryAttitude;
extern float odometryTime;
extern Vector positionTarget, velocityTarget, accelerationTarget;
extern float positionYawTarget, positionYawRateTarget;
extern bool positionControlActive;
extern bool landed;
extern float motors[4];

void setup() {
	Serial.begin(115200);
	print("Initializing Flix\n");
	setupParameters();
	setupPower();
	setupLED();
	setLED(true);
	setupMotors();
	setupWiFi();
	setupIMU();
	setupRC();
	setLED(false);
	print("Initializing complete\n");
}

void loop() {
	readIMU();
	step();
	readRC();
	estimate();
	control();
	sendMotors();
	handleInput();
	processMavlink();
	readVoltage();
	logData();
	syncParameters();
}
