// Copyright (c) 2026 Arthur Golubtsov <goldartt@gmail.com>
// Repository: https://github.com/goldarte/flix
// Assisted by ChatGPT

// Position and velocity control using an external odometry estimate

#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "util.h"

#define POSITION_P 1.0
#define VELOCITY_P 1.5
#define VELOCITY_I 0.2
#define VELOCITY_D 0.0
#define VELOCITY_I_LIM 1.0
#define VELOCITY_MAX_XY 2.0
#define VELOCITY_MAX_Z 1.0
#define HOVER_THRUST 0.5
#define ODOMETRY_TIMEOUT 0.5

Vector position(NAN, NAN, NAN); // external position estimate, local FLU, m
Vector velocity(NAN, NAN, NAN); // external velocity estimate, local FLU, m/s
Quaternion odometryAttitude(NAN, NAN, NAN, NAN); // external attitude estimate, body FLU to local FLU
float odometryTime = NAN;

Vector positionTarget(NAN, NAN, NAN);
Vector velocityTarget(NAN, NAN, NAN);
Vector accelerationTarget(NAN, NAN, NAN);
Vector velocitySetpoint(NAN, NAN, NAN);
Vector accelerationSetpoint(NAN, NAN, NAN);
float positionYawTarget = NAN;
float positionYawRateTarget = 0;
bool positionControlActive = false;

PID positionXPID(POSITION_P, 0, 0);
PID positionYPID(POSITION_P, 0, 0);
PID positionZPID(POSITION_P, 0, 0);
PID velocityXPID(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_I_LIM);
PID velocityYPID(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_I_LIM);
PID velocityZPID(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_I_LIM);
float velocityMaxXY = VELOCITY_MAX_XY;
float velocityMaxZ = VELOCITY_MAX_Z;
float hoverThrust = HOVER_THRUST;
float odometryTimeout = ODOMETRY_TIMEOUT;

extern const int STAB, POS;
extern int mode;
extern int previousMode;
extern bool armed;
extern float controlRoll, controlPitch, controlThrottle, controlYaw;
extern Quaternion attitudeTarget;
extern Vector ratesExtra, maxRate;
extern float thrustTarget, tiltMax;

bool odometryValid() {
	return position.valid() && velocity.valid() && t - odometryTime <= odometryTimeout;
}

void resetPositionTargets() {
	positionTarget = position;
	velocityTarget = Vector(0, 0, 0);
	accelerationTarget = Vector(0, 0, 0);
	positionYawTarget = attitude.getYaw();
	positionYawRateTarget = 0;
	positionXPID.reset();
	positionYPID.reset();
	positionZPID.reset();
	velocityXPID.reset();
	velocityYPID.reset();
	velocityZPID.reset();
}

void interpretPositionControls() {
	if (!odometryValid()) {
		// Position mode cannot operate without a fresh external estimate.
		mode = STAB;
		positionControlActive = false;
		return;
	}

	if (previousMode != POS || !armed) resetPositionTargets();
	positionControlActive = true;

	float yaw = attitude.getYaw();
	float forward = controlPitch * velocityMaxXY;
	float left = controlRoll * velocityMaxXY;
	velocityTarget.x = cos(yaw) * forward - sin(yaw) * left;
	velocityTarget.y = sin(yaw) * forward + cos(yaw) * left;

	float verticalControl = (controlThrottle - 0.5f) * 2;
	if (abs(verticalControl) < 0.1f) verticalControl = 0;
	velocityTarget.z = verticalControl * velocityMaxZ;

	// While a stick is active, move the hold point with the vehicle. When the
	// stick is released the last position is retained.
	if (abs(controlRoll) > 0.05f || abs(controlPitch) > 0.05f) {
		positionTarget.x = position.x;
		positionTarget.y = position.y;
	}
	if (verticalControl != 0) positionTarget.z = position.z;

	if (abs(controlYaw) < 0.1f) controlYaw = 0;
	if (!armed || controlYaw != 0) positionYawTarget = attitude.getYaw();
	positionYawRateTarget = -controlYaw * maxRate.z;
}

float positionAxis(PID& pid, float target, float actual, float feedforward, float limit) {
	if (!isfinite(target) && !isfinite(feedforward)) return NAN;
	float result = isfinite(target) ? pid.update(target - actual) : 0;
	if (isfinite(feedforward)) result += feedforward;
	return constrain(result, -limit, limit);
}

void controlPosition() {
	if (!positionControlActive) return;
	if (!odometryValid()) return;

	velocitySetpoint.x = positionAxis(positionXPID, positionTarget.x, position.x, velocityTarget.x, velocityMaxXY);
	velocitySetpoint.y = positionAxis(positionYPID, positionTarget.y, position.y, velocityTarget.y, velocityMaxXY);
	velocitySetpoint.z = positionAxis(positionZPID, positionTarget.z, position.z, velocityTarget.z, velocityMaxZ);
}

float velocityAxis(PID& pid, float target, float actual, float feedforward) {
	if (!isfinite(target) && !isfinite(feedforward)) return 0;
	float result = isfinite(target) ? pid.update(target - actual) : 0;
	if (isfinite(feedforward)) result += feedforward;
	return result;
}

void controlVelocity() {
	if (!positionControlActive) return;
	if (!odometryValid()) return;

	accelerationSetpoint.x = velocityAxis(velocityXPID, velocitySetpoint.x, velocity.x, accelerationTarget.x);
	accelerationSetpoint.y = velocityAxis(velocityYPID, velocitySetpoint.y, velocity.y, accelerationTarget.y);
	accelerationSetpoint.z = velocityAxis(velocityZPID, velocitySetpoint.z, velocity.z, accelerationTarget.z);
}

void controlAcceleration() {
	if (!positionControlActive) return;
	if (!odometryValid() || accelerationSetpoint.invalid()) return;

	// Limit horizontal acceleration to the configured maximum tilt.
	float horizontal = sqrt(accelerationSetpoint.x * accelerationSetpoint.x + accelerationSetpoint.y * accelerationSetpoint.y);
	float horizontalMax = ONE_G * tan(tiltMax);
	if (horizontal > horizontalMax) {
		accelerationSetpoint.x *= horizontalMax / horizontal;
		accelerationSetpoint.y *= horizontalMax / horizontal;
		horizontal = horizontalMax;
	}

	float yaw = isfinite(positionYawTarget) ? positionYawTarget : attitude.getYaw();
	float forward = cos(yaw) * accelerationSetpoint.x + sin(yaw) * accelerationSetpoint.y;
	float left = -sin(yaw) * accelerationSetpoint.x + cos(yaw) * accelerationSetpoint.y;
	float vertical = ONE_G + accelerationSetpoint.z;
	vertical = max(vertical, ONE_G * 0.1f);

	// Positive pitch tilts thrust forward; negative roll tilts it left in FLU.
	float roll = -atan2(left, vertical);
	float pitch = atan2(forward, vertical);
	attitudeTarget = Quaternion::fromEuler(Vector(roll, pitch, yaw));
	ratesExtra = Vector(0, 0, positionYawRateTarget);
	thrustTarget = constrain(hoverThrust * sqrt(horizontal * horizontal + vertical * vertical) / ONE_G, 0, 1);
}
