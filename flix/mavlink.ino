// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// MAVLink communication

#include <MAVLink.h>
#include "util.h"

extern float controlTime;
extern float voltage;

int mavlinkSysId = 1;

Rate telemetrySlow(2);
Rate telemetryAttitude(20);
Rate telemetryRC(10);
Rate telemetryMotors(10);
Rate telemetryIMU(15);
Rate telemetryPosition(10);

bool mavlinkConnected = false;
String mavlinkPrintBuffer;

bool decodeOdometryAttitude(const mavlink_odometry_t& odometry, Quaternion& result) {
	bool bodyFrame = odometry.child_frame_id == MAV_FRAME_BODY_NED ||
		odometry.child_frame_id == MAV_FRAME_BODY_OFFSET_NED ||
		odometry.child_frame_id == MAV_FRAME_BODY_FRD;
	if (!bodyFrame) return false;

	Quaternion raw(odometry.q[0], odometry.q[1], odometry.q[2], odometry.q[3]);
	float norm = raw.norm();
	if (!isfinite(norm) || norm < 1e-6f) return false;
	raw.normalize();

	// MAVLink body frames are FRD. Rotate both the local and body axes into
	// Flix's internal FLU convention. ENU also needs a -90 degree local yaw.
	Quaternion frdToFlu(0, 1, 0, 0);
	switch (odometry.frame_id) {
		case MAV_FRAME_LOCAL_NED:
		case MAV_FRAME_LOCAL_FRD:
			result = frdToFlu * raw * frdToFlu;
			break;
		case MAV_FRAME_LOCAL_ENU:
			result = Quaternion::fromEuler(Vector(0, 0, -PI / 2)) * raw * frdToFlu;
			break;
		case MAV_FRAME_LOCAL_FLU:
			result = raw * frdToFlu;
			break;
		default:
			return false;
	}
	result.normalize();
	return result.finite();
}

void processMavlink() {
	sendMavlink();
	receiveMavlink();
}

void sendMavlink() {
	sendMavlinkPrint();

	mavlink_message_t msg;
	uint32_t time = t * 1000;

	if (telemetrySlow) {
		mavlink_msg_heartbeat_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
			(armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0) |
			((mode == STAB || mode == POS) ? MAV_MODE_FLAG_STABILIZE_ENABLED : 0) |
			((mode == AUTO) ? MAV_MODE_FLAG_AUTO_ENABLED : MAV_MODE_FLAG_MANUAL_INPUT_ENABLED),
			mode, MAV_STATE_STANDBY);
		sendMessage(&msg);
	}

	if (!mavlinkConnected) return; // send only heartbeat until connected

	if (telemetrySlow) {
		mavlink_msg_extended_sys_state_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
			MAV_VTOL_STATE_UNDEFINED, landed ? MAV_LANDED_STATE_ON_GROUND : MAV_LANDED_STATE_IN_AIR);
		sendMessage(&msg);
	}

	if (telemetrySlow && valid(voltage)) {
		uint16_t voltages[] = {(uint16_t)(voltage * 1000), UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX};
		uint16_t voltagesExt[] = {0, 0, 0, 0};
		float remaining = constrain(mapf(voltage, 3.4, 4.2, 0, 1), 0, 1);
		mavlink_msg_battery_status_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg, 0, MAV_BATTERY_FUNCTION_ALL,
			MAV_BATTERY_TYPE_LIPO, INT16_MAX, voltages, -1, -1, -1, remaining * 100, 0, MAV_BATTERY_CHARGE_STATE_OK, voltagesExt, 0, 0);
		sendMessage(&msg);
	}

	if (telemetryAttitude) {
		const float offset[] = {0, 0, 0, 0};
		mavlink_msg_attitude_quaternion_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
			time, attitude.w, attitude.x, -attitude.y, -attitude.z, rates.x, -rates.y, -rates.z, offset); // convert to frd
		sendMessage(&msg);
	}

	if (telemetryRC && channels[0]) { // 0 means no RC input
		mavlink_msg_rc_channels_raw_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg, controlTime * 1000, 0,
			channels[0], channels[1], channels[2], channels[3], channels[4], channels[5], channels[6], channels[7], UINT8_MAX);
		sendMessage(&msg);
	}

	if (telemetryMotors) {
		float controls[8];
		memcpy(controls, motors, sizeof(motors));
		mavlink_msg_actuator_control_target_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg, time, 0, controls);
		sendMessage(&msg);
	}

	if (telemetryIMU) {
		mavlink_msg_scaled_imu_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg, time,
			acc.x / ONE_G * 1000, -acc.y / ONE_G * 1000, -acc.z / ONE_G * 1000, // convert to frd
			gyro.x * 1000, -gyro.y * 1000, -gyro.z * 1000,
			0, 0, 0, 0);
		sendMessage(&msg);
	}

	if (telemetryPosition && odometryValid()) {
		mavlink_msg_local_position_ned_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg, time,
			position.x, -position.y, -position.z, velocity.x, -velocity.y, -velocity.z);
		sendMessage(&msg);
	}
}

void sendMessage(const void *msg) {
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	int len = mavlink_msg_to_send_buffer(buf, (mavlink_message_t *)msg);
	sendWiFi(buf, len);
}

void receiveMavlink() {
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	int len = receiveWiFi(buf, MAVLINK_MAX_PACKET_LEN);

	// New packet, parse it
	mavlink_message_t msg;
	mavlink_status_t status;
	for (int i = 0; i < len; i++) {
		if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
			mavlinkConnected = true;
			handleMavlink(&msg);
		}
	}
}

void handleMavlink(const void *_msg) {
	const mavlink_message_t& msg = *(mavlink_message_t *)_msg;

	if (msg.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL) {
		mavlink_manual_control_t m;
		mavlink_msg_manual_control_decode(&msg, &m);
		if (m.target && m.target != mavlinkSysId) return; // 0 is broadcast

		controlThrottle = m.z / 1000.0f;
		controlPitch = m.x / 1000.0f;
		controlRoll = m.y / 1000.0f;
		controlYaw = m.r / 1000.0f;
		controlMode = NAN;
		controlTime = t;
	}

	if (msg.msgid == MAVLINK_MSG_ID_ODOMETRY) {
		mavlink_odometry_t m;
		mavlink_msg_odometry_decode(&msg, &m);
		if (m.quality < 0) {
			position.invalidate();
			velocity.invalidate();
			odometryAttitude.invalidate();
			return;
		}
		Quaternion newAttitude;
		bool attitudeSupported = decodeOdometryAttitude(m, newAttitude);

		Vector newPosition;
		bool positionSupported = true;
		switch (m.frame_id) {
			case MAV_FRAME_LOCAL_NED:
			case MAV_FRAME_LOCAL_FRD:
				newPosition = Vector(m.x, -m.y, -m.z);
				break;
			case MAV_FRAME_LOCAL_ENU:
				newPosition = Vector(m.y, -m.x, m.z);
				break;
			case MAV_FRAME_LOCAL_FLU:
				newPosition = Vector(m.x, m.y, m.z);
				break;
			default:
				positionSupported = false;
		}

		Vector newVelocity;
		bool velocitySupported = true;
		switch (m.child_frame_id) {
			case MAV_FRAME_LOCAL_NED:
			case MAV_FRAME_LOCAL_FRD:
				newVelocity = Vector(m.vx, -m.vy, -m.vz);
				break;
			case MAV_FRAME_LOCAL_ENU:
				newVelocity = Vector(m.vy, -m.vx, m.vz);
				break;
			case MAV_FRAME_LOCAL_FLU:
				newVelocity = Vector(m.vx, m.vy, m.vz);
				break;
			case MAV_FRAME_BODY_NED:
			case MAV_FRAME_BODY_FRD:
			case MAV_FRAME_BODY_OFFSET_NED:
				newVelocity = (attitudeSupported ? newAttitude : attitude).conjugate(
					Vector(m.vx, -m.vy, -m.vz));
				break;
			default:
				velocitySupported = false;
		}

		if (positionSupported && velocitySupported && newPosition.finite() && newVelocity.finite()) {
			position = newPosition;
			velocity = newVelocity;
			if (attitudeSupported) {
				odometryAttitude = newAttitude;
				attitude = newAttitude;
			} else {
				odometryAttitude.invalidate();
			}
			odometryTime = t;
		}
	}

	if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
		mavlink_param_request_list_t m;
		mavlink_msg_param_request_list_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;

		mavlink_message_t msg;
		for (int i = 0; i < parametersCount(); i++) {
			mavlink_msg_param_value_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
				getParameterName(i), getParameter(i), MAV_PARAM_TYPE_REAL32, parametersCount(), i);
			sendMessage(&msg);
		}
	}

	if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
		mavlink_param_request_read_t m;
		mavlink_msg_param_request_read_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;

		char name[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN + 1];
		strlcpy(name, m.param_id, sizeof(name)); // param_id might be not null-terminated
		float value = strlen(name) == 0 ? getParameter(m.param_index) : getParameter(name);
		if (m.param_index != -1) {
			memcpy(name, getParameterName(m.param_index), 16);
		}
		mavlink_message_t msg;
		mavlink_msg_param_value_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
			name, value, MAV_PARAM_TYPE_REAL32, parametersCount(), m.param_index);
		sendMessage(&msg);
	}

	if (msg.msgid == MAVLINK_MSG_ID_PARAM_SET) {
		mavlink_param_set_t m;
		mavlink_msg_param_set_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;

		char name[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN + 1];
		strlcpy(name, m.param_id, sizeof(name)); // param_id might be not null-terminated
		bool success = setParameter(name, m.param_value);
		if (!success) return;
		// send ack
		mavlink_message_t msg;
		mavlink_msg_param_value_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
			m.param_id, getParameter(name), MAV_PARAM_TYPE_REAL32, parametersCount(), 0); // index is unknown
		sendMessage(&msg);
	}

	if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_LIST) { // handle to make qgc happy
		mavlink_mission_request_list_t m;
		mavlink_msg_mission_request_list_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;

		mavlink_message_t msg;
		mavlink_msg_mission_count_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg, 0, 0, 0, MAV_MISSION_TYPE_MISSION, 0);
		sendMessage(&msg);
	}

	if (msg.msgid == MAVLINK_MSG_ID_SERIAL_CONTROL) {
		mavlink_serial_control_t m;
		mavlink_msg_serial_control_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;

		char data[MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN + 1];
		strlcpy(data, (const char *)m.data, m.count); // data might be not null-terminated
		doCommand(data, true);
	}

	if (msg.msgid == MAVLINK_MSG_ID_SET_ATTITUDE_TARGET) {
		if (mode != AUTO) return;

		mavlink_set_attitude_target_t m;
		mavlink_msg_set_attitude_target_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;
		positionControlActive = false;

		// copy attitude, rates and thrust targets
		ratesTarget.x = m.body_roll_rate;
		ratesTarget.y = -m.body_pitch_rate; // convert to flu
		ratesTarget.z = -m.body_yaw_rate;
		attitudeTarget.w = m.q[0];
		attitudeTarget.x = m.q[1];
		attitudeTarget.y = -m.q[2];
		attitudeTarget.z = -m.q[3];
		thrustTarget = m.thrust;
		ratesExtra = Vector(0, 0, 0);

		if (m.type_mask & ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE) attitudeTarget.invalidate();
		armed = m.thrust > 0;
	}

	if (msg.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) {
		if (mode != AUTO) return;

		mavlink_set_position_target_local_ned_t m;
		mavlink_msg_set_position_target_local_ned_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;
		if (m.target_component && m.target_component != MAV_COMP_ID_AUTOPILOT1) return;
		if (!odometryValid()) return;

		bool bodyFrame = m.coordinate_frame == MAV_FRAME_BODY_NED || m.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
		bool offsetFrame = m.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED || m.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
		if (m.coordinate_frame != MAV_FRAME_LOCAL_NED && !bodyFrame && !offsetFrame) return;
		positionControlActive = true;

		Vector positionCommand(m.x, -m.y, -m.z);
		Vector velocityCommand(m.vx, -m.vy, -m.vz);
		Vector accelerationCommand(m.afx, -m.afy, -m.afz);
		if (bodyFrame) {
			velocityCommand = attitude.conjugate(velocityCommand);
			accelerationCommand = attitude.conjugate(accelerationCommand);
		}
		if (m.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
			positionCommand = attitude.conjugate(positionCommand);
		}

		positionTarget.x = m.type_mask & POSITION_TARGET_TYPEMASK_X_IGNORE ? NAN : positionCommand.x + (offsetFrame ? position.x : 0);
		positionTarget.y = m.type_mask & POSITION_TARGET_TYPEMASK_Y_IGNORE ? NAN : positionCommand.y + (offsetFrame ? position.y : 0);
		positionTarget.z = m.type_mask & POSITION_TARGET_TYPEMASK_Z_IGNORE ? NAN : positionCommand.z + (offsetFrame ? position.z : 0);
		velocityTarget.x = m.type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE ? NAN : velocityCommand.x;
		velocityTarget.y = m.type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE ? NAN : velocityCommand.y;
		velocityTarget.z = m.type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE ? NAN : velocityCommand.z;
		bool forceSet = m.type_mask & POSITION_TARGET_TYPEMASK_FORCE_SET;
		accelerationTarget.x = forceSet || (m.type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) ? NAN : accelerationCommand.x;
		accelerationTarget.y = forceSet || (m.type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) ? NAN : accelerationCommand.y;
		accelerationTarget.z = forceSet || (m.type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) ? NAN : accelerationCommand.z;
		bool yawIgnored = m.type_mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE;
		bool yawRateIgnored = m.type_mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
		positionYawTarget = yawIgnored ? (yawRateIgnored ? attitude.getYaw() : NAN) : -m.yaw;
		positionYawRateTarget = yawRateIgnored ? 0 : -m.yaw_rate;
	}

	if (msg.msgid == MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET) {
		if (mode != AUTO) return;

		mavlink_set_actuator_control_target_t m;
		mavlink_msg_set_actuator_control_target_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;
		positionControlActive = false;

		attitudeTarget.invalidate();
		ratesTarget.invalidate();
		torqueTarget.invalidate();
		memcpy(motors, m.controls, sizeof(motors)); // copy motor thrusts
		armed = motors[0] > 0 || motors[1] > 0 || motors[2] > 0 || motors[3] > 0;
	}

	if (msg.msgid == MAVLINK_MSG_ID_LOG_REQUEST_DATA) {
		mavlink_log_request_data_t m;
		mavlink_msg_log_request_data_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;

		// Send all log records
		for (int i = 0; i < sizeof(logBuffer) / sizeof(logBuffer[0]); i++) {
			mavlink_message_t msg;
			mavlink_msg_log_data_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg, 0, i,
				sizeof(logBuffer[0]), (uint8_t *)logBuffer[i]);
			sendMessage(&msg);
		}
	}

	// Handle commands
	if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
		mavlink_command_long_t m;
		mavlink_msg_command_long_decode(&msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;
		mavlink_message_t response;
		bool accepted = false;

		if (m.command == MAV_CMD_REQUEST_MESSAGE && m.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
			accepted = true;
			mavlink_msg_autopilot_version_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &response,
				MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT | MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED | MAV_PROTOCOL_CAPABILITY_MAVLINK2,
				1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
			sendMessage(&response);
		}

		if (m.command == MAV_CMD_COMPONENT_ARM_DISARM) {
			if (m.param1 == 1 && controlThrottle > 0.05) return; // don't arm if throttle is not low
			accepted = true;
			armed = m.param1 == 1;
		}

		if (m.command == MAV_CMD_DO_SET_MODE) {
			if (m.param2 < 0 || m.param2 > POS) return; // incorrect mode
			accepted = true;
			mode = m.param2;
		}

		// send command ack
		mavlink_message_t ack;
		mavlink_msg_command_ack_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &ack, m.command, accepted ? MAV_RESULT_ACCEPTED : MAV_RESULT_UNSUPPORTED, UINT8_MAX, 0, msg.sysid, msg.compid);
		sendMessage(&ack);
	}
}

// Send shell output to GCS
void mavlinkPrint(const char* str) {
	mavlinkPrintBuffer += str;
}

void sendMavlinkPrint() {
	// Send mavlink print data in chunks
	const char *str = mavlinkPrintBuffer.c_str();
	for (int i = 0; i < strlen(str); i += MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN) {
		char data[MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN + 1];
		strlcpy(data, str + i, sizeof(data));
		mavlink_message_t msg;
		mavlink_msg_serial_control_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
			SERIAL_CONTROL_DEV_SHELL,
			i + MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN < strlen(str) ? SERIAL_CONTROL_FLAG_MULTI : 0, // more chunks to go
			0, 0, strlen(data), (uint8_t *)data, 0, 0);
		sendMessage(&msg);
	}
	mavlinkPrintBuffer.clear();
}
