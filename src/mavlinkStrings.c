/*
  August 2014

  myFlight32 Rev -

  Copyright (c) 2014 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Control Software

  Designed to run on Naze32Pro and AQ32 Flight Control Boards

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)MultiWii
  4)Paparazzi UAV
  5)S.O.H. Madgwick
  6)UAVX
  7)Me!!

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

void (*mavlinkPortPrintBinary)(uint8_t *buf, uint16_t length);

///////////////////////////////////////

mavlink_system_t mavlink_system;

// Define the system type variables
uint8_t  system_type;
uint8_t  autopilot_type;

uint8_t  system_mode;
uint32_t custom_mode;
uint8_t  system_state;

// Initialize the required buffers
mavlink_message_t msg;

uint8_t  buffer[MAVLINK_MAX_PACKET_LEN];

uint16_t length;

///////////////////////////////////////////////////////////////////////////////

void initMavlink(void)
{
	mavlink_system.sysid   = 20;                  // @todo Future EEPROM Candidate ID 20 for this vehicle
	mavlink_system.compid  = MAV_COMP_ID_IMU;     // The component sending the message is the IMU
	mavlink_system.type    = MAV_TYPE_QUADROTOR;  // @todo Future EEPROM Candidate System Type

	// Define the system type
	system_type    = MAV_TYPE_QUADROTOR;          // @todo Future EEPROM Candidate
	autopilot_type = MAV_AUTOPILOT_GENERIC;

	system_mode    = MAV_MODE_PREFLIGHT;          // Booting up
	custom_mode    = 0;                           // Custom mode, can be defined by user/adopter
	system_state   = MAV_STATE_STANDBY;           // System ready for flight
}

///////////////////////////////////////////////////////////////////////////////

void mavlinkSendAttitude(void)
{
	mavlink_msg_attitude_pack(mavlink_system.sysid,               // uint8_t            system_id,
                              mavlink_system.compid,              // uint8_t            component_id,
                              &msg,                               // mavlink_message_t* msg,
						      millis(),                           // uint32_t           time_boot_ms,
						      tasks500Hz_Y.attitudes[ROLL ],      // float              roll,
						      tasks500Hz_Y.attitudes[PITCH],      // float              pitch,
						      tasks500Hz_Y.attitudes[YAW  ],      // float              yaw,
						      tasks500Hz_U.gyro[ROLL ],           // float              rollspeed,
						      tasks500Hz_U.gyro[PITCH],           // float              pitchspeed,
						      tasks500Hz_U.gyro[YAW  ]);          // float              yawspeed);

	// Copy the message to the send buffer
	length = mavlink_msg_to_send_buffer(buffer, &msg);

    mavlinkPortPrintBinary(buffer, length);
}

///////////////////////////////////////////////////////////////////////////////

void mavlinkSendGpsRaw(void)
{
    mavlink_msg_gps_raw_int_pack(mavlink_system.sysid,            // uint8_t            system_id,
                                 mavlink_system.compid,           // uint8_t            component_id,
                                 &msg,                            // mavlink_message_t* msg,
							     (uint64_t)micros(),              // uint64_t           time_usec,
							     sensors.gps.fix,                 // uint8_t            fix_type,
							     sensors.gps.latitude,            // int32_t            lat,
							     sensors.gps.longitude,           // int32_t            lon,
							     sensors.gps.hMSL,                // int32_t            alt,
							     sensors.gps.hDop,                // uint16_t           eph,
							     sensors.gps.vDop,                // uint16_t           epv,
							     sensors.gps.gSpeed,              // uint16_t           vel,
							     UINT16_MAX,                      // uint16_t           cog,
							     sensors.gps.numSats);            // uint8_t            satellites_visible);

	// Copy the message to the send buffer
	length = mavlink_msg_to_send_buffer(buffer, &msg);

	mavlinkPortPrintBinary(buffer, length);
}

///////////////////////////////////////////////////////////////////////////////

void mavlinkSendHeartbeat(void)
{
	mavlink_msg_heartbeat_pack(mavlink_system.sysid,              // uint8_t            system_id,
                               mavlink_system.compid,             // uint8_t            component_id,
                               &msg,                              // mavlink_message_t* msg,
			                   system_type,                       // uint8_t            type,
			                   autopilot_type,                    // uint8_t            autopilot,
			                   system_mode,                       // uint8_t            base_mode,
			                   custom_mode,                       // uint32_t           custom_mode,
			                   system_state);                     // uint8_t            system_status);

	// Copy the message to the send buffer
	length = mavlink_msg_to_send_buffer(buffer, &msg);

	mavlinkPortPrintBinary(buffer, length);
}

///////////////////////////////////////////////////////////////////////////////

void mavlinkSendSysStatus(void)
{
    uint32_t cpuLoad;

    cpuLoad = executionTime1000Hz +
              executionTime500Hz  / COUNT_500HZ +
              executionTime100Hz  / COUNT_100HZ +
              executionTime50Hz   / COUNT_50HZ  +
              executionTime10Hz   / COUNT_10HZ  +
              executionTime5Hz    / COUNT_5HZ   +
              executionTime1Hz    / COUNT_1HZ;

    mavlink_msg_sys_status_pack(mavlink_system.sysid,                // uint8_t            system_id,
                                mavlink_system.compid,               // uint8_t            component_id,
                                &msg,                                // mavlink_message_t* msg,
							    0x0001BC2F,                          // uint32_t           onboard_control_sensors_present,
							    0x0001BC2F,                          // uint32_t           onboard_control_sensors_enabled,
							    0x0001BC2F,                          // uint32_t           onboard_control_sensors_health,
							    (uint16_t)cpuLoad,                   // uint16_t           load,
							    (uint16_t)batteryVoltage * 1000,     // uint16_t           voltage_battery,
							    -1,                                  // int16_t            current_battery,
							    -1,                                  // int8_t             battery_remaining,
							    0,                                   // uint16_t           drop_rate_comm,
							    0,                                   // uint16_t           errors_comm,
							    0,                                   // uint16_t           errors_count1,
							    0,                                   // uint16_t           errors_count2,
							    0,                                   // uint16_t           errors_count3,
							    0);                                  // uint16_t           errors_count4)

	// Copy the message to the send buffer
    length = mavlink_msg_to_send_buffer(buffer, &msg);

    mavlinkPortPrintBinary(buffer, length);
}

///////////////////////////////////////////////////////////////////////////////

void mavlinkSendVfrHud(void)
{
    mavlink_msg_vfr_hud_pack(mavlink_system.sysid,                                // uint8_t            system_id,
                             mavlink_system.compid,                               // uint8_t            component_id,
                             &msg,                                                // mavlink_message_t* msg,
						     0.0f,                                                // float              airspeed,
						     0.0f,                                                // float              groundspeed,
						     (int16_t)(tasks500Hz_Y.attitudes[YAW] * R2D) + 180,  // int16_t            heading,
						     0,                                                   // uint16_t           throttle,
						     tasks100Hz_Y.bodyPositionEstimates[ZAXIS],           // float              alt,
						     tasks100Hz_Y.bodyVelocityEstimates[ZAXIS]);          // float              climb);

	// Copy the message to the send buffer
    length = mavlink_msg_to_send_buffer(buffer, &msg);

    mavlinkPortPrintBinary(buffer, length);
}

///////////////////////////////////////////////////////////////////////////////
