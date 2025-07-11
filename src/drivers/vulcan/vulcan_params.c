/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

/**
 * VULCAN mode
 *
 *  0 - VULCAN disabled.
 *  1 - Enables support for VULCAN sensors without dynamic node ID allocation and firmware update.
 *  2 - Enables support for VULCAN sensors with dynamic node ID allocation and firmware update.
 *  3 - Enable
 *
 * @min 0
 * @max 3
 * @value 0 Disabled
 * @value 1 Sensors Manual Config
 * @value 2 Sensors Automatic Config
 * @value 3 Eanble
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_ENABLE, 3);

/**
 * VULCAN CAN bus bitrate.
 *
 * @unit bit/s
 * @min 20000
 * @max 1000000
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_BITRATE, 1000000);

/**
 * VULCAN rangefinder minimum range
 *
 * This parameter defines the minimum valid range for a rangefinder connected via VULCAN.
 *
 * @unit m
 * @group VULCAN
 */
PARAM_DEFINE_FLOAT(VULCAN_RNG_MIN, 0.3f);

/**
 * VULCAN rangefinder maximum range
 *
 * This parameter defines the maximum valid range for a rangefinder connected via VULCAN.
 *
 * @unit m
 * @group VULCAN
 */
PARAM_DEFINE_FLOAT(VULAN_RNG_MAX, 200.0f);

/**
 * VULCAN ANTI_COLLISION light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the ANTI_COLLISION lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_LGT_ANTCL, 2);

/**
 * VULCAN STROBE light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the STROBE lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_LGT_STROB, 1);

/**
 * VULCAN RIGHT_OF_WAY light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the RIGHT_OF_WAY lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_LGT_NAV, 3);

/**
 * VULCAN LIGHT_ID_LANDING light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the LIGHT_ID_LANDING lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_LGT_LAND, 0);

/**
 * publish Arming Status stream
 *
 * Enable VULCAN Arming Status stream publication
 *  VULcan::equipment::safety::ArmingStatus
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_PUB_ARM, 0);

/**
 * publish RTCM stream
 *
 * Enable VULCAN RTCM stream publication
 *  VULcan::equipment::gnss::RTCMStream
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_PUB_RTCM, 0);

/**
 * publish moving baseline data RTCM stream
 *
 * Enable VULCAN RTCM stream publication
 *  ardupilot::gnss::MovingBaselineData
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_PUB_MBD, 0);

/**
 * subscription airspeed
 *
 * Enable VULCAN airspeed subscriptions.
 *  VULcan::equipment::air_data::IndicatedAirspeed
 *  VULcan::equipment::air_data::TrueAirspeed
 *  VULcan::equipment::air_data::StaticTemperature
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_SUB_ASPD, 0);

/**
 * subscription barometer
 *
 * Enable VULCAN barometer subscription.
 *  VULcan::equipment::air_data::StaticPressure
 *  VULcan::equipment::air_data::StaticTemperature
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_SUB_BARO, 0);

/**
 * subscription battery
 *
 * Enable VULCAN battery subscription.
 *  VULcan::equipment::power::BatteryInfo
 *  ardupilot::equipment::power::BatteryInfoAux
 *
 *  0 - Disable
 *  1 - Use raw data. Recommended for Smart battery
 *  2 - Filter the data with internal battery library
 *
 * @min 0
 * @max 2
 * @value 0 Disable
 * @value 1 Raw data
 * @value 2 Filter data
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_SUB_BAT, 0);

/**
 * subscription differential pressure
 *
 * Enable VULCAN differential pressure subscription.
 *  VULcan::equipment::air_data::RawAirData
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_SUB_DPRES, 0);

/**
 * subscription flow
 *
 * Enable VULCAN optical flow subscription.
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_SUB_FLOW, 0);

/**
 * subscription GPS
 *
 * Enable VULCAN GPS subscriptions.
 *  VULcan::equipment::gnss::Fix
 *  VULcan::equipment::gnss::Fix2
 *  VULcan::equipment::gnss::Auxiliary
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_SUB_GPS, 1);

/**
 * subscription GPS Relative
 *
 * Enable VULCAN GPS Relative subscription.
 * ardupilot::gnss::RelPosHeading
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_SUB_GPS_R, 1);

/**
 * subscription hygrometer
 *
 * Enable VULCAN hygrometer subscriptions.
 *  dronecan::sensors::hygrometer::Hygrometer
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_SUB_HYGRO, 0);

/**
 * subscription ICE
 *
 * Enable VULCAN internal combustion engine (ICE) subscription.
 *  VULcan::equipment::ice::reciprocating::Status
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_SUB_ICE, 0);

/**
 * subscription IMU
 *
 * Enable VULCAN IMU subscription.
 *  VULcan::equipment::ahrs::RawIMU
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_SUB_IMU, 0);

/**
 * subscription magnetometer
 *
 * Enable VULCAN mag subscription.
 *  VULcan::equipment::ahrs::MagneticFieldStrength
 *  VULcan::equipment::ahrs::MagneticFieldStrength2
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_SUB_MAG, 1);

/**
 * subscription range finder
 *
 * Enable VULCAN range finder subscription.
 *  VULcan::equipment::range_sensor::Measurement
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_SUB_RNG, 0);

/**
 * subscription buttonadw
 *
 * Enable VULCAN button subscription.
 *  ardupilot::indication::Button
 *
 * @boolean
 * @reboot_required true
 * @group VULCAN
 */
PARAM_DEFINE_INT32(VULCAN_SUB_BTN, 0);
