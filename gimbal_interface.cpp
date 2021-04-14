/*******************************************************************************
 * @file    gimbal_interface.cpp
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    August-21-2018
 * @brief   This file contains API for gimbal interface.
 *
 *  @Copyright (c) 2018 Gremsy
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************/

 /* Includes ------------------------------------------------------------------*/

#include "gimbal_interface.h"

// Gimbal status 1
typedef enum {
  STATUS1_ERROR_NONE          = 0x00,
  STATUS1_MODE_FOLLOW_LOCK    = 0x01,
  STATUS1_MISS_STEP           = 0x02,
  STATUS1_SENSOR_ERROR        = 0x04,
  STATUS1_BATT_LOW            = 0x08,
  STATUS1_MOTORS              = 0x10,         /// motors on = 1, motor off = 0 (fimware 1.3.4)*/
  STATUS1_INIT_MOTOR          = 0x20,
  STATUS1_AUTO_TUNER          = 0x40,         /// 0b0100 0000
  STATUS1_CANLINK             = 0x80,         /// 0b1000 0000 ket noi can link.
  STATUS1_SEARCH_HOME         = 0x100,        /// search home
  STATUS1_SET_HOME            = 0x200,        /// set home
  STATUS1_SENSOR_CALIB        = 0x400,        /// calib sensor gom accel va gyro
  STATUS1_STARTUP             = 0x800,
  STATUS1_REMOTE              = 0x1000,
  STATUS1_INVERTED            = 0x2000,
  STATUS1_MOTOR_PHASE_ERROR   = 0x4000,
  STATUS1_MOTOR_ANGLE_ERROR   = 0x8000,
} status1_t;

// Gimbal status 2
typedef enum {
  STATUS2_ERROR_NONE              = 0x00,
  STATUS2_IMU_ERROR               = 0x01,
  STATUS2_MOTOR_TILT_ERROR        = 0x02,
  STATUS2_MOTOR_ROLL_ERROR        = 0x04,
  STATUS2_MOTOR_PAN_ERROR         = 0x08,
  STATUS2_JOYSTICK_ERROR          = 0x10,
  STATUS2_INVERTED_ERROR          = 0x20,
  STATUS2_PAN_SEARCH_HOME_ERROR   = 0x40,
  STATUS2_ANGLE_TILT_ERROR        = 0x80,
  STATUS2_ANGLE_ROLL_ERROR        = 0x100,
  STATUS2_ANGLE_PAN_ERROR         = 0x200,
  STATUS2_MOVE_TILT_ERROR         = 0x400,
  STATUS2_MOVE_ROLL_ERROR         = 0x800,
  STATUS2_MOVE_PAN_ERROR          = 0x1000,
} status2_t;

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t get_time_usec() {
  static struct timeval _time_stamp;
  gettimeofday(&_time_stamp, NULL);
  return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

uint64_t get_time_msec() {
  static struct timeval _time_stamp;
  gettimeofday(&_time_stamp, NULL);
  return _time_stamp.tv_sec*1000 + _time_stamp.tv_usec;
}

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Gimbal_Interface::Gimbal_Interface(Serial_Port* serial_port_) {

  // initialize attributes
  write_count = 0;

  reading_status = 0; // whether the read thread is running
  writing_status = 0; // whether the write thread is running
  exit_signalled = false; // flag to signal thread exit
  heartbeat_detected = false;	// flag to detect gimbal

  read_tid = 0; // read thread id
  write_tid = 0; // write thread id

  system_id = 0; // system id
  gimbal_id = MAV_COMP_ID_GIMBAL; // gimbal component id
  component_id = MAV_COMP_ID_SYSTEM_CONTROL; // companion computer component id

  last_message.sysid = system_id;
  last_message.compid = gimbal_id;
  last_message.sys_status.errors_count2 = 0;
  last_message.sys_status.errors_count1 = 0;

  gimbal_state = GIMBAL_STATE_NOT_PRESENT;

  serial_port = serial_port_; // serial port management object
}

Gimbal_Interface::~Gimbal_Interface() {}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void Gimbal_Interface::read_messages() {

  bool success; // receive success flag
  bool received_all = false;  // receive only one message

  uint8_t last_message_seq = 255;

  // Blocking wait for new data
  while(!exit_signalled) {

    mavlink_message_t message;
    success = serial_port->read_message(message);

    if(success && (message.seq != last_message_seq)) {

      last_message_seq = message.seq;
      
      //printf(
      //  "Message: ID = %d, seq = %d, length = %d\n",
      //  message.msgid,
      //  message.seq,
      //  message.len
      //);

      switch(message.msgid) {

        case MAVLINK_MSG_ID_HEARTBEAT:
        {
          printf("  MAVLINK_MSG_ID_HEARTBEAT. seq = %d ", message.seq);
          mavlink_msg_heartbeat_decode(&message, &(last_message.heartbeat));
          last_message.time_stamps.heartbeat = get_time_usec();
          //time_of_last_heartbeart_us = get_time_usec();
          mavlink_status_t* channel_status = mavlink_get_channel_status(MAVLINK_COMM_1);

          // If this is the first time we have detected the heartbeat, store the system and component IDs.
          if(heartbeat_detected == false) {
            last_message.sysid = message.sysid;
            last_message.compid = message.compid;
            heartbeat_detected = true;
            time_of_first_heartbeart_us = last_message.time_stamps.heartbeat;
            printf(
              "First heartbeat detected. System ID = %d. Component ID = %d. Time (us since epoch) = %ld. ", 
              message.sysid, 
              message.compid,
              time_of_first_heartbeart_us
            );
          }

          float seconds_since_first_heartbeat = ((float) (last_message.time_stamps.heartbeat - time_of_first_heartbeart_us)) / 1000000.0f;
          //printf("Time since first heartbeat (s) = %.3f\n", seconds_since_first_heartbeat);
          break;
        }

        case MAVLINK_MSG_ID_SYS_STATUS:
        {
          mavlink_msg_sys_status_decode(&message, &(last_message.sys_status));
          last_message.time_stamps.sys_status = get_time_usec();
          mavlink_status_t* channel_status = mavlink_get_channel_status(MAVLINK_COMM_1);

          //printf(
          //  "  MAVLINK_MSG_ID_SYS_STATUS. control_sensors_enabled = %d, control_sensors_health = %d, control_sensors_present = %d\n",
          //  last_message.sys_status.onboard_control_sensors_enabled,
          //  last_message.sys_status.onboard_control_sensors_health,
          //  last_message.sys_status.onboard_control_sensors_present
          //);
          break;
        }

        case MAVLINK_MSG_ID_MOUNT_STATUS:
        {
          mavlink_msg_mount_status_decode(&message, &(last_message.mount_status));
          last_message.time_stamps.mount_status = get_time_usec();
          mavlink_status_t* channel_status = mavlink_get_channel_status(MAVLINK_COMM_1);

          printf(
            "  MAVLINK_MSG_ID_MOUNT_STATUS. YPR = [%d, %d, %d]\n",
            last_message.mount_status.pointing_c,
            last_message.mount_status.pointing_a,
            last_message.mount_status.pointing_b
          );
          break;
        }

        case MAVLINK_MSG_ID_MOUNT_ORIENTATION:
        {
          mavlink_msg_mount_orientation_decode(&message, &(last_message.mount_orientation));
          last_message.time_stamps.mount_orientation = get_time_usec();
          mavlink_status_t* channel_status = mavlink_get_channel_status(MAVLINK_COMM_1);

          printf(
            "  MAVLINK_MSG_ID_MOUNT_ORIENTATION. yaw = %.1f, pitch = %.1f, roll = %.1f, absolute yaw = %.1f\n",
            last_message.mount_orientation.yaw,
            last_message.mount_orientation.pitch,
            last_message.mount_orientation.roll,
            last_message.mount_orientation.yaw_absolute
          );
          break;
        }

        case MAVLINK_MSG_ID_RAW_IMU:
        {
          mavlink_msg_raw_imu_decode(&message, &(last_message.raw_imu));
          last_message.time_stamps.raw_imu = get_time_usec();
          mavlink_status_t* channel_status = mavlink_get_channel_status(MAVLINK_COMM_1);

          //printf(
          //  "  MAVLINK_MSG_ID_RAW_IMU. accelerometer XYZ: [%d, %d, %d], gyro XYZ: [%d, %d, %d]\n", // , mag-xyz: [%d, %d, %d]
          //  last_message.raw_imu.xacc,
          //  last_message.raw_imu.yacc,
          //  last_message.raw_imu.zacc,
          //  last_message.raw_imu.xgyro,
          //  last_message.raw_imu.ygyro,
          //  last_message.raw_imu.zgyro,
          //  //last_message.raw_imu.xmag,
          //  //last_message.raw_imu.ymag,
          //  //last_message.raw_imu.zmag
          //);
          break;
        }

        case MAVLINK_MSG_ID_COMMAND_ACK:
        {
          mavlink_command_ack_t packet;
          mavlink_msg_command_ack_decode(&message, &packet);
          last_message.time_stamps.command_ack = get_time_usec();
          mavlink_status_t* channel_status = mavlink_get_channel_status(MAVLINK_COMM_1);

          if(packet.command == MAV_CMD_DO_MOUNT_CONFIGURE) {
            last_message.result_cmd_ack_msg_configure = packet.progress; // was packet.result;
          } else if(packet.command == MAV_CMD_DO_MOUNT_CONTROL) {
            last_message.result_cmd_ack_msg_control = packet.progress; // was packet.result;
          }

          //printf(
          //  "  MAVLINK_MSG_ID_COMMAND_ACK. seq = %d, command = %d, progress = %d, result = %d\n",
          //  message.seq,
          //  packet.command,
          //  packet.progress,
          //  packet.result
          //);
          break;
        }

        case MAVLINK_MSG_ID_PARAM_VALUE:
        {
          mavlink_param_value_t packet;
          mavlink_msg_param_value_decode(&message, &packet);

          printf(
            "  MAVLINK_MSG_ID_PARAM_VALUE. seq = %d, ID = %s, index = %d, type = %d, value = %f\n",
            message.seq,
            packet.param_id,
            packet.param_index,
            packet.param_type,
            packet.param_value
          );

          // Loop over all parameters until you find one with an index that matches that from the message.
          for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
            if(packet.param_index == _params_list[i].gmb_idx) {

              _params_list[i].seen = true;

              switch(_params_list[i].state) {

                case PARAM_STATE_NONEXISTANT:
                  //printf("  Got parameter (PARAM_STATE_NONEXISTANT, i = %d) %s = %d\n", i, get_param_name((param_index_t) i), _params_list[i].value);

                case PARAM_STATE_NOT_YET_READ:
                  //printf("  Got parameter (PARAM_STATE_NOT_YET_READ, i = %d) %s = %d\n", i, get_param_name((param_index_t) i), _params_list[i].value);

                case PARAM_STATE_FETCH_AGAIN:
                  //printf("  Got parameter (fetch again) %s = %d\n", get_param_name((param_index_t) i), _params_list[i].value);
                  _params_list[i].value = packet.param_value;
                  _params_list[i].state = PARAM_STATE_CONSISTENT;
                  break;

                case PARAM_STATE_CONSISTENT:
                  //printf("  Got parameter (consistent) %s = %d\n", get_param_name((param_index_t) i), _params_list[i].value);
                  _params_list[i].value = (int16_t) packet.param_value;
                  break;

                case PARAM_STATE_ATTEMPTING_TO_SET:
                  //printf("  Got parameter (attempting to set) %s = %d\n", get_param_name((param_index_t) i), _params_list[i].value);
                  if(packet.param_value == _params_list[i].value) { _params_list[i].state = PARAM_STATE_CONSISTENT; }
                  break;

              } // switch _params_list[i].state

            } // if
          } // for

          break;
        }

        default:
        {
          printf("Warning, did not handle message id %i\n", message.msgid);
          break;
        }

      } // end: switch msgid

    } // end: if read message

    // Give the write thread time to use the port
    if(writing_status > false) { usleep(100); }

  } // end: while not received all

  return;
}

int Gimbal_Interface::write_message(mavlink_message_t message) {
  int len = serial_port->write_message(message);
  write_count++;
  return len;
}

void Gimbal_Interface::reset_params() {

  _last_request_ms	= 0;

  for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
    _params_list[i].value = 0;
    _params_list[i].state = PARAM_STATE_NOT_YET_READ;
    _params_list[i].fetch_attempts = 0;
    _params_list[i].seen = 0;
  }
}

void Gimbal_Interface::fetch_params() {

  for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
    /// Check the param has been read before
    if(_params_list[i].state != PARAM_STATE_NOT_YET_READ) {
      // Then set the state to allow read again
      _params_list[i].state = PARAM_STATE_FETCH_AGAIN;
    }
  }
}

bool Gimbal_Interface::params_initialized() {

  for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
    if(_params_list[i].state == PARAM_STATE_NOT_YET_READ) { return false; }
  }

  return true;
}

bool Gimbal_Interface::params_received_all() {

  for(uint8_t i=0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
    if(_params_list[i].state == PARAM_STATE_NOT_YET_READ || _params_list[i].state == PARAM_STATE_FETCH_AGAIN) {
      return false;
    }
  }

  return true;
}

void Gimbal_Interface::get_param(param_index_t param, int16_t &value, int16_t val) {
  if(!_params_list[param].seen) {
    printf("Warning: Parameter not seen, get_param returning default value.\n");
    value = val;
  } else {
    value = _params_list[param].value;
  }
}

/* Transmits a message to the gimbal to set a parameter. */
void Gimbal_Interface::set_param(param_index_t param, int16_t value) {

  if(_params_list[param].state == PARAM_STATE_NONEXISTANT) {
    printf("Warning: parameter %d does not exist\n", param);
    exit(-1);
  }

  // If the parameter is consistent and already at the requested value, skip sending the message.
  if((_params_list[param].state == PARAM_STATE_CONSISTENT) && (_params_list[param].value == value)) { return; }

  // Update the local parameter value and status.
  _params_list[param].value = value;
  _params_list[param].state = PARAM_STATE_ATTEMPTING_TO_SET;

  // Prepare command for off-board mode
  mavlink_param_set_t param_set = {0};
  param_set.param_value	= value;
  param_set.target_system	= system_id;
  param_set.target_component = gimbal_id;
  mav_array_memcpy(param_set.param_id, get_param_name(param), sizeof(char)*16);
  param_set.param_type = MAVLINK_TYPE_UINT16_T;
  mavlink_message_t message;
  mavlink_msg_param_set_encode(system_id, component_id, &message, &param_set);

  _last_request_ms = get_time_msec();
  int len = write_message(message);
  if(len <= 0) { fprintf(stderr, "WARNING: could not set param: %s\n", get_param_name(param)); }
}

// Queries the gimbal to update the local values of all gimbal parameters.
void Gimbal_Interface::param_update() {

  uint32_t tnow_ms = get_time_msec();

  /// Retry initial param retrieval
  if(!params_received_all()) {

    for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {

      if(!_params_list[i].seen) {
        mavlink_param_request_read_t request ={0};

        request.target_system = system_id;
        request.target_component = gimbal_id;
        request.param_index = _params_list[i].gmb_idx;

        mav_array_memcpy(request.param_id, get_param_name((param_index_t)i), sizeof(char)*16);
        //printf("Request param read: %s \n", get_param_name((param_index_t)i));

        mavlink_message_t message;
        mavlink_msg_param_request_read_encode(system_id, component_id, &message, &request);

        int len = write_message(message);
        if(len <= 0) { fprintf(stderr, "WARNING: could not send Request list to gimbal\n"); }

        // Send request read try again
        _params_list[i].fetch_attempts++;

        // Waing to read
        // usleep(100000);
      }
    }
  }

  // retry param set
  for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {

    if((_params_list[i].state == PARAM_STATE_ATTEMPTING_TO_SET) && (tnow_ms - _last_request_ms > _retry_period)) {

      _last_request_ms = tnow_ms;

      // Prepare command for off-board mode
      mavlink_param_set_t param_set ={0};
      param_set.param_value	= _params_list[i].value;
      param_set.target_system	= system_id;
      param_set.target_component = gimbal_id;
      mav_array_memcpy(param_set.param_id, get_param_name((param_index_t)i), sizeof(char)*16);
      param_set.param_type = MAVLINK_TYPE_UINT16_T;

      mavlink_message_t message;
      mavlink_msg_param_set_encode(system_id, component_id, &message, &param_set);

      int len = write_message(message);
      if(len <= 0) { fprintf(stderr, "WARNING: could not Set param \n"); }

      if(!_params_list[i].seen) {
        // Send request read
        _params_list[i].fetch_attempts++;
      }
    }
  }

  // Check for nonexistent parameters
  for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
    if(!_params_list[i].seen && (_params_list[i].fetch_attempts > _max_fetch_attempts)) {
      _params_list[i].state = PARAM_STATE_NONEXISTANT;
      printf("Gimbal parameter %s timed out\n", get_param_name((param_index_t) i));
    }
  }
}


// ------------------------------------------------------------------------------
//   Process when gimbal connected
// ------------------------------------------------------------------------------
void Gimbal_Interface::param_process(void) {

  if(!get_connection()) { gimbal_state = GIMBAL_STATE_NOT_PRESENT; }

  switch(gimbal_state) {

    case GIMBAL_STATE_NOT_PRESENT: // gimbal was just connected or we just rebooted    
      printf("GIMBAL_STATE_NOT_PRESENT\n");
      reset_params();
      gimbal_state = GIMBAL_STATE_PRESENT_INITIALIZING;
      break;

    case GIMBAL_STATE_PRESENT_INITIALIZING:
      printf("GIMBAL_STATE_PRESENT_INITIALIZING\n");
      param_update();
      if(params_initialized()) {
        for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
          printf("Check [%s] %d \n", _params_list[i].gmb_id, _params_list[i].value);
        }
        gimbal_state = GIMBAL_STATE_PRESENT_ALIGNING;
      }
      break;

    case GIMBAL_STATE_PRESENT_ALIGNING:
      printf("GIMBAL_STATE_PRESENT_ALIGNING\n");
      param_update();
      if(last_message.sys_status.errors_count2 == 0x00) {
        gimbal_state = GIMBAL_STATE_PRESENT_RUNNING;
        printf("GIMBAL_STATE_PRESENT_RUNNING \n");
      } else {
        printf("Error: %d\n", last_message.sys_status.errors_count2);
      }
      break;

    case GIMBAL_STATE_PRESENT_RUNNING:
      //printf("GIMBAL_STATE_PRESENT_RUNNING \n");
      param_update();
      break;
  }
}



// ------------------------------------------------------------------------------
//   GIMBAL Command
// ------------------------------------------------------------------------------

/**
 * @brief  This function shall reboot the gimbal
 * @param: NONE
 * @ret: None
 */
void Gimbal_Interface::set_gimbal_reboot(void) {

  // Prepare command for off-board mode
  mavlink_message_t message;
  mavlink_command_long_t command = {0};
  command.target_system = system_id;
  command.target_component = gimbal_id;
  command.command = MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
  command.param1 = 0;
  command.param2 = 0;
  command.param3 = 0;
  command.param4 = 1;
  command.param5 = 0;
  command.param6 = 0;
  command.param7 = 0;
  command.confirmation = true;
  mavlink_msg_command_long_encode(system_id, component_id, &message, &command);

  // --------------------------------------------------------------------------
  //   WRITE
  // --------------------------------------------------------------------------

  // do the write
  int len = write_message(message);

  // check the write
  if(len <= 0)
    fprintf(stderr, "WARNING: could not send GIMBAL REBOOT \n");
  else
    // printf("%lu GIMBAL_REBOOT = [ %d] \n", write_count);
    return;
}

// Turn the motor on or off (see control_gimbal_motor_t).
void Gimbal_Interface::set_gimbal_motor_mode(control_gimbal_motor_t type) {

  // Prepare command for off-board mode
  mavlink_command_long_t command = {0};
  command.target_system = system_id;
  command.target_component = gimbal_id;
  command.command = MAV_CMD_USER_1;
  command.param7 = type;	// type 0 =>off , 1=>on
  command.confirmation = true;

  // --------------------------------------------------------------------------
  //   ENCODE
  // --------------------------------------------------------------------------

  mavlink_message_t message;
  mavlink_msg_command_long_encode(system_id, component_id, &message, &command);

  // --------------------------------------------------------------------------
  //   WRITE
  // --------------------------------------------------------------------------

  // do the write
  int len = write_message(message);

  // check the write
  if(len <= 0)
    fprintf(stderr, "WARNING: could not send MOTOR MODE \n");
  else
    // printf("%lu MOTOR_MODE  = [ %d] \n", write_count, type);

    return;
}

/**
 * @brief  This function shall turn on/off motor driver
 * @param: type see control_gimbal_motor_t
 * @ret: None
 */
void Gimbal_Interface::set_gimbal_mode(control_gimbal_mode_t mode) {

  // Prepare command for off-board mode
  mavlink_command_long_t command = {0};
  command.target_system = system_id;
  command.target_component = gimbal_id;
  command.command = MAV_CMD_USER_2;
  command.param7 = mode;
  command.confirmation = false;

  mavlink_message_t message;
  mavlink_msg_command_long_encode(system_id, component_id, &message, &command);

  // do the write
  int len = write_message(message);
  if(len <= 0)
    fprintf(stderr, "WARNING: could not send GIMBAL MODE \n");
  else
    // printf("%lu GIMBAL_MODE  = [ %d] \n", write_count, mode);
    return;
}

void Gimbal_Interface::set_gimbal_axes_mode(
  control_gimbal_axis_mode_t tilt,
  control_gimbal_axis_mode_t roll,
  control_gimbal_axis_mode_t pan) {

  /*Default all axes that stabilize mode */
  roll.stabilize = 1;
  tilt.stabilize = 1;
  pan.stabilize = 1;

  // Prepare command for off-board mode
  mavlink_command_long_t command = {0};
  command.target_system = system_id;
  command.target_component = gimbal_id;
  command.confirmation = false;
  command.command = MAV_CMD_DO_MOUNT_CONFIGURE;
  command.param1 = MAV_MOUNT_MODE_MAVLINK_TARGETING;
  command.param2 = roll.stabilize;
  command.param3 = tilt.stabilize;
  command.param4 = pan.stabilize;
  command.param5 = roll.input_mode;
  command.param6 = tilt.input_mode;
  command.param7 = pan.input_mode;

  mavlink_message_t message;
  mavlink_msg_command_long_encode(system_id, component_id, &message, &command);

  int len = write_message(message);
  if(len <= 0)
    fprintf(stderr, "WARNING: could not send GIMBAL AXES MODE \n");
  else
    // printf("%lu GIMBAL_AXES_MODE \n", write_count);
    return;
}

void Gimbal_Interface::set_gimbal_move(float tilt, float roll, float pan) {

  // Prepare command for off-board mode
  mavlink_command_long_t command = {0};
  command.target_system = system_id;
  command.target_component = gimbal_id;
  command.command = MAV_CMD_DO_MOUNT_CONTROL;
  command.confirmation = true;
  command.param1 = tilt;
  command.param2 = roll;
  command.param3 = -pan;
  command.param7 = (float) MAV_MOUNT_MODE_MAVLINK_TARGETING;

  mavlink_message_t message;
  mavlink_msg_command_long_encode(system_id, component_id, &message, &command);

  int len = write_message(message);
  if(len <= 0)
    fprintf(stderr, "WARNING: could not send GIMBAL AXES MODE \n");
  else
    // printf("%lu GIMBAL_AXES_MODE \n", write_count);
    return;
}

/**
 * @brief  This function set motor controls setting
 * @param: tilt, roll, pan - stiffness and holdstrengtg, see user_manual (https://gremsy.com/gremsy-t3-manual/)
 * @param: gyro_filter - The coefficent for denoising the sensor filter
 * @param: output_filter - The coefficent for denoising the output filter
 * @param: gain - Defines how fast each axis will return to commanded position.
 * @ret: gimbal_motor_control_t contains setting related to tilt axis
 *
 *
 *	GYRO FILTER 	2
 *	OUTPUT FILTER 	3
 *
 *	HOLD STRENGTH 	TILT 	ROLL 	PAN
 *					40 		40 		40
 * 	GAIN 			120		120		120
 */
void Gimbal_Interface::set_gimbal_motor_control(
  gimbal_motor_control_t tilt,
  gimbal_motor_control_t roll,
  gimbal_motor_control_t pan,
  uint8_t gyro_filter,
  uint8_t output_filter,
  uint8_t gain) {

  set_param(GMB_PARAM_STIFFNESS_PITCH, (int16_t)tilt.stiffness);
  set_param(GMB_PARAM_HOLDSTRENGTH_PITCH, (int16_t)tilt.holdstrength);
  set_param(GMB_PARAM_STIFFNESS_ROLL, (int16_t)roll.stiffness);
  set_param(GMB_PARAM_HOLDSTRENGTH_ROLL, (int16_t)roll.holdstrength);
  set_param(GMB_PARAM_STIFFNESS_YAW, (int16_t)pan.stiffness);
  set_param(GMB_PARAM_HOLDSTRENGTH_YAW, (int16_t)pan.holdstrength);

  set_param(GMB_PARAM_OUTPUT_FILTER, (int16_t)output_filter);
  set_param(GMB_PARAM_GYRO_FILTER, (int16_t)gyro_filter);
  set_param(GMB_PARAM_GAIN, (int16_t)gain);
}

/**
 * @brief  This function get motor controls setting
 * @param: tilt, roll, pan - stiffness and holdstrengtg, see user_manual (https://gremsy.com/gremsy-t3-manual/)
 * @param: gyro_filter - The coefficent for denoising the sensor filter
 * @param: output_filter - The coefficent for denoising the output filter
 * @param: gain - Defines how fast each axis will return to commanded position.
 * @ret: gimbal_motor_control_t contains setting related to tilt axis
 *
 *	GYRO FILTER 	2
 *	OUTPUT FILTER 	3
 *
 *	HOLD STRENGTH 	TILT 	ROLL 	PAN
 *					40 		40 		40
 * 	GAIN 			120		120		120
 */
void Gimbal_Interface::get_gimbal_motor_control(
  gimbal_motor_control_t &tilt,
  gimbal_motor_control_t &roll,
  gimbal_motor_control_t &pan,
  uint8_t &gyro_filter,
  uint8_t &output_filter,
  uint8_t &gain) {

  int16_t value = 0;

  get_param(GMB_PARAM_STIFFNESS_PITCH, value);
  tilt.stiffness = (uint8_t) value;
  get_param(GMB_PARAM_HOLDSTRENGTH_PITCH, value);
  tilt.holdstrength = (uint8_t) value;

  get_param(GMB_PARAM_STIFFNESS_ROLL, value);
  roll.stiffness = (uint8_t) value;
  get_param(GMB_PARAM_HOLDSTRENGTH_ROLL, value);
  roll.holdstrength = (uint8_t) value;

  get_param(GMB_PARAM_STIFFNESS_YAW, value);
  pan.stiffness = (uint8_t) value;
  get_param(GMB_PARAM_HOLDSTRENGTH_YAW, value);
  pan.holdstrength = (uint8_t) value;

  get_param(GMB_PARAM_OUTPUT_FILTER, value);
  output_filter	= (uint8_t) value;
  get_param(GMB_PARAM_GYRO_FILTER, value);
  gyro_filter = (uint8_t) value;
  get_param(GMB_PARAM_GAIN, value);
  gain= (uint8_t) value;
}

/**
 * @brief  This function shall configure on the tilt axis
 *
 * @param: config - see gimbal_config_axis_t structure
 * @note: The smooth starts with a low value of 50
 *			Slowly increase this setting until you feel an oscillation in the tilt axis,
 *			then reduce the setting until the oscillation subsides.
 * @ret: None
 */
void Gimbal_Interface::set_gimbal_config_tilt_axis(gimbal_config_axis_t config) {

  set_param(GMB_PARAM_SMOOTH_CONTROL_PITCH, (int16_t) config.smooth_control);
  set_param(GMB_PARAM_SMOOTH_FOLLOW_PITCH, (int16_t) config.smooth_follow);
  set_param(GMB_PARAM_WINDOW_FOLLOW_PITCH, (int16_t) config.window_follow);
  set_param(GMB_PARAM_SPEED_FOLLOW_PITCH, (int16_t) config.speed_follow);
  set_param(GMB_PARAM_SPEED_CONTROL_PITCH, (int16_t) config.speed_control);

  int16_t get_dir;
  get_param(GMB_PARAM_AXIS_DIR, get_dir);

  if(config.dir == DIR_CCW) {
    get_dir = get_dir | 0x01;
  } else {
    get_dir &= (~0x01);
  }

  set_param(GMB_PARAM_AXIS_DIR, get_dir);
}

/**
 * @brief  This function shall return the setting on the tilt axis
 *
 * @param: none
 * @ret: see structure gimbal_config_axis_t
 */
gimbal_config_axis_t Gimbal_Interface::get_gimbal_config_tilt_axis(void) {

  gimbal_config_axis_t setting;

  int16_t ret;

  get_param(GMB_PARAM_SMOOTH_CONTROL_PITCH, ret);
  setting.smooth_control = (uint8_t) ret;

  get_param(GMB_PARAM_SMOOTH_FOLLOW_PITCH, ret);
  setting.smooth_follow = (uint8_t) ret;

  get_param(GMB_PARAM_WINDOW_FOLLOW_PITCH, ret);
  setting.window_follow = (uint8_t) ret;

  get_param(GMB_PARAM_SPEED_FOLLOW_PITCH, ret);
  setting.speed_follow = (uint8_t) ret;

  get_param(GMB_PARAM_SPEED_CONTROL_PITCH, ret);
  setting.speed_control = (uint8_t) ret;

  get_param(GMB_PARAM_AXIS_DIR, ret);

  if(ret & 0x01) {
    setting.dir = DIR_CCW;
  } else if(!(ret & 0x01)) {
    setting.dir = DIR_CW;
  }

  return setting;
}


/**
 * @brief  This function shall configure for pan axis
 *
 * @param: none
 * @ret: see structure gimbal_config_axis_t
 */
void Gimbal_Interface::set_gimbal_config_pan_axis(gimbal_config_axis_t config) {

  set_param(GMB_PARAM_SMOOTH_CONTROL_YAW, (int16_t) config.smooth_control);
  set_param(GMB_PARAM_SMOOTH_FOLLOW_YAW, (int16_t) config.smooth_follow);
  set_param(GMB_PARAM_WINDOW_FOLLOW_YAW, (int16_t) config.window_follow);
  set_param(GMB_PARAM_SPEED_FOLLOW_YAW, (int16_t) config.speed_follow);
  set_param(GMB_PARAM_SPEED_CONTROL_YAW, (int16_t) config.speed_control);

  int16_t get_dir;
  get_param(GMB_PARAM_AXIS_DIR, get_dir);

  if(config.dir == DIR_CCW) {
    get_dir = get_dir | 0x02;
  } else {
    get_dir &= (~0x02);
  }

  set_param(GMB_PARAM_AXIS_DIR, get_dir);
}
/**
 * @brief  This function shall return the setting on the pan axis
 *
 * @param: none
 * @ret: see structure gimbal_config_axis_t
 */
gimbal_config_axis_t Gimbal_Interface::get_gimbal_config_pan_axis(void) {

  gimbal_config_axis_t setting;
  int16_t ret;

  get_param(GMB_PARAM_SMOOTH_CONTROL_YAW, ret);
  setting.smooth_control = (uint8_t) ret;

  get_param(GMB_PARAM_SMOOTH_FOLLOW_YAW, ret);
  setting.smooth_follow = (uint8_t) ret;

  get_param(GMB_PARAM_WINDOW_FOLLOW_YAW, ret);
  setting.window_follow = (uint8_t) ret;

  get_param(GMB_PARAM_SPEED_FOLLOW_YAW, ret);
  setting.speed_follow = (uint8_t) ret;

  get_param(GMB_PARAM_SPEED_CONTROL_YAW, ret);
  setting.speed_control = (uint8_t) ret;

  get_param(GMB_PARAM_AXIS_DIR, ret);

  if(ret & 0x02) {
    setting.dir = DIR_CCW;
  } else if(!(ret & 0x02)) {
    setting.dir = DIR_CW;
  }

  return setting;
}

/**
 * @brief  This function shall configure for roll axis
 *
 * @param: none
 * @ret: see structure gimbal_config_axis_t
 */
void Gimbal_Interface::set_gimbal_config_roll_axis(gimbal_config_axis_t config) {

  set_param(GMB_PARAM_SMOOTH_CONTROL_ROLL, (int16_t) config.smooth_control);
  set_param(GMB_PARAM_SPEED_CONTROL_ROLL, (int16_t) config.speed_control);

  int16_t get_dir;
  get_param(GMB_PARAM_AXIS_DIR, get_dir);

  if(config.dir == DIR_CCW) {
    get_dir = get_dir | 0x04;
  } else {
    get_dir &= (~0x04);
  }

  set_param(GMB_PARAM_AXIS_DIR, get_dir);
}

/**
 * @brief  This function shall return the setting on the roll axis
 *
 * @param: none
 * @ret: see structure gimbal_config_axis_t
 */
gimbal_config_axis_t Gimbal_Interface::get_gimbal_config_roll_axis(void) {

  gimbal_config_axis_t setting;
  int16_t ret;

  get_param(GMB_PARAM_SMOOTH_CONTROL_ROLL, ret);
  setting.smooth_control = (uint8_t) ret;

  // Roll dosen't support in follow mode
  setting.smooth_follow = 0;
  setting.window_follow = 0;
  setting.speed_follow = 0;

  get_param(GMB_PARAM_SPEED_CONTROL_ROLL, ret);
  setting.speed_control = (uint8_t) ret;

  get_param(GMB_PARAM_AXIS_DIR, ret);
  if(ret & 0x04) {
    setting.dir = DIR_CCW;
  } else if(!(ret & 0x04)) {
    setting.dir = DIR_CW;
  }

  return setting;
}

/**
 * @brief  Sets the configuration the message mavink with rate.
 *
 * @param: emit_heatbeat - enable the heartbeat when lost connection or not enable = 1, disable = 0
 * @param: status_rate - the time rate of the system status. Gimbal sends as default 10Hz
 * @param: enc_value_rate - the time rate of the encoder values. Gimbal sends as default 50Hz
 * @param: enc_type_send - Set the type of encoder has been sent from gimbal is angle or count (Resolution 2^16)
 * @param: orien_rate - the time rate of the mount orientation of gimbal.Gimbal sends as default 50Hz
 * @param: imu_rate - the time rate of the raw_imu value. Gimbal sends as default 10Hz
 * @NOTE The range [0 - 100Hz]. 0 will disable that message
 * @ret: None
 */
void Gimbal_Interface::set_gimbal_config_mavlink_msg(
  uint8_t emit_heatbeat,
  uint8_t status_rate,
  uint8_t enc_value_rate,
  uint8_t enc_type_send,
  uint8_t orien_rate,
  uint8_t imu_rate) {

  set_param(GMB_PARAM_HEATBEAT_EMIT, (int16_t) emit_heatbeat);
  set_param(GMB_PARAM_STATUS_RATE, (int16_t) status_rate);
  set_param(GMB_PARAM_ENCODER_VALUE_RATE, (int16_t) enc_value_rate);
  set_param(GMB_PARAM_ENCODER_TYPE, (int16_t) enc_type_send);
  set_param(GMB_PARAM_ORIENTATION_RATE, (int16_t) orien_rate);
  set_param(GMB_PARAM_RAW_IMU_RATE, (int16_t) imu_rate);
}

/* Returns the _currently_ known values of the gimbal message emission rates. 
 * Does not query the gimbal again to obtain the most recent values.
 */
config_mavlink_message_t Gimbal_Interface::get_gimbal_config_mavlink_msg(void) {

  config_mavlink_message_t config;
  int16_t value;

  get_param(GMB_PARAM_HEATBEAT_EMIT, value);
  config.emit_heatbeat = (uint8_t) value;

  get_param(GMB_PARAM_STATUS_RATE, value);
  config.status_rate = (uint8_t) value;

  get_param(GMB_PARAM_ENCODER_VALUE_RATE, value);
  config.enc_value_rate = (uint8_t) value;

  get_param(GMB_PARAM_ENCODER_TYPE, value);
  config.enc_type_send = (uint8_t) value;

  get_param(GMB_PARAM_ORIENTATION_RATE, value);
  config.orientation_rate	= (uint8_t) value;

  get_param(GMB_PARAM_RAW_IMU_RATE, value);
  config.imu_rate	= (uint8_t) value;

  return config;
}

gimbal_status_t Gimbal_Interface::get_gimbal_status(void) {

  /* Check gimbal status has changed*/
  if(last_message.time_stamps.sys_status) {

    // Get gimbal status 
    uint16_t errors_count1 = last_message.sys_status.errors_count1;
    uint16_t errors_count2 = last_message.sys_status.errors_count2;

    /* Check gimbal's motor */
    if(errors_count1 & STATUS1_MOTORS) {

      this->gimbal_status.state = GIMBAL_STATE_ON;

      /* Check gimbal is follow mode*/
      if(errors_count1 & STATUS1_MODE_FOLLOW_LOCK) {
        this->gimbal_status.mode = GIMBAL_STATE_FOLLOW_MODE;
      } else {
        this->gimbal_status.mode = GIMBAL_STATE_LOCK_MODE;
      }

    } else if(not (errors_count1 & STATUS1_MOTORS)) {
      this->gimbal_status.state = GIMBAL_STATE_OFF;
      this->gimbal_status.mode = GIMBAL_STATE_OFF;
    } else if(errors_count1 & STATUS1_INIT_MOTOR) { /* Check gimbal is initializing*/
      this->gimbal_status.state = GIMBAL_STATE_INIT;
    } else if(
      (errors_count1 & STATUS1_SENSOR_ERROR) ||
      (errors_count1 & STATUS1_MOTOR_PHASE_ERROR) ||
      (errors_count1 & STATUS1_MOTOR_ANGLE_ERROR)) { /* Check gimbal is error state*/

      this->gimbal_status.state = GIMBAL_STATE_ERROR;
    }

    /* Check gimbal's sensor status */
    if(errors_count2 & 0x01) { this->gimbal_status.sensor |= SENSOR_IMU_ERROR; }
    if(errors_count2 & 0x02) { this->gimbal_status.sensor |= SENSOR_EN_TILT; }
    if(errors_count2 & 0x04) { this->gimbal_status.sensor |= SENSOR_EN_ROLL; }

    if(errors_count2 & 0x08) {
      this->gimbal_status.sensor |= SENSOR_EN_PAN;
    } else {
      this->gimbal_status.sensor = SENSOR_OK; // TODO (MEF): Seems like an error that this is the else condition.
    }
  }

  return gimbal_status;
}


mavlink_raw_imu_t Gimbal_Interface::get_gimbal_raw_imu(void) {
  if(last_message.time_stamps.raw_imu) { return last_message.raw_imu; }
}

mavlink_mount_orientation_t Gimbal_Interface::get_gimbal_mount_orientation(void) {
  if(last_message.time_stamps.mount_orientation) { return last_message.mount_orientation; }
}

mavlink_mount_status_t Gimbal_Interface::get_gimbal_mount_status(void) {
  if(last_message.time_stamps.mount_status) { return last_message.mount_status; }
}

Time_Stamps Gimbal_Interface::get_gimbal_time_stamps(void) { return last_message.time_stamps; }

// Returns the sequence number of the last packet received.
Sequence_Numbers Gimbal_Interface::get_gimbal_seq_num(void) { return last_message.current_seq_rx; }

/**
 * @brief  This function get gimbal the command ack of MAV_CMD_DO_MOUNT_CONFIGURE
 * @param: None
 * @ret: Result of command
 */
uint8_t Gimbal_Interface::get_command_ack_do_mount_configure(void) {
  if(last_message.time_stamps.command_ack) { // any ack received?
    return last_message.result_cmd_ack_msg_configure;
  }
}

/**
 * @brief  This function get gimbal the command ack of MAV_CMD_DO_MOUNT_CONTROL
 * @param: None
 * @ret: Result of command
 */
uint8_t Gimbal_Interface::get_command_ack_do_mount_control(void) {
  if(last_message.time_stamps.command_ack) { // any ack received?
    return last_message.result_cmd_ack_msg_control;
  }
}

void Gimbal_Interface::write_heartbeat(void) {

  mavlink_heartbeat_t heartbeat;
  heartbeat.type = MAV_TYPE_ONBOARD_CONTROLLER;
  heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
  heartbeat.base_mode = 0;
  heartbeat.custom_mode = 0;
  heartbeat.system_status = MAV_STATE_ACTIVE;

  // Encode and write the message.
  mavlink_message_t message;
  mavlink_msg_heartbeat_encode(SYSID_ONBOARD, MAV_COMP_ID_SYSTEM_CONTROL, &message, &heartbeat);
  int len = write_message(message);
  if(len <= 0) {
    fprintf(stderr, "WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
  }
  else {
    // printf("%lu Write Heartbeat\n", write_count);
  }

  return;
}

void Gimbal_Interface::start() {

  // Check serial port...
  if(serial_port->status != 1) { // SERIAL_PORT_OPEN
    fprintf(stderr, "ERROR: serial port not open\n");
    throw 1;
  }

  // Start read thread...
  printf("START READ THREAD\n");
  int result = pthread_create(&read_tid, NULL, &start_gimbal_interface_read_thread, this);
  if(result) throw result;
  printf("\n"); // now we're reading messages

  // Check for messages...
  do {
    if(exit_signalled) {
      printf("CHECK FOR MESSAGES sysid: %d compid: %d\n", last_message.sysid, last_message.compid);
      return;
    }
    usleep(500000); // Check at 2Hz
  } while(not get_connection());
  printf("Found\n\n"); // We know the gimbal is sending messages

  // --------------------------------------------------------------------------
  //   GET SYSTEM and COMPONENT IDs
  // --------------------------------------------------------------------------

  // This comes from the heartbeat, which in theory should only come from
  // the Gimbal we're directly connected to it.  If there is more than one
  // vehicle then we can't expect to discover id's like this.
  // In which case set the id's manually.

  // System ID
  if(not system_id) {
    system_id = last_message.sysid;
    printf("GOT GIMBAL SYSTEM ID: %i\n", system_id);
  }

  // Component ID
  if(not gimbal_id) {
    gimbal_id = last_message.compid;
    printf("GOT GIMBAL COMPONENT ID: %i\n\n", gimbal_id);
  }

  // --------------------------------------------------------------------------
  //   WRITE THREAD
  // --------------------------------------------------------------------------
  printf("START WRITE THREAD \n");

  result = pthread_create(&write_tid, NULL, &start_gimbal_interface_write_thread, this);
  if(result) throw result;

  // wait for it to be started
  while(not writing_status) { usleep(100000); } // 10Hz 

  // now we're streaming setpoint commands
  printf("\n");
  return;
}


void Gimbal_Interface::stop() {

  printf("CLOSE THREADS\n");
  exit_signalled = true; // signal exit

  // wait for exit
  pthread_join(read_tid, NULL);
  pthread_join(write_tid, NULL);

  // now the read and write threads are closed
  printf("\n");

  // still need to close the serial_port separately
}

void Gimbal_Interface::start_read_thread() {
  if(reading_status != 0) {
    fprintf(stderr, "read thread already running\n");
    return;
  }
  else {
    read_thread();
    return;
  }
}

void Gimbal_Interface::start_write_thread(void) {
  if(not writing_status == false) {
    fprintf(stderr, "write thread already running\n");
    return;
  }
  else {
    write_thread();
    return;
  }
}


// Quit Handler
void Gimbal_Interface::handle_quit(int sig) {

  // Send command disable 
  // disable_offboard_control();

  try {
    stop();
  }
  catch(int error) {
    fprintf(stderr, "Warning, could not stop gimbal interface\n");
  }
}

bool Gimbal_Interface::get_flag_exit(void) { return exit_signalled; }


bool Gimbal_Interface::get_connection(void) {

  // Check heartbeat from gimbal
  //uint64_t timeout = get_time_usec() - time_of_last_heartbeart_us;
  uint64_t time_since_last_heartbeat = get_time_usec() - last_message.time_stamps.heartbeat;
  if(!heartbeat_detected && (time_since_last_heartbeat > _time_lost_connection)) {
    printf(" Could not connect.\n");
    return false;
  }

  return true;
}

bool Gimbal_Interface::present() {

  //uint64_t timeout = get_time_usec() - time_of_last_heartbeart_us;
  uint64_t time_since_last_heartbeat = get_time_usec() - last_message.time_stamps.heartbeat;
  if(gimbal_state != GIMBAL_STATE_NOT_PRESENT && (time_since_last_heartbeat > _time_lost_connection)) { // check time out
    printf(" Not Present!\n");
    gimbal_state = GIMBAL_STATE_NOT_PRESENT;
    return false;
  }

  return (gimbal_state != GIMBAL_STATE_NOT_PRESENT) && (gimbal_state == GIMBAL_STATE_PRESENT_RUNNING);
}

void Gimbal_Interface::read_thread() {

  reading_status = true;

  while(!exit_signalled) {
    read_messages();
    usleep(10000); // Read batches at 10Hz
  }

  reading_status = false;
  return;
}

uint32_t time_send_param, time_send_heartbeat;

void Gimbal_Interface::write_thread(void) {

  while(!writing_status and !exit_signalled) { // Blocking wait for new data

    uint32_t tnow_ms = get_time_usec();
    writing_status = true; // signal startup

    if(tnow_ms - time_send_heartbeat > 1000000) {
      time_send_heartbeat = get_time_usec();
      write_heartbeat(); // write a message and signal writing
      //printf("HB: %d\n", (uint32_t)(tnow_ms - time_send_heartbeat));
    } else if(tnow_ms - time_send_param > 500000) {
      time_send_param = get_time_usec();
      param_process(); // process check param
    }

    writing_status = false; // signal end
  }

  return;
}


//  Pthread Starter Helper Functions

void* start_gimbal_interface_read_thread(void* args) {
  Gimbal_Interface* gimbal_interface = (Gimbal_Interface*) args;
  gimbal_interface->start_read_thread();
  return NULL;
}

void* start_gimbal_interface_write_thread(void* args) {
  Gimbal_Interface* gimbal_interface = (Gimbal_Interface*) args;
  gimbal_interface->start_write_thread();
  return NULL;
}

/************************ (C) COPYRIGHT Gremsy *****END OF FILE****************/
