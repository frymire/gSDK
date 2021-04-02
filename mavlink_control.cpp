/*******************************************************************************
 * @file    mavlink_control.cpp
 * @author  The GremsyCo
 * @version V2.3.0
 * @date    August-21-2018
 * @brief   This file contains a example for showing how to control gimbal in
 *          some cases
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
 
#include "mavlink_control.h"

typedef enum _sdk_process_state {
    STATE_IDLE,
    STATE_CHECK_FIRMWARE_VERSION,
    STATE_SETTING_GIMBAL,
    STATE_SETTING_MESSAGE_RATE,
    STATE_SET_GIMBAL_OFF,
    STATE_SET_GIMBAL_ON,
    STATE_SET_CTRL_GIMBAL_YAW_FOLLOW_MODE,
    STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CW,
    STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CCW,
    STATE_SET_CTRL_GIMBAL_SPEED_MODE,
    STATE_MOVE_SPEED_MODE,
    STATE_MOVE_TO_ZERO,
    STATE_SET_GIMBAL_REBOOT,
    STATE_DONE
} sdk_process_state_t; 

typedef struct {
    sdk_process_state_t state;
    uint64_t last_time_send;
    uint64_t timeout;
} sdk_process_t;

static sdk_process_t sdk;


int main(int argc, char** argv) {
  try {
    int result = gGimbal_sample(argc, argv);
    return result;
  }
  catch (int error) {
    fprintf(stderr, "mavlink_control threw exception %i \n", error);
    return error;
  }
}

int gGimbal_sample(int argc, char** argv) {

	char* uart_name = (char*) "/dev/ttyUSB0"; // Unix default
	parse_commandline(argc, argv, uart_name, baudrate);
	Serial_Port serial_port(uart_name, baudrate);
	Gimbal_Interface gimbal_interface(&serial_port);

	// Setup interrupt signal handler to respond to early exits signaled with Ctrl-C.
	serial_port_quit = &serial_port;
	gimbal_interface_quit = &gimbal_interface;
	signal(SIGINT, quit_handler);

	// Start the port and Gimbal_interface. This is where the port is opened, and read and write threads are started.
	serial_port.start();
	gimbal_interface.start();

	/// Process data until an exit has been signaled.
	while (!gimbal_interface.get_flag_exit()) {

		uint32_t time_ms = (uint32_t) (get_time_usec()/1000);

		if((time_ms % 500 == 0) && gimbal_interface.present()) {
      sdk.timeout = get_time_usec(); // reset time 
			gGimbal_control_sample(gimbal_interface); // sample control
			gGimbal_displays(gimbal_interface); // sample display value
		} else {
      if(get_time_usec() - sdk.timeout > 2000000) { sdk.state = STATE_IDLE; }
    }
	}

	gimbal_interface.stop();
	serial_port.stop();
	return 0;
}

void parse_commandline(int argc, char** argv, char*& uart_name, int& baudrate) {

  const char* commandline_usage = "usage: mavlink_control -d <devicename> -b <baudrate>";

  // Read input arguments
  for (int i = 1; i < argc; i++) { // argv[0] is "mavlink_control"

    // Help
    if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      printf("%s\n", commandline_usage);
      throw EXIT_FAILURE;
    }

    // UART device ID
    if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
      if (argc > i + 1) {
        uart_name = argv[i + 1];
      }
      else {
        printf("%s\n", commandline_usage);
        throw EXIT_FAILURE;
      }
    }

    // Baud rate
    if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
      if (argc > i + 1) {
        baudrate = atoi(argv[i + 1]);
      }
      else {
        printf("%s\n", commandline_usage);
        throw EXIT_FAILURE;
      }
    }
  }

  return;
}

void gGimbal_displays(Gimbal_Interface &api) {

  gimbal_status_t gimbal_status = api.get_gimbal_status();

  printf("Gimbal status: ");
  switch (gimbal_status.state) {
  case GIMBAL_STATE_OFF:
    printf("OFF\n");
    break;
  case GIMBAL_STATE_ON:
    printf("OPERATING\n");
    break;
  case GIMBAL_STATE_INIT:
    printf("BUSY\n");
    break;
  case GIMBAL_STATE_ERROR:
    printf("ERROR\n");
    break;
  default:
    printf("Unrecognized gimbal status.\n");
  }

  mavlink_raw_imu_t imu = api.get_gimbal_raw_imu();
  imu.time_usec = api.get_gimbal_time_stamps().raw_imu;

	printf("Raw IMU: time: %lu, acc-xyz: [%d, %d, %d], gyro-xyz: [%d, %d, %d]; ", // mag-xyz: [%d, %d, %d]
    (unsigned long) imu.time_usec / 100000, 
    imu.xacc, 
    imu.yacc, 
    imu.zacc,
    //imu.xmag,
    //imu.ymag,
    //imu.zmag,
    imu.xgyro,
    imu.ygyro,
    imu.zgyro);

	mavlink_mount_orientation_t mnt_orien = api.get_gimbal_mount_orientation();
  mnt_orien.time_boot_ms = api.get_gimbal_time_stamps().mount_orientation;

	printf("Mount YPR (deg): [%f, %f, %f]; ", // time: %lu, 
    //(unsigned long) mnt_orien.time_boot_ms,
    mnt_orien.yaw,
    mnt_orien.pitch,
    mnt_orien.roll);

  mavlink_mount_status_t mnt_status = api.get_gimbal_mount_status();
  uint64_t mnt_status_time_stamp = api.get_gimbal_time_stamps().mount_status;

	//printf("Got message Mount status.");

  if(api.get_gimbal_config_mavlink_msg().enc_type_send) {
    printf(
      "\tEncoder Count: time: %lu, p:%d, r:%d, y:%d (Resolution 2^16)\n", 
      (unsigned long) mnt_status_time_stamp,
      mnt_status.pointing_a, 
      mnt_status.pointing_b,
      mnt_status.pointing_c);
  } else {
    printf(
      "\tEncoder Angle YPR (deg): [%d, %d, %d]\n", // time: %lu, 
      //(unsigned long) mnt_status_time_stamp,
      mnt_status.pointing_c,
      mnt_status.pointing_a,
      mnt_status.pointing_b);
  }

  gimbal_config_axis_t setting = api.get_gimbal_config_tilt_axis();
  //printf(
  //  "\tSETTING TILT: dir %d, speed_follow: %d, speed_control: %d\n", 
  //  setting.dir,
  //  setting.speed_follow,
  //  setting.speed_control);

  gimbal_motor_control_t tilt;
  gimbal_motor_control_t roll;
  gimbal_motor_control_t pan;
  uint8_t output_filter, gyro_filter, gain;
  api.get_gimbal_motor_control(tilt, roll, pan, gyro_filter, output_filter, gain);
  //printf("\tMOTOR_CONTROL: GYRO - %d, OUTPUT FILTER - %d, GAIN - %d\n", gyro_filter, output_filter, gain);
  //printf("\tTILT stiffness %d, hold strength: %d\n", tilt.stiffness, tilt.holdstrength);
  //printf("\tROLL stiffness %d, hold strength: %d\n", roll.stiffness, roll.holdstrength);
  //printf("\tPAN  stiffness %d, hold strength: %d\n\n", pan.stiffness, pan.holdstrength);
}

// Demonstrate how to set gimbal mode and control gimbal angle and speed.
void gGimbal_control_sample(Gimbal_Interface &onboard) {

  switch(sdk.state) {

    case STATE_IDLE:

      sdk.state = STATE_CHECK_FIRMWARE_VERSION;
      sdk.last_time_send = get_time_usec();
      break;

    case STATE_CHECK_FIRMWARE_VERSION:

      fw_version_t fw = onboard.get_gimbal_version();
      printf("Firmware Version: %d.%d.%d.%s\n", fw.x, fw.y, fw.z, fw.type);

      // This firmware only apply for the firmware version from v7.x.x or above
      if(fw.x >= 7 && fw.y >= 5) {
        sdk.state = STATE_SETTING_GIMBAL;
      } else {
        //printf("DO NOT SUPPORT FUNCTIONS. Please check the firmware version\n");
        //printf("1. MOTOR CONTROL\n");
        //printf("2. AXIS CONFIGURATION\n");
        //printf("3. MAVLINK MSG RATE CONFIGURATION\n");
        sdk.state = STATE_SETTING_GIMBAL; // MEF: Try it anyway!
      }

      usleep(100000);
      break;

    case STATE_SETTING_GIMBAL:

      printf("Setting gimbal...\n");

      // Setting axis for control. see the struct gimbal_config_axis_t
      gimbal_config_axis_t config = {0};

      config = {DIR_CCW, 50, 50, 65, 50, 0};
      onboard.set_gimbal_config_tilt_axis(config);

      config = {DIR_CW, 50, 60, 0, 0, 0};
      onboard.set_gimbal_config_roll_axis(config);

      config = {DIR_CW, 50, 70, 87, 50, 0};
      onboard.set_gimbal_config_pan_axis(config);

      gimbal_motor_control_t tilt = {80, 40}; // stiffness, hold strength
      gimbal_motor_control_t roll = {90, 40};
      gimbal_motor_control_t pan = {100, 40};
      onboard.set_gimbal_motor_control(tilt, roll, pan, 2, 3, 120);

      usleep(100000);
      sdk.state = STATE_SETTING_MESSAGE_RATE;
      break;

    case STATE_SETTING_MESSAGE_RATE:

      // configuration message. Note emit_heartbeat need to emit when using this gSDK. If not, the gSDK will waiting forever.
      printf("Setting message rates...\n");
      uint8_t emit_heatbeat = 1; // must be 1, otherwise gSDK will wait forever
      uint8_t status_rate = 1; // 10
      uint8_t enc_value_rate = 1; // 10
      uint8_t enc_type_send = 0;  // Set type of encoder is angle
      uint8_t orien_rate = 10; // 50
      uint8_t imu_rate = 10;
      onboard.set_gimbal_config_mavlink_msg(emit_heatbeat, status_rate, enc_value_rate, enc_type_send, orien_rate, imu_rate);

      usleep(100000);
      sdk.state = STATE_SET_GIMBAL_OFF;
      break;

    case STATE_SET_GIMBAL_OFF:

      printf("Turning off gimbal...\n");

      if(onboard.get_gimbal_status().state == GIMBAL_STATE_ON) {
        onboard.set_gimbal_motor_mode(TURN_OFF);
        printf("TURN_OFF! %d \n", onboard.get_gimbal_status().mode);
        sdk.last_time_send = get_time_usec();
      } else if(onboard.get_gimbal_status().state == GIMBAL_STATE_OFF) {
        if((get_time_usec() - sdk.last_time_send) > 1000000) {
          sdk.last_time_send = get_time_usec();                    
          sdk.state = STATE_SET_GIMBAL_ON;
        }
      }

      break;

    case STATE_SET_GIMBAL_ON:

      printf("Turning on gimbal...\n");

      if(onboard.get_gimbal_status().mode == GIMBAL_STATE_OFF) {
        onboard.set_gimbal_motor_mode(TURN_ON);                
        printf("TURN_ON!\n");                
        sdk.last_time_send = get_time_usec();
      } else if(onboard.get_gimbal_status().mode) {
        if((get_time_usec() - sdk.last_time_send) > 1000000) {
          sdk.last_time_send = get_time_usec();                    
          sdk.state = STATE_SET_CTRL_GIMBAL_YAW_FOLLOW_MODE;
        }
      }

      break;

    case STATE_SET_CTRL_GIMBAL_YAW_FOLLOW_MODE:
          
      // Set gimbal axes mode. NOTE: ROLL only allows controlling in ABSOLUTE_FRAME and ANGULAR_RATE.
      printf("Set gimbal's yaw follow mode. %d\n", onboard.get_command_ack_do_mount_configure());
            
      control_gimbal_axis_mode_t pitch, roll, yaw;
      pitch.input_mode = CTRL_ANGLE_ABSOLUTE_FRAME;
      roll.input_mode = CTRL_ANGLE_ABSOLUTE_FRAME;
      yaw.input_mode = CTRL_ANGLE_BODY_FRAME;
      onboard.set_gimbal_axes_mode(pitch, roll, yaw);
            
      // Check gimbal feedback COMMAND_ACK when sending MAV_CMD_DO_MOUNT_CONFIGURE. 
      if(onboard.get_command_ack_do_mount_configure() == MAV_RESULT_ACCEPTED) {

        // Wait 5 seconds to see the effect.
        if((get_time_usec() - sdk.last_time_send) > 5000000) {
          sdk.last_time_send = get_time_usec();
          sdk.state = STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CW;
        }
      }

      break;

    case STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CW:

      printf("Control gimbal's yaw cw follow mode. %d\n", onboard.get_command_ack_do_mount_control());
      float setpoint_pitch = 40.0;
      float setpoint_roll = 0;
      float setpoint_yaw = 170.0;
      onboard.set_gimbal_move(setpoint_pitch, setpoint_roll, setpoint_yaw);

      // Check gimbal feedback COMMAND_ACK when sending MAV_CMD_DO_MOUNT_CONTROL
      if(onboard.get_command_ack_do_mount_control() == MAV_RESULT_ACCEPTED) {
        if((get_time_usec() - sdk.last_time_send) > 5000000) { // wait 5 seconds
          sdk.last_time_send = get_time_usec(); // Reset time for the next step
          sdk.state = STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CCW; // Switch to move gimbal in CCW
        }
      }

      break;

    case STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CCW:

      printf("Control gimbal's yaw ccw follow mode. %d\n", onboard.get_command_ack_do_mount_control());
      float setpoint_pitch = -40.0;
      float setpoint_roll = 0;
      float setpoint_yaw = -170.0;
      onboard.set_gimbal_move(setpoint_pitch, setpoint_roll, setpoint_yaw);

      // Check gimbal feedback COMMAND_ACK after sending angle
      if(onboard.get_command_ack_do_mount_control() == MAV_RESULT_ACCEPTED) {
        if((get_time_usec() - sdk.last_time_send) > 5000000) { // wait 5 seconds
          sdk.last_time_send = get_time_usec();
          sdk.state = STATE_SET_CTRL_GIMBAL_SPEED_MODE;
        }
      }

      break;

    case STATE_SET_CTRL_GIMBAL_SPEED_MODE:

      printf("Set move gimbal in speed mode.\n");            
      control_gimbal_axis_mode_t pitch, roll, yaw;            
      pitch.input_mode = CTRL_ANGULAR_RATE;
      roll.input_mode = CTRL_ANGULAR_RATE;
      yaw.input_mode = CTRL_ANGULAR_RATE;            
      onboard.set_gimbal_axes_mode(pitch, roll, yaw);
            
      // Check gimbal feedback COMMAND_ACK after sending angle
      if(onboard.get_command_ack_do_mount_configure() == MAV_RESULT_ACCEPTED) {
        sdk.last_time_send = get_time_usec();
        sdk.state = STATE_MOVE_SPEED_MODE;
      }

      break;

    case STATE_MOVE_SPEED_MODE:

      printf("Speed control gimbal in speed mode:\n");

      // Moving gimbal in speed mode with speed = 10 degree/second
      float setpoint_pitch = 0.1; // MEF: Really 10 degrees/second here?
      float setpoint_roll = 0;
      float setpoint_yaw = 0.1;

      mavlink_mount_orientation_t mount = onboard.get_gimbal_mount_orientation();
      printf("YPR: [%2.3f, %2.3f, %2.3f]\n", mount.yaw, mount.pitch, mount.roll);

      /// Move gimbal in speed mode
      onboard.set_gimbal_move(setpoint_pitch, setpoint_roll, setpoint_yaw);

      //Moving gimbal in speed mode about 5 seconds
      if((get_time_usec() - sdk.last_time_send) > 5000000) {
        sdk.last_time_send = get_time_usec();
        sdk.state = STATE_MOVE_TO_ZERO;
      }

      break;

    case STATE_MOVE_TO_ZERO:

      printf("Set move angle for axes mode.\n");
      control_gimbal_axis_mode_t pitch, roll, yaw;
      pitch.input_mode = CTRL_ANGLE_ABSOLUTE_FRAME;
      roll.input_mode = CTRL_ANGLE_ABSOLUTE_FRAME;
      yaw.input_mode = CTRL_ANGLE_BODY_FRAME;            
      onboard.set_gimbal_axes_mode(pitch, roll, yaw);
            
      // Check gimbal feedback COMMAND_ACK after sending angle
      if(onboard.get_command_ack_do_mount_configure() != MAV_RESULT_ACCEPTED) { 
        printf("Did not receive ack.\n");
        break; 
      }

      printf("Move gimbal to home position.\n");
      float setpoint_pitch = 0;
      float setpoint_roll = 0;
      float setpoint_yaw = 0;
      onboard.set_gimbal_move(setpoint_pitch, setpoint_roll, setpoint_yaw);

      if(onboard.get_command_ack_do_mount_control() == MAV_RESULT_ACCEPTED) { // check for COMMAND_ACK
        if((get_time_usec() - sdk.last_time_send) > 5000000) { // wait 5 seconds
          sdk.last_time_send = get_time_usec();
          sdk.state = STATE_SET_GIMBAL_REBOOT;
        }
      }
      break;

    case STATE_SET_GIMBAL_REBOOT:

      printf("Rebooting gimbal.\n");
      onboard.set_gimbal_reboot();
      if((get_time_usec() - sdk.last_time_send) > 1000000) {
        sdk.last_time_send = get_time_usec();
        sdk.state = STATE_IDLE;
      }
      break;

  } // switch
}


// Quit signal handler, called when you press Ctrl-C. Without this, the vehicle would go into failsafe.
void quit_handler(int sig) {

	printf("\nTerminating at user request...\n\n");
	
	try {
		gimbal_interface_quit->handle_quit(sig); // autopilot interface
	} catch (int error){}
	
	try {
		serial_port_quit->handle_quit(sig); // serial port
	} catch (int error){}

	exit(0);
}

/************************ (C) COPYRIGHT Gremsy *****END OF FILE****************/
