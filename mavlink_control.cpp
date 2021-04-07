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

//#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include <string.h>
using std::string;

#include <ardupilotmega/mavlink.h>
#include "gimbal_interface.h"
#include "serial_port.h"

void ParseCommandLine(int argc, char** argv, char*& uart_name, int& baudrate);
void DisplayGimbalStatus(Gimbal_Interface& gimbal_interface);

//void gGimbal_control_sample(Gimbal_Interface &gimbal);
void CheckFirmwareVersion(Gimbal_Interface &gimbal);
void SetMessageRates(Gimbal_Interface &gimbal);
void PrintMessageRates(config_mavlink_message_t message_rates);
void TurnOff(Gimbal_Interface &gimbal);
void TurnOn(Gimbal_Interface &gimbal);
void ConfigureGimbalAxes(Gimbal_Interface &gimbal);
void CheckMountConfigureAck(Gimbal_Interface &gimbal);
void CheckMountControlAck(Gimbal_Interface &gimbal);
void PrintGimbalControlValues(Gimbal_Interface &gimbal);
void Point(Gimbal_Interface &gimbal, float yaw, float pitch, float roll);
void PointHome(Gimbal_Interface &gimbal);

Gimbal_Interface* gimbal_interface_quit;
Serial_Port* serial_port_quit;
void HandleQuitSignal(int sig);

int main(int argc, char** argv) {

  try {

    char* uart_name = (char*)"/dev/ttyUSB0"; // Unix default
    int baudrate = 115200;
    ParseCommandLine(argc, argv, uart_name, baudrate);
    Serial_Port serial_port(uart_name, baudrate);
    Gimbal_Interface gimbal(&serial_port);

    // Setup interrupt signal handler to respond to early exits signaled with Ctrl-C.
    serial_port_quit = &serial_port;
    gimbal_interface_quit = &gimbal;
    signal(SIGINT, HandleQuitSignal);

    // Start the port and Gimbal_interface. This is where the port is opened, and read and write threads are started.
    serial_port.start();
    gimbal.start();

    // Spin until the gimbal is ready
    while(!gimbal.present()) {
      printf("Gimbal not present.\n");
      usleep(1000000);
    } 

    CheckFirmwareVersion(gimbal);
    SetMessageRates(gimbal);
    TurnOff(gimbal);
    TurnOn(gimbal);
    ConfigureGimbalAxes(gimbal);
    Point(gimbal, 80.0f, 25.0f, -45.0f);
    Point(gimbal, -45.0f, -10.0f, 30.0f);
    PointHome(gimbal);

    /// Process data until an exit has been signaled.
    while (!gimbal.get_flag_exit()) {
      usleep(1000000);
      DisplayGimbalStatus(gimbal);
    }

    gimbal.stop();
    serial_port.stop();
  }
  catch (int error) {
    fprintf(stderr, "main threw exception %i \n", error);
    return error;
  }
}

void ParseCommandLine(int argc, char** argv, char* &uart_name, int &baudrate) {

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


// Quit signal handler, called when you press Ctrl-C. Without this, the vehicle would go into failsafe.
void HandleQuitSignal(int sig) {

  printf("\nTerminating at user request...\n\n");

  try {
    gimbal_interface_quit->handle_quit(sig); // autopilot interface
  }
  catch (int error) {}

  try {
    serial_port_quit->handle_quit(sig); // serial port
  }
  catch (int error) {}

  exit(0);
}


void CheckFirmwareVersion(Gimbal_Interface &gimbal) {
  printf("Checking firmware version...\n");
  fw_version_t fw = gimbal.get_gimbal_version();
  usleep(3*1000000);
  printf("Firmware Version %d.%d.%d.%s\n", fw.x, fw.y, fw.z, fw.type);
}

void SetMessageRates(Gimbal_Interface &gimbal) {
  
  config_mavlink_message_t message_rates = gimbal.get_gimbal_config_mavlink_msg(); 
  PrintMessageRates(message_rates);
  
  printf("Setting message rates...\n");
  uint8_t emit_heatbeat = 1; // must be 1, otherwise gSDK will wait forever
  uint8_t status_rate = 1; // 10
  uint8_t enc_value_rate = 1; // 10
  uint8_t enc_type_send = 0;  // angle encoding (0)
  uint8_t orientation_rate = 5; // 50
  uint8_t imu_rate = 5;
  gimbal.set_gimbal_config_mavlink_msg(emit_heatbeat, status_rate, enc_value_rate, enc_type_send, orientation_rate, imu_rate);
  usleep(3*1000000);
  
  message_rates = gimbal.get_gimbal_config_mavlink_msg();
  PrintMessageRates(message_rates);
}

void PrintMessageRates(config_mavlink_message_t message_rates) {
  printf("Message Rates...\n");
  printf("  emit_heartbeat = %d\n", message_rates.emit_heatbeat);
  printf("  enc_type_send = %d\n", message_rates.enc_type_send);
  printf("  enc_value_rate = %d\n", message_rates.enc_value_rate);
  printf("  imu_rate = %d\n", message_rates.imu_rate);
  printf("  orientation_rate = %d\n", message_rates.orientation_rate);
  printf("  status_rate = %d\n", message_rates.status_rate);
}

void TurnOff(Gimbal_Interface &gimbal) {
  printf("Turning off gimbal...\n");
  gimbal.set_gimbal_motor_mode(TURN_OFF);
  usleep(5*1000000);
}

void TurnOn(Gimbal_Interface &gimbal) {
  printf("Turning on gimbal...\n");
  gimbal.set_gimbal_motor_mode(TURN_ON);
  usleep(5*1000000);
}

void ConfigureGimbalAxes(Gimbal_Interface &gimbal) {

  printf("Old gimbal control values...\n");
  PrintGimbalControlValues(gimbal);

  printf("Configuring gimbal axes...\n");
  
  gimbal_config_axis_t config = {0};
  
  config = {DIR_CCW, 50, 50, 65, 50, 0};
  gimbal.set_gimbal_config_tilt_axis(config);
  
  config = {DIR_CW, 50, 60, 0, 0, 0};
  gimbal.set_gimbal_config_roll_axis(config);
  
  config = {DIR_CW, 50, 70, 87, 50, 0};
  gimbal.set_gimbal_config_pan_axis(config);
  
  gimbal_motor_control_t tilt = {80, 40}; // stiffness, hold strength
  gimbal_motor_control_t roll = {90, 40};
  gimbal_motor_control_t pan = {100, 40};
  gimbal.set_gimbal_motor_control(tilt, roll, pan, 2, 3, 120);
  
  usleep(1000000);

  printf("New gimbal control values...\n");
  PrintGimbalControlValues(gimbal);
}

void PrintGimbalControlValues(Gimbal_Interface &gimbal) {
  gimbal_motor_control_t tilt, roll, pan;
  uint8_t gyro_filter, output_filter, gain;
  gimbal.get_gimbal_motor_control(tilt, roll, pan, gyro_filter, output_filter, gain);
  usleep(1000000);
  printf("  Tilt: %d %d", tilt.holdstrength, tilt.stiffness);
  printf("  Roll: %d %d", roll.holdstrength, roll.stiffness);
  printf("  Pan: %d %d", pan.holdstrength, pan.stiffness);
  printf("  Gyro Filter: %d, Output Filter: %d, Gain: %d\n", gyro_filter, output_filter, gain);
}

void SetLockMode(Gimbal_Interface &gimbal) {
  printf("Setting lock mode... (ack result = %d)\n", gimbal.get_command_ack_do_mount_configure());
  control_gimbal_axis_mode_t pitch, roll, yaw;
  pitch.input_mode = CTRL_ANGLE_ABSOLUTE_FRAME;
  roll.input_mode = CTRL_ANGLE_ABSOLUTE_FRAME;
  yaw.input_mode = CTRL_ANGLE_ABSOLUTE_FRAME;
  gimbal.set_gimbal_axes_mode(pitch, roll, yaw);
  CheckMountConfigureAck(gimbal);
}


void SetFollowMode(Gimbal_Interface &gimbal) {
  printf("Set follow mode... (ack result = %d)\n", gimbal.get_command_ack_do_mount_configure());
  control_gimbal_axis_mode_t pitch, roll, yaw;
  pitch.input_mode = CTRL_ANGLE_ABSOLUTE_FRAME;
  roll.input_mode = CTRL_ANGLE_ABSOLUTE_FRAME; // can only control roll in ABSOLUTE_FRAME and ANGULAR_RATE
  yaw.input_mode = CTRL_ANGLE_BODY_FRAME;
  gimbal.set_gimbal_axes_mode(pitch, roll, yaw);
  CheckMountConfigureAck(gimbal);
}

void CheckMountConfigureAck(Gimbal_Interface &gimbal) {
  uint8_t ack_result = gimbal.get_command_ack_do_mount_configure();
  if(ack_result == MAV_RESULT_ACCEPTED) {
    printf("Mount configure command ACK received. %d\n", ack_result);
  } else {
    printf("Mount configure command ACK not received. %d\n", ack_result);
  }
}

void CheckMountControlAck(Gimbal_Interface &gimbal) {
  uint8_t ack_result = gimbal.get_command_ack_do_mount_control();
  if(ack_result == MAV_RESULT_ACCEPTED) {
    printf("Mount control command ACK received. %d\n", ack_result);
  } else {
    printf("Mount control command ACK not received. %d\n", ack_result);
  }
}

void Point(Gimbal_Interface &gimbal, float yaw, float pitch, float roll) {
  printf("Pointing...\n");
  printf("Before: gimbal.get_command_ack_do_mount_control() = %d\n", gimbal.get_command_ack_do_mount_control());
  gimbal.set_gimbal_move(pitch, roll, yaw);
  printf("Just After: gimbal.get_command_ack_do_mount_control() = %d\n", gimbal.get_command_ack_do_mount_control());
  usleep(10 * 1000000);
  printf("10 Seconds After: gimbal.get_command_ack_do_mount_control() = %d\n", gimbal.get_command_ack_do_mount_control());
}

void PointHome(Gimbal_Interface &gimbal) {

  printf("Move gimbal to home position.\n");
  SetFollowMode(gimbal);  
  gimbal.set_gimbal_move(0.0f, 0.0f, 0.0f);
  
  if(gimbal.get_command_ack_do_mount_configure() == MAV_RESULT_ACCEPTED) {
    printf("Mount configure command ACK received.\n");
    usleep(5*1000000);
  } else {
    printf("Mount configure command ACK not received.\n");
  }
}


//  case STATE_SET_CTRL_GIMBAL_SPEED_MODE:
//  {
//    printf("Set move gimbal in speed mode.\n");
//    control_gimbal_axis_mode_t pitch, roll, yaw;
//    pitch.input_mode = CTRL_ANGULAR_RATE;
//    roll.input_mode = CTRL_ANGULAR_RATE;
//    yaw.input_mode = CTRL_ANGULAR_RATE;
//    onboard.set_gimbal_axes_mode(pitch, roll, yaw);
//
//    // Check gimbal feedback COMMAND_ACK after sending angle
//    if (onboard.get_command_ack_do_mount_configure() == MAV_RESULT_ACCEPTED) {
//      sdk.last_time_send = get_time_usec();
//      sdk.state = STATE_MOVE_SPEED_MODE;
//    }
//  }
//  break;
//
//  case STATE_MOVE_SPEED_MODE:
//  {
//    printf("Speed control gimbal in speed mode:\n");
//
//    // Moving gimbal in speed mode with speed = 10 degree/second
//    float setpoint_pitch = 0.1; // MEF: Really 10 degrees/second here?
//    float setpoint_roll = 0;
//    float setpoint_yaw = 0.1;
//
//    mavlink_mount_orientation_t mount = onboard.get_gimbal_mount_orientation();
//    printf("YPR: [%2.3f, %2.3f, %2.3f]\n", mount.yaw, mount.pitch, mount.roll);
//
//    /// Move gimbal in speed mode
//    onboard.set_gimbal_move(setpoint_pitch, setpoint_roll, setpoint_yaw);
//
//    //Moving gimbal in speed mode about 5 seconds
//    if ((get_time_usec() - sdk.last_time_send) > 5000000) {
//      sdk.last_time_send = get_time_usec();
//      sdk.state = STATE_MOVE_TO_ZERO;
//    }
//  }
//  break;
//
//  case STATE_SET_GIMBAL_REBOOT:
//  {
//    printf("Rebooting gimbal.\n");
//    onboard.set_gimbal_reboot();
//    if ((get_time_usec() - sdk.last_time_send) > 1000000) {
//      sdk.last_time_send = get_time_usec();
//      sdk.state = STATE_IDLE;
//    }
//  }
//  break;
//
//  } // switch
//}


void DisplayGimbalStatus(Gimbal_Interface& api) {

  static unsigned long first_time = 0;

  gimbal_status_t gimbal_status = api.get_gimbal_status();

  printf("Gimbal status: ");
  switch(gimbal_status.state) {
  case GIMBAL_STATE_OFF:
    printf("OFF, ");
    break;
  case GIMBAL_STATE_ON:
    printf("OPERATING, ");
    break;
  case GIMBAL_STATE_INIT:
    printf("BUSY, ");
    break;
  case GIMBAL_STATE_ERROR:
    printf("ERROR, ");
    break;
  default:
    printf("Unrecognized gimbal status.\n");
  }

  mavlink_raw_imu_t imu = api.get_gimbal_raw_imu();
  imu.time_usec = api.get_gimbal_time_stamps().raw_imu;

  if(first_time == 0) { first_time = imu.time_usec; }

  printf("raw IMU: time: %lu, acc-xyz: [%d, %d, %d], gyro-xyz: [%d, %d, %d]; ", // mag-xyz: [%d, %d, %d]
    (unsigned long) (imu.time_usec - first_time) / 1000000,
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

  printf("mount YPR (deg): [%f, %f, %f]; ", // time: %lu, 
    //(unsigned long) mnt_orien.time_boot_ms,
    mnt_orien.yaw,
    mnt_orien.pitch,
    mnt_orien.roll);

  mavlink_mount_status_t mnt_status = api.get_gimbal_mount_status();
  uint64_t mnt_status_time_stamp = api.get_gimbal_time_stamps().mount_status;

  //printf("Got message Mount status.");

  if(api.get_gimbal_config_mavlink_msg().enc_type_send) {
    printf(
      "encoder count: time: %lu, p:%d, r:%d, y:%d (Resolution 2^16)\n",
      (unsigned long)mnt_status_time_stamp,
      mnt_status.pointing_a,
      mnt_status.pointing_b,
      mnt_status.pointing_c);
  }
  else {
    printf(
      "encoder angle YPR (deg): [%d, %d, %d]\n", // time: %lu, 
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
