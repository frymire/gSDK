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

void PrintFirmwareVersion(Gimbal_Interface &gimbal);
void SetMessageRates(Gimbal_Interface &gimbal);
void PrintMessageRates(config_mavlink_message_t message_rates);
void WaitForConfigAck(Gimbal_Interface &gimbal, uint polling_interval_us);
void WaitForCommandAck(Gimbal_Interface &gimbal, uint polling_interval_us);
void TurnOff(Gimbal_Interface &gimbal);
void TurnOn(Gimbal_Interface &gimbal);
void ConfigureGimbalAxes(Gimbal_Interface &gimbal);
void PrintGimbalControlValues(Gimbal_Interface &gimbal);
void SetLockMode(Gimbal_Interface &gimbal);
void SetFollowMode(Gimbal_Interface &gimbal);
void SetGimbalSpeed(Gimbal_Interface &gimbal);
void Point(Gimbal_Interface &gimbal, float yaw, float pitch, float roll);
void PointHome(Gimbal_Interface &gimbal);
void WriteGimbalStatus(gimbal_status_t gimbal_status);
void WriteRawIMU(mavlink_raw_imu_t imu);
void WriteMountOrientation(mavlink_mount_orientation_t mnt_orien);
void WriteEncoderAngles(mavlink_mount_status_t mnt_status);
void WriteAxisSettings(gimbal_config_axis_t axis_settings);

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

    while(!gimbal.present()) { usleep(1000000); } // spin until the gimbal is ready
    PrintFirmwareVersion(gimbal);
    SetMessageRates(gimbal);
    TurnOff(gimbal);
    TurnOn(gimbal);
    ConfigureGimbalAxes(gimbal);
    SetLockMode(gimbal);
    Point(gimbal, 80.0f, 25.0f, -45.0f);
    SetGimbalSpeed(gimbal);
    Point(gimbal, -45.0f, -10.0f, 30.0f);
    PointHome(gimbal);

    //printf("Rebooting gimbal.\n");
    //gimbal.set_gimbal_reboot();

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
  } catch (int error) {}

  try {
    serial_port_quit->handle_quit(sig); // serial port
  } catch (int error) {}

  exit(0);
}


void PrintFirmwareVersion(Gimbal_Interface &gimbal) {
  if(!gimbal.present()) { printf("WARNING: Printing firmware version before gimbal initialization is complete.\n"); }
  fw_version_t fw = gimbal.get_gimbal_version();
  printf("Firmware Version %d.%d.%d.%s\n", fw.x, fw.y, fw.z, fw.type);
}

void SetMessageRates(Gimbal_Interface &gimbal) {
  
  PrintMessageRates(gimbal.get_gimbal_config_mavlink_msg());
  
  printf("Setting message rates...\n");
  uint8_t emit_heatbeat = 1; // must be 1, otherwise gSDK will wait forever
  uint8_t status_rate = 10; // 10
  uint8_t enc_value_rate = 10; // 10
  uint8_t enc_type_send = 0;  // angle encoding (0)
  uint8_t orientation_rate = 8; // 50
  uint8_t imu_rate = 5;
  gimbal.set_gimbal_config_mavlink_msg(emit_heatbeat, status_rate, enc_value_rate, enc_type_send, orientation_rate, imu_rate);
  //usleep(3*1000000);
  
  PrintMessageRates(gimbal.get_gimbal_config_mavlink_msg());
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
  //gimbal.reset_acks();
  gimbal.set_gimbal_motor_mode(TURN_OFF);
  //WaitForCommandAck(gimbal, 50000);
  usleep(5*1000000);
}

void TurnOn(Gimbal_Interface &gimbal) {
  printf("Turning on gimbal...\n");
  //gimbal.reset_acks();
  gimbal.set_gimbal_motor_mode(TURN_ON);
  //WaitForCommandAck(gimbal, 50000);
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

  gimbal.set_gimbal_mode(LOCK_MODE);

  control_gimbal_axis_mode_t pitch, roll, yaw;
  pitch.input_mode = CTRL_ANGLE_ABSOLUTE_FRAME;
  roll.input_mode = CTRL_ANGLE_ABSOLUTE_FRAME;
  yaw.input_mode = CTRL_ANGLE_ABSOLUTE_FRAME;
  //gimbal.reset_acks();
  gimbal.set_gimbal_axes_mode(pitch, roll, yaw);
  //WaitForConfigAck(gimbal, 50000);
}


void SetFollowMode(Gimbal_Interface &gimbal) {
  printf("Set follow mode... (ack result = %d)\n", gimbal.get_command_ack_do_mount_configure());

  gimbal.set_gimbal_mode(FOLLOW_MODE);

  control_gimbal_axis_mode_t pitch, roll, yaw;
  pitch.input_mode = CTRL_ANGLE_ABSOLUTE_FRAME;
  roll.input_mode = CTRL_ANGLE_ABSOLUTE_FRAME; // can only control roll in ABSOLUTE_FRAME and ANGULAR_RATE
  yaw.input_mode = CTRL_ANGLE_BODY_FRAME;
  //gimbal.reset_acks();
  gimbal.set_gimbal_axes_mode(pitch, roll, yaw);
  //WaitForConfigAck(gimbal, 50000);
}

void WaitForConfigAck(Gimbal_Interface &gimbal, uint polling_interval_us) {
  bool done = false;
  while(!done) {
    uint8_t ack_value = gimbal.get_command_ack_do_mount_configure();
    printf("gimbal.get_command_ack_do_mount_configure() = %d\n", ack_value);
    if(ack_value == 0) { done = true; }
    usleep(polling_interval_us);
  }
}

void WaitForCommandAck(Gimbal_Interface &gimbal, uint polling_interval_us) {
  bool done = false;
  while(!done) {
    uint8_t ack_value = gimbal.get_command_ack_do_mount_control();
    printf("gimbal.get_command_ack_do_mount_control() = %d\n", ack_value);
    if(ack_value == 0) { done = true; }
    usleep(polling_interval_us);
  }
}

void Point(Gimbal_Interface &gimbal, float yaw, float pitch, float roll) {
  printf("Pointing...\n");
  gimbal.reset_acks();
  gimbal.set_gimbal_move(pitch, roll, yaw);
  WaitForCommandAck(gimbal, 50000); // poll every 0.05 seconds
}

void PointHome(Gimbal_Interface &gimbal) {
  printf("Move gimbal to home position.\n");
  SetFollowMode(gimbal);
  mavlink_mount_orientation_t mnt_orien = gimbal.get_gimbal_mount_orientation();
  gimbal.reset_acks();
  gimbal.set_gimbal_move(-1*mnt_orien.pitch, mnt_orien.roll, mnt_orien.yaw);
  //gimbal.set_gimbal_move(-1*mnt_orien.pitch, mnt_orien.roll, mnt_orien.yaw_absolute);
  WaitForCommandAck(gimbal, 50000);
  SetLockMode(gimbal);
}

void SetGimbalSpeed(Gimbal_Interface &gimbal) {

  printf("Set move gimbal in speed mode.\n");
  control_gimbal_axis_mode_t pitch, roll, yaw;
  pitch.input_mode = CTRL_ANGULAR_RATE;
  roll.input_mode = CTRL_ANGULAR_RATE;
  yaw.input_mode = CTRL_ANGULAR_RATE;

  gimbal.reset_acks();
  uint8_t ack_value = gimbal.get_command_ack_do_mount_configure();
  printf("gimbal.get_command_ack_do_mount_configure() = %d\n", ack_value);
  //gimbal.set_gimbal_axes_mode(pitch, roll, yaw);
  usleep(1*1000000);
  ack_value = gimbal.get_command_ack_do_mount_configure();
  printf("gimbal.get_command_ack_do_mount_configure() = %d\n", ack_value);

  //WaitForConfigAck(gimbal, 50000);

  printf("\"Move\" the gimbal in CTRL_ANGULAR_RATE mode to set the rate...\n");
  gimbal.set_gimbal_move(0.0f, 0.0f, 180.0f); // (pitch, roll, yaw) previously (0.1f, 0.0f, 0.1f)
  usleep(5*1000000);

  SetLockMode(gimbal); // go back to an actual pointing mode
}

//  case STATE_SET_GIMBAL_REBOOT:
//  {
//    printf("Rebooting gimbal.\n");
//    onboard.set_gimbal_reboot();
//    if ((get_time_usec() - sdk.last_time_send) > 1000000) {
//      sdk.last_time_send = get_time_usec();
//      sdk.state = STATE_IDLE;
//    }
//  }


void DisplayGimbalStatus(Gimbal_Interface &gimbal) {

  gimbal_status_t gimbal_status = gimbal.get_gimbal_status();
  WriteGimbalStatus(gimbal_status);

  mavlink_raw_imu_t imu = gimbal.get_gimbal_raw_imu();
  imu.time_usec = gimbal.get_gimbal_time_stamps().raw_imu;

  static unsigned long first_time = 0;
  if(first_time == 0) { first_time = imu.time_usec; }
  printf("; time: %lu", (unsigned long) (imu.time_usec - first_time) / 1000000);
  WriteRawIMU(imu);

  mavlink_mount_orientation_t mnt_orien = gimbal.get_gimbal_mount_orientation();
  mnt_orien.time_boot_ms = gimbal.get_gimbal_time_stamps().mount_orientation;
  WriteMountOrientation(mnt_orien);

  mavlink_mount_status_t mnt_status = gimbal.get_gimbal_mount_status();
  //uint64_t mnt_status_time_stamp = gimbal.get_gimbal_time_stamps().mount_status;
  WriteEncoderAngles(mnt_status);

  //// TODO: Just returns a struct full of zeros.
  //gimbal_config_axis_t yaw_axis_settings = gimbal.get_gimbal_config_pan_axis();
  //printf("\tYaw axis settings :");
  //WriteAxisSettings(yaw_axis_settings);

  gimbal_motor_control_t tilt;
  gimbal_motor_control_t roll;
  gimbal_motor_control_t pan;
  uint8_t output_filter, gyro_filter, gain;
  gimbal.get_gimbal_motor_control(tilt, roll, pan, gyro_filter, output_filter, gain);
  //printf("\tMOTOR_CONTROL: GYRO - %d, OUTPUT FILTER - %d, GAIN - %d\n", gyro_filter, output_filter, gain);
  //printf("\tTILT stiffness %d, hold strength: %d\n", tilt.stiffness, tilt.holdstrength);
  //printf("\tROLL stiffness %d, hold strength: %d\n", roll.stiffness, roll.holdstrength);
  //printf("\tPAN  stiffness %d, hold strength: %d\n\n", pan.stiffness, pan.holdstrength);
}

void WriteGimbalStatus(gimbal_status_t gimbal_status) {
  printf("Gimbal status: ");
  switch(gimbal_status.state) {
    case GIMBAL_STATE_OFF:
      printf("OFF");
      break;
    case GIMBAL_STATE_ON:
      printf("ON");
      break;
    case GIMBAL_STATE_INIT:
      printf("INIT");
      break;
    case GIMBAL_STATE_ERROR:
      printf("ERROR");
      break;
    default:
      printf("Unrecognized");
  }
}

void WriteRawIMU(mavlink_raw_imu_t imu) {
  printf("; accelerometer XYZ: [%d, %d, %d], gyro XYZ: [%d, %d, %d]", // mag-xyz: [%d, %d, %d]
    imu.xacc,
    imu.yacc,
    imu.zacc,
    //imu.xmag,
    //imu.ymag,
    //imu.zmag,
    imu.xgyro,
    imu.ygyro,
    imu.zgyro);
}

void WriteMountOrientation(mavlink_mount_orientation_t mnt_orien) {
  printf("; mount YPR (deg): [%f, %f, %f]", // time: %lu, 
    //(unsigned long) mnt_orien.time_boot_ms,
    mnt_orien.yaw,
    mnt_orien.pitch,
    mnt_orien.roll);
}

void WriteEncoderAngles(mavlink_mount_status_t mnt_status) {
  //if(gimbal.get_gimbal_config_mavlink_msg().enc_type_send) {
  //  printf(
  //    "encoder count: time: %lu, y:%d, p:%d, r:%d (Resolution 2^16)\n",
  //    (unsigned long)mnt_status_time_stamp,
  //    mnt_status.pointing_c,
  //    mnt_status.pointing_a,
  //    mnt_status.pointing_b
  //  );
  //} else {
    printf(
      "; encoder angle YPR (deg): [%d, %d, %d]\n", // time: %lu, 
      //(unsigned long) mnt_status_time_stamp,
      mnt_status.pointing_c,
      mnt_status.pointing_a,
      mnt_status.pointing_b
    );
  //}
}

void WriteAxisSettings(gimbal_config_axis_t axis_settings) {
  printf(
    "direction %d, follow speed: %d, control speed: %d\n",
    axis_settings.dir,
    axis_settings.speed_follow,
    axis_settings.speed_control);
}
