#ifndef SERIAL_DEVICE_H
#define SERIAL_DEVICE_H

#include "ros/ros.h"
#include "vortex_msgs/Pwm.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>     // String function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>

extern "C" {
#include "crc.h"
}

typedef enum msg_type
{
  MSG_TYPE_NOTYPE,
  MSG_TYPE_THRUSTER   = 0x41,
  MSG_TYPE_LIGHT      = 0x42,
  MSG_TYPE_HEARTBEAT  = 0x43,
  MSG_TYPE_ACK        = 0x44,
  MSG_TYPE_NOACK      = 0x45,
  MSG_TYPE_ARM        = 0x46,
  MSG_TYPE_DISARM     = 0x47,
}msg_type;

#define READ_BUFFERSIZE     512

#define MAGIC_START_BYTE    0x24
#define MAGIC_STOP_BYTE     0x40

#define MAX_PAYLOAD_SIZE    16
#define MAX_MSG_SIZE        21

#define MSG_HEARTBEAT_SIZE  3
#define MSG_ARMING_SIZE     3
#define MSG_LIGHT_SIZE      7

#define MAGIC_START_BYTE_INDEX  0
#define MSG_TYPE_INDEX          1
#define MSG_PAYLOAD_START_INDEX 2
#define MSG_PAYLOAD_STOP_INDEX  17
#define MSG_CRC_BYTE_INDEX      18
#define MAGIC_STOP_BYTE_INDEX   20

class McuInterface
{
private:
  int m_dev = 0;
  char m_read_buffer[READ_BUFFERSIZE];
  char m_thruster_cmd[MAX_MSG_SIZE] = {0};
  char m_heartbeat_cmd[MSG_HEARTBEAT_SIZE] = {5};
  char m_arming_cmd[MSG_ARMING_SIZE] = {10};
  char m_light_cmd[MSG_LIGHT_SIZE]  = {5};

public:
  void thruster_pwm_callback(const vortex_msgs::Pwm& msg);
  void heartbeat_callback(const std_msgs::String::ConstPtr& msg);
  void arming_callback(const std_msgs::String::ConstPtr& msg);
  void light_pwm_callback(const std_msgs::Int16::ConstPtr& msg);
  uint8_t read_leak_sensor();
  int serial_write(char* cmd, int cmd_size);
  uint16_t crc_checksum(char* input, uint8_t num);
  int serial_read();
  void clear_read_buffer();
  McuInterface(const char device[]);
  ~McuInterface();
};

#endif // SERIAL_DEVICE_H
