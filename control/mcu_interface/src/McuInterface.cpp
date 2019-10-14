#include "McuInterface.h"

McuInterface::McuInterface(const char device[])
{
  // Open File Descriptor
  m_dev = open(device, O_RDWR| O_NONBLOCK);

  while (m_dev < 0)
  {
    ROS_ERROR("Error %d opening %s: %s. Could not connect to MCU, retrying",
              errno, device, strerror(errno));

    usleep(2000*1000);

    m_dev = open(device, O_RDWR| O_NONBLOCK);
  }

  // Configure port
  struct termios tty;
  memset (&tty, 0, sizeof(tty));

  if (tcgetattr (m_dev, &tty) != 0 )
  {
    ROS_ERROR("Error %d from tcgetattr:  %s", errno, strerror(errno));
  }

  // Set Baud Rate
  cfsetospeed (&tty, B38400);
  cfsetispeed (&tty, B38400);

  // Set port settings
  tty.c_cflag     &=  ~CSTOPB;	// stop byte
  tty.c_cflag     &=  ~CRTSCTS;   // no flow control
  tty.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
  tty.c_oflag     =   0;      	// no remapping, no delays
  tty.c_cc[VMIN]  =   0;      	// read doesn't block
  tty.c_cc[VTIME] =   5;      	// 0.5 seconds read timeout

  tty.c_cflag     |=  CREAD | CLOCAL; // turn on READ & ignore ctrl lines
  tty.c_iflag     &=  ~IXOFF;			// turn off s/w flow ctrl

  // set terminal to raw mod
  cfmakeraw(&tty);
  // Flush Port, then applies attributes
  tcflush(m_dev, TCIFLUSH );

  if (tcsetattr (m_dev, TCSANOW, &tty) != 0)
  {
    ROS_ERROR("Error %d from tcsetattr:  %s", errno, strerror(errno));
  }

  // Allocate memory for read and write buffer
  memset(&m_read_buffer, '\0', sizeof(m_read_buffer));

  m_thruster_cmd[MAGIC_START_BYTE_INDEX] = MAGIC_START_BYTE;
  m_thruster_cmd[MAGIC_STOP_BYTE_INDEX] = MAGIC_STOP_BYTE;
  m_thruster_cmd[MSG_TYPE_INDEX] = MSG_TYPE_THRUSTER;

  m_heartbeat_cmd[MAGIC_START_BYTE_INDEX] = MAGIC_START_BYTE;
  m_heartbeat_cmd[MSG_HEARTBEAT_SIZE-1] = MAGIC_STOP_BYTE;
  m_heartbeat_cmd[MSG_TYPE_INDEX] = MSG_TYPE_HEARTBEAT;

  m_arming_cmd[MAGIC_START_BYTE_INDEX] = MAGIC_START_BYTE;
  m_arming_cmd[MSG_ARMING_SIZE-1] = MAGIC_STOP_BYTE;
  m_arming_cmd[MSG_TYPE_INDEX] = MSG_TYPE_ARM;

  m_light_cmd[MAGIC_START_BYTE_INDEX] = MAGIC_START_BYTE;
  m_light_cmd[MSG_LIGHT_SIZE-1] = MAGIC_STOP_BYTE;
  m_light_cmd[MSG_TYPE_INDEX] = MSG_TYPE_LIGHT;
}


int McuInterface::serial_write(char* cmd, int cmd_size)
{
  return write(m_dev, cmd, cmd_size);
}


int McuInterface::serial_read()
{
  return read(m_dev, &m_read_buffer, sizeof(m_read_buffer));
}


void McuInterface::thruster_pwm_callback(const vortex_msgs::Pwm& msg)
{
  int i;
  ROS_DEBUG("I heard: ");

  for (i = 0; i < 8; i++)
  {
    ROS_DEBUG("%d ", msg.positive_width_us[i]);
  }

  for (i = 0; i < 8; i++)
  {
    m_thruster_cmd[MSG_PAYLOAD_START_INDEX + i*2] = (uint8_t)(msg.positive_width_us[i]  >> 8);
    m_thruster_cmd[MSG_PAYLOAD_START_INDEX + i*2 + 1] = (uint8_t)(msg.positive_width_us[i]  & 0xFF);
  }

  int16_t checksum = crc_checksum(&m_thruster_cmd[MSG_PAYLOAD_START_INDEX],
                                 (MSG_PAYLOAD_STOP_INDEX - MSG_PAYLOAD_START_INDEX + 1));

  m_thruster_cmd[MSG_CRC_BYTE_INDEX] 	 = (uint8_t)(checksum  >> 8);
  m_thruster_cmd[MSG_CRC_BYTE_INDEX + 1] = (uint8_t)(checksum  & 0xFF);

  ROS_DEBUG("THRUSTER COMMAND: %s", m_thruster_cmd);

  // Error Handling
  if (serial_write(&m_thruster_cmd[0], MAX_MSG_SIZE) < 0)
  {
    ROS_WARN("THRUSTER - ERROR WRITING: [ %s ]", strerror(errno));
  }
}


void McuInterface::heartbeat_callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_DEBUG("I heard: [%s]", msg->data.c_str());

  // Error Handling
  if (serial_write(&m_heartbeat_cmd[0], MSG_HEARTBEAT_SIZE) < 0)
  {
    ROS_WARN("HEARTBEAT - ERROR WRITING: [ %s ]", strerror(errno));
  }

  if (serial_read() < 0)
  {
    ROS_DEBUG("HEARTBEAT - ERROR READING: [ %s ]", strerror(errno));
  }

  clear_read_buffer();
}


void McuInterface::arming_callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_DEBUG("I heard: [%s]", msg->data.c_str());

  if(msg->data == "ARM" || msg->data == "arm")
  {
    m_arming_cmd[MSG_TYPE_INDEX] = MSG_TYPE_ARM;
  }
  else
  {
    m_arming_cmd[MSG_TYPE_INDEX] = MSG_TYPE_DISARM;
  }

  // Error Handling
  if (serial_write(&m_arming_cmd[0], MSG_ARMING_SIZE) < 0)
  {
    ROS_WARN("ARMING - ERROR WRITING: [ %s ]", strerror(errno));
  }
}

void McuInterface::light_pwm_callback(const std_msgs::Int16::ConstPtr& msg)
{
  m_light_cmd[MSG_PAYLOAD_START_INDEX] = (uint8_t)(msg->data  >> 8);
  m_light_cmd[MSG_PAYLOAD_START_INDEX + 1] = (uint8_t)(msg->data & 0xFF);

  int16_t checksum = crc_checksum(&m_light_cmd[MSG_PAYLOAD_START_INDEX], 2);

  m_light_cmd[4] = (uint8_t)(checksum  >> 8);
  m_light_cmd[5] = (uint8_t)(checksum  & 0xFF);

  // Error Handling
  if (serial_write(&m_light_cmd[0], MSG_LIGHT_SIZE) < 0)
  {
    ROS_DEBUG("LIGHT PWM - ERROR WRITING: [ %s ]", strerror(errno));
  }
}

uint8_t McuInterface::read_leak_sensor()
{
  char leak_detection_msg[] = "LEAK DETECTED";
  if (serial_read() < 0)
  {
    ROS_DEBUG("READ LEAK SENSOR - ERROR READING: [ %s ]", strerror(errno));
  }
  else
  {
    ROS_INFO("MCU: [ %s ]", m_read_buffer);

    int i = 0;
    bool match = false;
    for (i = 0; i < sizeof(leak_detection_msg) - 1; i++)
    {
      if (m_read_buffer[i] == leak_detection_msg[i])
      {
        match = true;
      }
      else
      {
        match = false;
      }
    }

    clear_read_buffer();

    if (match)
    {
      return 1;
    }
  }

  clear_read_buffer();
  return 0;
}

uint16_t McuInterface::crc_checksum(char *input, uint8_t num)
{
  return crc_16((const unsigned char*)input, (unsigned long)num);
}

void McuInterface::clear_read_buffer()
{
  std::fill(std::begin(m_read_buffer), std::end(m_read_buffer), 0);
  tcflush(m_dev, TCIFLUSH );
}

McuInterface::~McuInterface()
{
  close(m_dev);
}
