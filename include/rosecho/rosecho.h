#ifndef __ROSECHO_H__
#define __ROSECHO_H__

#include "ros/ros.h"
#include "serial.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "stdint.h"
#include <stdio.h>

#define OPEN 0
#define WEP 1
#define WPA 2

#define RECV_BUF_LEN 12
#define MSG_NORMAL_LEN 4
#define MSG_EXTRA_LEN 8
#define PACKET_LEN_BIT 4
#define SYNC_HEAD 0xa5
#define SYNC_HEAD_SECOND 0x01

#define DEFAULT_SERIAL_DEVICE "/dev/ttyS4"
#define DEFAULT_WIFI_SSID "TianbotOffice"
#define DEFAULT_WIFI_PASSWORD "www.tianbot.com"

class Rosecho
{
public:
  void tts(uint8_t flag, const char *str, const char *emot);
  void cfg(const char *config);
  void wifi_status(void);
  static void serial_data_proc(uint8_t *data, unsigned int data_len, void *param);
  Rosecho(void);

private:
  Serial serial;
  void wifi_cfg(const char *ssid, const char *password, uint8_t mode);
  void ack(void);
  void tts_cb(const std_msgs::String::ConstPtr &msg);
  void cfg_cb(const std_msgs::String::ConstPtr &msg);
  void rosecho_data_proc(unsigned char *buf, int len);
  std::string param_ssid;
  std::string param_password;
  std::string param_serial_port;
  ros::Publisher asr_pub;
  ros::Publisher status_pub;
  ros::Publisher wakeup_pos_pub;
  ros::Subscriber tts_sub;
  ros::Subscriber cfg_sub;
  uint16_t id;
};

#endif
