// BSD 3-Clause License
// 
// Copyright (c) 2019, TIANBOT
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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

#define DEFAULT_SERIAL_DEVICE "/dev/ttyUSB0"
#define DEFAULT_WIFI_SSID "TianbotOffice"
#define DEFAULT_WIFI_PASSWORD "www.tianbot.com"

class Rosecho
{
public:
  void tts(uint8_t flag, const char *str, const char *emot);
  void cfg(const char *config);
  void checkWifiStatus(void);
  static void serialDataProc(uint8_t *data, unsigned int data_len, void *param);
  Rosecho(void);

private:
  void wifiCfg(const char *ssid, const char *password, uint8_t mode);
  void ack(void);
  void ttsCallback(const std_msgs::String::ConstPtr &msg);
  void cfgCallback(const std_msgs::String::ConstPtr &msg);
  void rosechoDataProc(unsigned char *buf, int len);
  std::string param_ssid_;
  std::string param_password_;
  std::string param_serial_port_;
  ros::Publisher asr_pub_;
  ros::Publisher status_pub_;
  ros::Publisher wakeup_pos_pub_;
  ros::Subscriber tts_sub_;
  ros::Subscriber cfg_sub_;
  ros::NodeHandle nh_;
  uint16_t id_;
  Serial serial_;
};

#endif
