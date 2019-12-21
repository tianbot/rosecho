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
#include "aiui.h"
#include "serial.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "rosecho/ttsAction.h"
#include "actionlib/server/simple_action_server.h"
#include "rosecho/WifiCfg.h"
#include "std_srvs/Empty.h"
#include "stdint.h"

using namespace std;

#define BACKEND_AIUI

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

class Rosecho_tts
{
public:
#ifdef BACKEND_AIUI
    Rosecho_tts(ros::NodeHandle nh_, std::string name, Aiui *p);
#else
#error "No backend device defined"
#endif
    void ttsStartCallback(void);
    void ttsFinishCallback(void);
    void goalCB();
    void preemptCB();

protected:
    actionlib::SimpleActionServer<rosecho::ttsAction> as_;
    rosecho::ttsResult tts_result_;
    std::string tts_text_;
#ifdef BACKEND_AIUI
    Aiui *backend_;
#else
#error "No backend device defined"
#endif
};

class Rosecho
{
public:
    Rosecho(void);

private:
    ros::Publisher asr_pub_;
    ros::Publisher answer_pub_;
    ros::Publisher wakeup_pos_pub_;
    ros::NodeHandle nh_;

#ifdef BACKEND_AIUI
    Aiui *backend_;
#else
#error "No backend device defined"
#endif
    Rosecho_tts *rosecho_tts_;
    void answerCallback(string str);
    void asrCallback(string str);
    void wakeCallback(int angle);
    void wifiConnectCallback(string str);
    void wifiDisconnectCallback(void);
    bool wifiCfg(rosecho::WifiCfg::Request &req, rosecho::WifiCfg::Response &res);
    bool enable(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool disable(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool wakeup(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool sleep(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    bool isWifiConnected_;
    std::string ssid_;
};

#endif
