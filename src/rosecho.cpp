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

#include "rosecho.h"

#ifdef BACKEND_AIUI
#include "aiui.h"
#else
#error "No backend device defined"
#endif
using namespace std;

#ifdef BACKEND_AIUI
Rosecho_tts::Rosecho_tts(ros::NodeHandle nh_, std::string name, Aiui *p) : as_(nh_, name, false), backend_(p)
#else
#error "No backend device defined"
#endif
{
    backend_->ttsFinishCallbackRegister(boost::bind(&Rosecho_tts::ttsFinishCallback, this));
    backend_->ttsStartCallbackRegister(boost::bind(&Rosecho_tts::ttsStartCallback, this));
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&Rosecho_tts::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&Rosecho_tts::preemptCB, this));
    as_.start();
}

void Rosecho_tts::ttsStartCallback(void)
{
    tts_result_.is_finished = false;
}

void Rosecho_tts::ttsFinishCallback(void)
{
    tts_result_.is_finished = true;
    as_.setSucceeded(tts_result_);
}

void Rosecho_tts::goalCB()
{
    tts_result_.is_finished = 0;
    tts_text_ = as_.acceptNewGoal()->text;
    backend_->tts(1, (char *)tts_text_.c_str(), "happy");
}

void Rosecho_tts::preemptCB()
{
    backend_->tts(0, NULL, NULL);
    as_.setPreempted();
}

Rosecho::Rosecho(ros::NodeHandle *nh):nh_(*nh)
{
    std::string param_serial_port;
    //nh_ = ros::NodeHandle("rosecho");

    nh_.param<std::string>("serial_port", param_serial_port,
                           DEFAULT_SERIAL_DEVICE);

    asr_pub_ = nh_.advertise<std_msgs::String>("asr", 1000);
    answer_pub_ = nh_.advertise<std_msgs::String>("answer", 1000);
    wakeup_pos_pub_ = nh_.advertise<std_msgs::Int16>("wakeup_pos", 1000);

    wifiCfgService_ = nh_.advertiseService<rosecho::WifiCfg::Request, rosecho::WifiCfg::Response>("wifi_cfg", boost::bind(&Rosecho::wifiCfg, this, _1, _2));
    enableService_ = nh_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("enable", boost::bind(&Rosecho::enable, this, _1, _2));
    disableService_ = nh_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("disable", boost::bind(&Rosecho::disable, this, _1, _2));
    sleepService_ = nh_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("sleep", boost::bind(&Rosecho::sleep, this, _1, _2));
    wakeupService_ = nh_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("wakeup", boost::bind(&Rosecho::wakeup, this, _1, _2));
#ifdef BACKEND_AIUI
    backend_ = new Aiui(param_serial_port);
#else
#error "No backend device defined"
#endif

    backend_->asrCallbackRegister(boost::bind(&Rosecho::asrCallback, this, _1));
    backend_->answerCallbackRegister(boost::bind(&Rosecho::answerCallback, this, _1));
    backend_->wakeCallbackRegister(boost::bind(&Rosecho::wakeCallback, this, _1));
    backend_->wifiConnectCallbackRegister(boost::bind(&Rosecho::wifiConnectCallback, this, _1));
    backend_->wifiDisconnectCallbackRegister(boost::bind(&Rosecho::wifiDisconnectCallback, this));

    rosecho_tts_ = new Rosecho_tts(nh_, "tts", backend_);
}

bool Rosecho::wifiCfg(rosecho::WifiCfg::Request &req, rosecho::WifiCfg::Response &res)
{
    int count = 20;
    isWifiConnected_ = false;
    backend_->wifiCfg(req.ssid.c_str(), req.password.c_str(), WPA);
    do
    {
        ros::Duration(0.5).sleep();
        backend_->wifiStatusCheck();
    } while (!isWifiConnected_ && count--);
    res.connected = isWifiConnected_;
    if (res.connected)
    {
        res.ssid = ssid_;
    }
    else
    {
        res.ssid = req.ssid;
    }

    return true;
}

bool Rosecho::enable(std_srvs::Empty::Request &req,  std_srvs::Empty::Response &res)
{
    backend_->enable();
    ros::Duration(1).sleep();//wait backend ready
    return true;
}

bool Rosecho::disable(std_srvs::Empty::Request &req,  std_srvs::Empty::Response &res)
{
    backend_->disable();
    return true;
}

bool Rosecho::wakeup(std_srvs::Empty::Request &req,  std_srvs::Empty::Response &res)
{
    backend_->wakeup();
    return true;
}

bool Rosecho::sleep(std_srvs::Empty::Request &req,  std_srvs::Empty::Response &res)
{
    backend_->sleep();
    return true;
}

void Rosecho::asrCallback(string str)
{
    std_msgs::String asr_msg;
    asr_msg.data = str;
    asr_pub_.publish(asr_msg);
}

void Rosecho::answerCallback(string str)
{
    std_msgs::String answer_msg;
    answer_msg.data = str;
    answer_pub_.publish(answer_msg);
}

void Rosecho::wakeCallback(int angle)
{
    std_msgs::Int16 wakeup_pos_msg;
    wakeup_pos_msg.data = angle;
    wakeup_pos_pub_.publish(wakeup_pos_msg);
}

void Rosecho::wifiConnectCallback(string str)
{
    ssid_ = str;
    //ROS_INFO("SSID [%s] connect", ssid_.c_str());
    isWifiConnected_ = true;
}

void Rosecho::wifiDisconnectCallback(void)
{
    //ROS_INFO("Wifi disconnect");
    isWifiConnected_ = false;
}
