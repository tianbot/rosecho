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
#include "cJSON.h"
#include "gzip.h"
#include "serial.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h>
#include <vector>

using namespace std;

void Rosecho::ttsCallback(const std_msgs::String::ConstPtr &msg)
{
    tts(1, (char *)(msg->data.c_str()), "happy");
}

void Rosecho::cfgCallback(const std_msgs::String::ConstPtr &msg)
{
    cfg((char *)msg->data.c_str());
}

Rosecho::Rosecho(void)
{
    nh_ = ros::NodeHandle("rosecho");

    nh_.param<std::string>("serial_port", param_serial_port_,
                           DEFAULT_SERIAL_DEVICE);
    nh_.param<std::string>("wifi_ssid", param_ssid_, DEFAULT_WIFI_SSID);
    nh_.param<std::string>("wifi_password", param_password_,
                           DEFAULT_WIFI_PASSWORD);

    if (serial_.open(param_serial_port_.c_str(), 115200, 0, 8, 1, 'N',
                     serialDataProc, this) != true)
    {
        ROS_ERROR("serial error\n");
        exit(-1);
    }

    ack();

    asr_pub_ = nh_.advertise<std_msgs::String>("/rosecho/asr", 1000);
    answer_pub_ = nh_.advertise<std_msgs::String>("/rosecho/answer", 1000);
    status_pub_ = nh_.advertise<std_msgs::String>("/rosecho/status", 1000);
    wakeup_pos_pub_ = nh_.advertise<std_msgs::Int16>("/rosecho/wakeup_pos", 1000);
    tts_sub_ = nh_.subscribe("/rosecho/tts", 1000, &Rosecho::ttsCallback, this);
    cfg_sub_ = nh_.subscribe("/rosecho/cfg", 1000, &Rosecho::cfgCallback, this);
}

void Rosecho::wifiCfg(const char *ssid, const char *password, uint8_t mode)
{
    vector<uint8_t> buf;

    uint16_t len = strlen(ssid) + strlen(password) + 4;

    uint8_t checksum = 0;

    int i;

    id_++;

    buf.push_back(0xA5);
    buf.push_back(0x01);

    buf.push_back(0x02);

    buf.push_back(len & 0xFF);
    buf.push_back((len >> 8) & 0xFF);

    buf.push_back(id_ & 0xFF);
    buf.push_back((id_ >> 8) & 0xFF);

    buf.push_back(0x00);
    buf.push_back(mode);
    buf.push_back(strlen(ssid));
    buf.push_back(strlen(password));

    for (i = 0; i < strlen(ssid); i++)
    {
        buf.push_back(ssid[i]);
    }

    for (i = 0; i < strlen(password); i++)
    {
        buf.push_back(password[i]);
    }

    for (i = 0; i < buf.size(); i++)
    {
        checksum += buf[i];
    }
    checksum = (~checksum) + 1;

    buf.push_back(checksum);

    serial_.send(&buf[0], buf.size());
    buf.clear();
}

void Rosecho::checkWifiStatus(void)
{
    cJSON *root, *content;
    uint16_t len;
    vector<uint8_t> buf;
    char *out;

    uint8_t checksum = 0;

    int i;

    id_++;

    buf.push_back(0xA5);
    buf.push_back(0x01);

    buf.push_back(0x05);

    root = cJSON_CreateObject();

    cJSON_AddItemToObject(root, "type", cJSON_CreateString("status"));
    cJSON_AddItemToObject(root, "content", content = cJSON_CreateObject());

    cJSON_AddItemToObject(content, "query", cJSON_CreateString("wifi"));

    out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    len = strlen(out);

    buf.push_back(len & 0xFF);
    buf.push_back((len >> 8) & 0xFF);

    buf.push_back(id_ & 0xFF);
    buf.push_back((id_ >> 8) & 0xFF);

    for (i = 0; i < len; i++)
    {
        buf.push_back(out[i]);
    }

    free(out);

    for (i = 0; i < buf.size(); i++)
    {
        checksum += buf[i];
    }
    checksum = (~checksum) + 1;

    buf.push_back(checksum);

    serial_.send(&buf[0], buf.size());
    buf.clear();
}

// emot is not used by iflytek now
void Rosecho::tts(uint8_t flag, const char *str, const char *emot)
{
    cJSON *root, *content, *parameters;
    uint16_t len;
    vector<uint8_t> buf;
    char *out;

    uint8_t checksum = 0;

    int i;

    id_++;

    buf.push_back(0xA5);
    buf.push_back(0x01);

    buf.push_back(0x05);

    root = cJSON_CreateObject();

    cJSON_AddItemToObject(root, "type", cJSON_CreateString("tts"));
    cJSON_AddItemToObject(root, "content", content = cJSON_CreateObject());

    if (flag)
    {
        cJSON_AddItemToObject(content, "action", cJSON_CreateString("start"));
        cJSON_AddItemToObject(content, "text", cJSON_CreateString(str));
        // cJSON_AddItemToObject(content, "parameters", parameters =
        // cJSON_CreateObject());
        // cJSON_AddItemToObject(parameters,"emot",cJSON_CreateString(emot));
    }
    else
    {
        cJSON_AddItemToObject(content, "action", cJSON_CreateString("stop"));
    }

    out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    len = strlen(out);

    buf.push_back(len & 0xFF);
    buf.push_back((len >> 8) & 0xFF);

    buf.push_back(id_ & 0xFF);
    buf.push_back((id_ >> 8) & 0xFF);

    for (i = 0; i < len; i++)
    {
        buf.push_back(out[i]);
    }

    free(out);

    for (i = 0; i < buf.size(); i++)
    {
        checksum += buf[i];
    }
    checksum = (~checksum) + 1;

    buf.push_back(checksum);

    serial_.send(&buf[0], buf.size());
    buf.clear();
}

void Rosecho::cfg(const char *config)
{
    cJSON *root, *content;
    uint16_t len;
    vector<uint8_t> buf;
    char *out;

    uint8_t checksum = 0;

    int i;

    if (strcmp("wifi", config) == 0)
    {
        wifiCfg(param_ssid_.c_str(), param_password_.c_str(), WPA);
        return;
    }
    else if (strcmp("wifi_status", config) == 0)
    {
        checkWifiStatus();
        return;
    }

    id_++;

    buf.push_back(0xA5);
    buf.push_back(0x01);

    buf.push_back(0x05);

    root = cJSON_CreateObject();

    cJSON_AddItemToObject(root, "type", cJSON_CreateString("aiui_msg"));
    cJSON_AddItemToObject(root, "content", content = cJSON_CreateObject());

    if (strcmp("enable", config) == 0)
    {
        cJSON_AddItemToObject(content, "msg_type", cJSON_CreateNumber(5));
    }
    else if (strcmp("disable", config) == 0)
    {
        cJSON_AddItemToObject(content, "msg_type", cJSON_CreateNumber(6));
    }
    else if (strcmp("wakeup", config) == 0)
    {
        cJSON_AddItemToObject(content, "msg_type", cJSON_CreateNumber(7));
    }
    else if (strcmp("sleep", config) == 0)
    {
        cJSON_AddItemToObject(content, "msg_type", cJSON_CreateNumber(8));
    }
    else if (strcmp("state", config) == 0)
    {
        cJSON_AddItemToObject(content, "msg_type", cJSON_CreateNumber(1));
    }
    cJSON_AddItemToObject(content, "arg1", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(content, "arg2", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(content, "params", cJSON_CreateString(""));

    out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    len = strlen(out);

    buf.push_back(len & 0xFF);
    buf.push_back((len >> 8) & 0xFF);

    buf.push_back(id_ & 0xFF);
    buf.push_back((id_ >> 8) & 0xFF);

    for (i = 0; i < len; i++)
    {
        buf.push_back(out[i]);
    }

    free(out);

    for (i = 0; i < buf.size(); i++)
    {
        checksum += buf[i];
    }
    checksum = (~checksum) + 1;

    buf.push_back(checksum);

    serial_.send(&buf[0], buf.size());
    buf.clear();
}

void Rosecho::ack(void)
{
    int i;
    uint8_t checksum = 0;
    vector<uint8_t> buf;
    buf.push_back(0xA5);
    buf.push_back(0x01);
    buf.push_back(0xff); // 0xff confirm msg type
    buf.push_back(0x04);
    buf.push_back(0x00);
    buf.push_back(id_ & 0xff); // Msg ID should be the same as recv msg
    buf.push_back((id_ >> 8) & 0xFF);
    buf.push_back(0xA5);
    buf.push_back(0x00);
    buf.push_back(0x00);
    buf.push_back(0x00);

    //checksum
    for (i = 0; i < buf.size(); i++)
    {
        checksum += buf[i];
    }
    checksum = ~checksum + 1;
    buf.push_back(checksum);

    serial_.send(&buf[0], buf.size());
    buf.clear();
}

void Rosecho::rosechoDataProc(unsigned char *buf, int len)
{
    int i;
    if (buf[2] == 0xff)
    {
        //Do nothing
        return;
    }

    unsigned char unzip_buf[100 * 1024];
    int unzip_len = sizeof(unzip_buf);

    id_ = buf[5] + buf[6] * 256;
    ack();

    if (buf[2] != 0x04)
    {
        return;
    }

    if (gzdecompress(buf + 7, buf[3] + buf[4] * 256, unzip_buf,
                     (ulong *)&unzip_len) != 0)
    {
        ROS_ERROR("gzip error\n");
        return;
    }
    unzip_buf[unzip_len] = '\0';
    cJSON *json, *p;
    char key_asr[4][10] = {"content", "result", "intent", "text"};
    char key_answer[5][10] = {"content", "result", "intent", "answer", "text"};
    json = cJSON_Parse((const char *)unzip_buf);
    if (!json)
    {
        // printf("Error before: [%s]\n",cJSON_GetErrorPtr());
        ROS_ERROR("Error before: [%s]\n", cJSON_GetErrorPtr());
    }
    else
    {
        int i;
        p = json;
        for (i = 0; i < 4; i++)
        {
            p = cJSON_GetObjectItem(p, key_asr[i]);
            if (!p)
            {
                break;
            }
        }
        if (i == 4)
        {
            char *out = cJSON_Print(p);
            std_msgs::String asr_msg;
            asr_msg.data = out;
            ROS_DEBUG("%s\n", asr_msg.data.c_str());
            asr_pub_.publish(asr_msg);
            free(out);
        }

        p = json;
        for (i = 0; i < 5; i++)
        {
            p = cJSON_GetObjectItem(p, key_answer[i]);
            if (!p)
            {
                break;
            }
        }
        if (i == 5)
        {
            char *out = cJSON_Print(p);
            std_msgs::String answer_msg;
            answer_msg.data = out;
            ROS_DEBUG("%s\n", answer_msg.data.c_str());
            answer_pub_.publish(answer_msg);
            free(out);
        }
        p = cJSON_GetObjectItem(json, "type");
        if (p)
        {
            if (strcmp(p->valuestring, "tts_event") == 0)
            {
                p = cJSON_GetObjectItem(json, "content");
                if (p)
                {
                    p = cJSON_GetObjectItem(p, "eventType");
                    if (p)
                    {
                        if (p->valueint == 0)
                        {
                            std_msgs::String status_msg;
                            status_msg.data = "tts:start";
                            status_pub_.publish(status_msg);
                        }
                        else
                        {
                            std_msgs::String status_msg;
                            status_msg.data = "tts:end";
                            status_pub_.publish(status_msg);
                        }
                    }
                }
            }
            else if (strcmp(p->valuestring, "aiui_event") == 0)
            {
                p = cJSON_GetObjectItem(json, "content");
                if (p)
                {
                    cJSON *q;
                    q = cJSON_GetObjectItem(p, "eventType");
                    if (q)
                    {
                        if (q->valueint == 5) // sleep event
                        {
                            std_msgs::String status_msg;
                            status_msg.data = "state:sleep";
                            status_pub_.publish(status_msg);
                        }
                        else if (q->valueint == 4) // wakeup event
                        {
                            q = cJSON_GetObjectItem(p, "info");
                            if (q)
                            {
                                q = cJSON_GetObjectItem(q, "angle");
                                if (q)
                                {
                                    std_msgs::Int16 wakeup_pos_msg;
                                    wakeup_pos_msg.data = 360 - q->valueint;
                                    wakeup_pos_pub_.publish(wakeup_pos_msg);
                                }
                            }
                        }
                    }
                }
            }
            else if (strcmp(p->valuestring, "wifi_status") == 0)
            {
                p = cJSON_GetObjectItem(json, "content");
                if (p)
                {
                    cJSON *q;
                    q = cJSON_GetObjectItem(p, "connected");
                    if (q)
                    {
                        if (q->type == cJSON_False)
                        {
                            std_msgs::String status_msg;
                            status_msg.data = "wifi:false";
                            status_pub_.publish(status_msg);
                        }
                        else if (q->type == cJSON_True)
                        {
                            std_msgs::String status_msg;
                            status_msg.data = "wifi:true";
                            q = cJSON_GetObjectItem(p, "ssid");
                            if (q)
                            {
                                status_msg.data += q->valuestring;
                            }
                            status_pub_.publish(status_msg);
                        }
                    }
                }
            }
        }

        cJSON_Delete(json);
    }
}

void Rosecho::serialDataProc(uint8_t *data, unsigned int data_len, void *param)
{
    Rosecho *pThis = (Rosecho *)param;
    static uint8_t state = 0;
    uint8_t *p = data;
    static vector<uint8_t> recv_msg;
    static uint32_t len;
    uint32_t j;

    while (data_len != 0)
    {
        switch (state)
        {
        case 0:
            if (*p == SYNC_HEAD)
            {
                recv_msg.clear();
                recv_msg.push_back(SYNC_HEAD);
                state = 1;
            }
            p++;
            data_len--;
            break;

        case 1:
            if (*p == SYNC_HEAD_SECOND)
            {
                recv_msg.push_back(SYNC_HEAD_SECOND);
                p++;
                data_len--;
                state = 2;
            }
            else
            {
                state = 0;
            }
            break;

        case 2:
            recv_msg.push_back(*p);
            p++;
            data_len--;
            state = 3;
            break;

        case 3: // len
            recv_msg.push_back(*p);
            len = *p;
            p++;
            data_len--;
            state = 4;
            break;

        case 4: // len
            recv_msg.push_back(*p);
            len += (*p) * 256;
            if (len > 1024 * 10)
            {
                state = 0;
                break;
            }
            p++;
            data_len--;
            state = 5;
            break;

        case 5: // id
            recv_msg.push_back(*p);
            p++;
            data_len--;
            state = 6;
            break;

        case 6: // id
            recv_msg.push_back(*p);
            p++;
            data_len--;
            state = 7;
            break;

        case 7: //
            if (len--)
            {
                recv_msg.push_back(*p);
                p++;
                data_len--;
            }
            else
            {
                int i;
                uint8_t crc = 0;
                recv_msg.push_back(*p);
                p++;
                data_len--;
                state = 0;
                for (i = 0; i < recv_msg.size() - 1; i++)
                {
                    crc += recv_msg[i];
                }
                crc = (~crc) + 1;
                if (crc == recv_msg[recv_msg.size() - 1])
                {
                    pThis->rosechoDataProc(&recv_msg[0], recv_msg.size()); // process recv msg
                }
                else
                {
                    ROS_ERROR("crc error");
                }
            }
            break;

        default:
            state = 0;
            break;
        }
    }
}
