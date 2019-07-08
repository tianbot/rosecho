#include "rosecho.h"
#include "cJSON.h"
#include "gzip.h"
#include "serial.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h>

void Rosecho::tts_cb(const std_msgs::String::ConstPtr &msg, void *param) {
    Rosecho *p = (Rosecho *)param;
    p->tts(1, (char *)(msg->data.c_str()), "happy");
}

int16_t wakeup_pos = 0;

void Rosecho::cfg_cb(const std_msgs::String::ConstPtr &msg, void *param) {
    Rosecho *p = (Rosecho *)param;
    p->cfg((char *)msg->data.c_str());
}

Rosecho::Rosecho(void) {
    ros::NodeHandle n("rosecho");

    n.param<std::string>("serial_port", param_serial_port,
                         DEFAULT_SERIAL_DEVICE);
    n.param<std::string>("wifi_ssid", param_ssid, DEFAULT_WIFI_SSID);
    n.param<std::string>("wifi_password", param_password,
                         DEFAULT_WIFI_PASSWORD);

    if (serial.open(param_serial_port.c_str(), 115200, 0, 8, 1, 'N',
                    serial_data_proc) != true) {
        ROS_ERROR("serial error\n");
        exit(-1);
    }

    ack();

    asr_pub = n.advertise<std_msgs::String>("/rosecho/aiui_asr", 1000);
    status_pub = n.advertise<std_msgs::String>("/rosecho/status", 1000);
    wakeup_pos_pub = n.advertise<std_msgs::Int16>("/rosecho/wakeup_pos", 1000);
    ros::Subscriber tts_sub =
        n.subscribe("/rosecho/tts", 1000, boost::bind(tts_cb, _1, this));
    ros::Subscriber cfg_sub =
        n.subscribe("/rosecho/cfg", 1000, boost::bind(cfg_cb, _1, this));
}

void Rosecho::wifi_cfg(const char *ssid, const char *password, uint8_t mode) {
    uint8_t buf[1024];
    uint16_t offset = 0;

    uint16_t len = strlen(ssid) + strlen(password) + 4;

    uint8_t checksum = 0;

    int i;

    id++;

    buf[offset++] = 0xA5;
    buf[offset++] = 0x01;

    buf[offset++] = 0x02;

    buf[offset++] = len & 0xFF;
    buf[offset++] = (len >> 8) & 0xFF;

    buf[offset++] = id & 0xFF;
    buf[offset++] = (id >> 8) & 0xFF;

    buf[offset++] = 0x00;
    buf[offset++] = mode;
    buf[offset++] = strlen(ssid);
    buf[offset++] = strlen(password);

    for (i = 0; i < strlen(ssid); i++) {
        buf[offset++] = ssid[i];
    }

    for (i = 0; i < strlen(password); i++) {
        buf[offset++] = password[i];
    }

    for (i = 0; i < offset; i++) {
        checksum += buf[i];
    }
    checksum = (~checksum) + 1;

    buf[offset++] = checksum;

    serial.send(buf, offset);
}

void Rosecho::wifi_status(void) {
    cJSON *root, *content;
    uint16_t len;
    uint8_t buf[1024];
    uint16_t offset = 0;
    char *out;

    uint8_t checksum = 0;

    int i;

    id++;

    buf[offset++] = 0xA5;
    buf[offset++] = 0x01;

    buf[offset++] = 0x05;

    offset += 2;

    buf[offset++] = id & 0xFF;
    buf[offset++] = (id >> 8) & 0xFF;

    root = cJSON_CreateObject();

    cJSON_AddItemToObject(root, "type", cJSON_CreateString("status"));
    cJSON_AddItemToObject(root, "content", content = cJSON_CreateObject());

    cJSON_AddItemToObject(content, "query", cJSON_CreateString("wifi"));

    out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    len = strlen(out);

    buf[3] = len & 0xFF;
    buf[4] = (len >> 8) & 0xFF;

    for (i = 0; i < len; i++) {
        buf[offset++] = out[i];
    }

    free(out);

    for (i = 0; i < offset; i++) {
        checksum += buf[i];
    }
    checksum = (~checksum) + 1;

    buf[offset++] = checksum;

    serial.send(buf, offset);
}

// emot is not used by iflytek now
void Rosecho::tts(uint8_t flag, const char *str, const char *emot) {
    cJSON *root, *content, *parameters;
    uint16_t len;
    uint8_t buf[8192];
    uint16_t offset = 0;
    char *out;

    uint8_t checksum = 0;

    int i;

    id++;

    buf[offset++] = 0xA5;
    buf[offset++] = 0x01;

    buf[offset++] = 0x05;

    offset += 2;

    buf[offset++] = id & 0xFF;
    buf[offset++] = (id >> 8) & 0xFF;

    root = cJSON_CreateObject();

    cJSON_AddItemToObject(root, "type", cJSON_CreateString("tts"));
    cJSON_AddItemToObject(root, "content", content = cJSON_CreateObject());

    if (flag) {
        cJSON_AddItemToObject(content, "action", cJSON_CreateString("start"));
        cJSON_AddItemToObject(content, "text", cJSON_CreateString(str));
        // cJSON_AddItemToObject(content, "parameters", parameters =
        // cJSON_CreateObject());
        // cJSON_AddItemToObject(parameters,"emot",cJSON_CreateString(emot));
    } else {
        cJSON_AddItemToObject(content, "action", cJSON_CreateString("stop"));
    }

    out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    len = strlen(out);

    buf[3] = len & 0xFF;
    buf[4] = (len >> 8) & 0xFF;

    for (i = 0; i < len; i++) {
        buf[offset++] = out[i];
    }

    free(out);

    for (i = 0; i < offset; i++) {
        checksum += buf[i];
    }
    checksum = (~checksum) + 1;

    buf[offset++] = checksum;

    serial.send(buf, offset);
}

void Rosecho::cfg(const char *config) {
    cJSON *root, *content;
    uint16_t len;
    uint8_t buf[1024];
    uint16_t offset = 0;
    char *out;

    uint8_t checksum = 0;

    int i;

    if (strcmp("wifi", config) == 0) {
        wifi_cfg(param_ssid.c_str(), param_password.c_str(), WPA);
        return;
    } else if (strcmp("wifi_status", config) == 0) {
        wifi_status();
        return;
    }

    id++;

    buf[offset++] = 0xA5;
    buf[offset++] = 0x01;

    buf[offset++] = 0x05;

    offset += 2;

    buf[offset++] = id & 0xFF;
    buf[offset++] = (id >> 8) & 0xFF;

    root = cJSON_CreateObject();

    cJSON_AddItemToObject(root, "type", cJSON_CreateString("aiui_msg"));
    cJSON_AddItemToObject(root, "content", content = cJSON_CreateObject());

    if (strcmp("enable", config) == 0) {
        cJSON_AddItemToObject(content, "msg_type", cJSON_CreateNumber(5));
    } else if (strcmp("disable", config) == 0) {
        cJSON_AddItemToObject(content, "msg_type", cJSON_CreateNumber(6));
    } else if (strcmp("wakeup", config) == 0) {
        cJSON_AddItemToObject(content, "msg_type", cJSON_CreateNumber(7));
    } else if (strcmp("sleep", config) == 0) {
        cJSON_AddItemToObject(content, "msg_type", cJSON_CreateNumber(8));
    } else if (strcmp("state", config) == 0) {
        cJSON_AddItemToObject(content, "msg_type", cJSON_CreateNumber(1));
    }
    cJSON_AddItemToObject(content, "arg1", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(content, "arg2", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(content, "params", cJSON_CreateString(""));

    out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    len = strlen(out);

    buf[3] = len & 0xFF;
    buf[4] = (len >> 8) & 0xFF;

    for (i = 0; i < len; i++) {
        buf[offset++] = out[i];
    }

    free(out);

    for (i = 0; i < offset; i++) {
        checksum += buf[i];
    }
    checksum = (~checksum) + 1;

    buf[offset++] = checksum;

    serial.send(buf, offset);
}

void Rosecho::ack(void) {
    int i;
    uint8_t ack_buf[12];
    ack_buf[0] = 0xA5;
    ack_buf[1] = 0x01;
    ack_buf[2] = 0xff; // 0xff是确认消息类型
    ack_buf[3] = 0x04;
    ack_buf[4] = 0x00;
    ack_buf[5] = id & 0xff; // 消息ID同要收到消息ID相同
    ack_buf[6] = (id >> 8) & 0xFF;
    ack_buf[7] = 0xA5;
    ack_buf[8] = 0x00;
    ack_buf[9] = 0x00;
    ack_buf[10] = 0x00;

    //计算校检码
    char check_code = 0;
    for (i = 0; i <= 10; i++) {
        check_code += ack_buf[i];
    }
    check_code = ~check_code + 1;
    ack_buf[11] = check_code;
    serial.send(ack_buf, sizeof(ack_buf));
}

void Rosecho::rosecho_data_proc(unsigned char *buf, int len) {
    int i;
    if (buf[2] == 0xff) {
        //对确认消息先不处理
        return;
    }

    unsigned char unzip_buf[100 * 1024];
    int unzip_len = sizeof(unzip_buf);

    id = buf[5] + buf[6] * 256;
    ack();

    if (buf[2] != 0x04) {
        return;
    }

    if (gzdecompress(buf + 7, buf[3] + buf[4] * 256, unzip_buf,
                     (ulong *)&unzip_len) != 0) {
        ROS_ERROR("gzip error\n");
        return;
    }

    unzip_buf[unzip_len] = '\0';
    cJSON *json, *p;
    char key[4][10] = {"content", "result", "intent", "text"};
    json = cJSON_Parse((const char *)unzip_buf);
    if (!json) {
        // printf("Error before: [%s]\n",cJSON_GetErrorPtr());
        ROS_ERROR("Error before: [%s]\n", cJSON_GetErrorPtr());
    } else {
        int i;
        p = json;
        for (i = 0; i < 4; i++) {
            p = cJSON_GetObjectItem(p, key[i]);
            if (!p) {
                break;
            }
        }
        if (i == 4) {
            char *out = cJSON_Print(p);
            std_msgs::String asr_msg;
            asr_msg.data = out;
            asr_pub.publish(asr_msg);
            free(out);
        }

        p = cJSON_GetObjectItem(json, "type");
        if (p) {
            if (strcmp(p->valuestring, "tts_event") == 0) {
                p = cJSON_GetObjectItem(json, "content");
                if (p) {
                    p = cJSON_GetObjectItem(p, "eventType");
                    if (p) {
                        if (p->valueint == 0) {
                            std_msgs::String status_msg;
                            status_msg.data = "tts:start";
                            status_pub.publish(status_msg);
                        } else {
                            std_msgs::String status_msg;
                            status_msg.data = "tts:end";
                            status_pub.publish(status_msg);
                        }
                    }
                }
            } else if (strcmp(p->valuestring, "aiui_event") == 0) {
                p = cJSON_GetObjectItem(json, "content");
                if (p) {
                    cJSON *q;
                    q = cJSON_GetObjectItem(p, "eventType");
                    if (q) {
                        if (q->valueint == 5) // sleep event
                        {
                            std_msgs::String status_msg;
                            status_msg.data = "state:sleep";
                            status_pub.publish(status_msg);
                        } else if (q->valueint == 4) // wakeup event
                        {
                            q = cJSON_GetObjectItem(p, "info");
                            if (q) {
                                q = cJSON_GetObjectItem(q, "angle");
                                if (q) {
                                    std_msgs::Int16 wakeup_pos_msg;
                                    wakeup_pos_msg.data = q->valueint;
                                    wakeup_pos_pub.publish(wakeup_pos_msg);
                                }
                            }
                        }
                    }
                }
            } else if (strcmp(p->valuestring, "wifi_status") == 0) {
                p = cJSON_GetObjectItem(json, "content");
                if (p) {
                    cJSON *q;
                    q = cJSON_GetObjectItem(p, "connected");
                    if (q) {
                        if (q->type == cJSON_False) {
                            std_msgs::String status_msg;
                            status_msg.data = "wifi:false";
                            status_pub.publish(status_msg);
                        } else if (q->type == cJSON_True) {
                            std_msgs::String status_msg;
                            status_msg.data = "wifi:true";
                            q = cJSON_GetObjectItem(p, "ssid");
                            if (q) {
                                status_msg.data += q->valuestring;
                            }
                            status_pub.publish(status_msg);
                        }
                    }
                }
            }
        }

        cJSON_Delete(json);
    }
}

void Rosecho::serial_data_proc(uint8_t *data, unsigned int data_len) {
    static uint8_t state = 0;
    uint8_t *p = data;
    static uint8_t recv_msg[10 * 1024];
    static uint32_t recv_msg_len;
    static uint32_t len;
    uint32_t j;

    while (data_len != 0) {
        switch (state) {
        case 0:
            if (*p == SYNC_HEAD) {
                recv_msg_len = 0;
                recv_msg[recv_msg_len++] = SYNC_HEAD;
                state = 1;
            }
            p++;
            data_len--;
            break;

        case 1:
            if (*p == SYNC_HEAD_SECOND) {
                recv_msg[recv_msg_len++] = SYNC_HEAD_SECOND;
                p++;
                data_len--;
                state = 2;
            } else {
                state = 0;
            }
            break;

        case 2:
            recv_msg[recv_msg_len++] = *p;
            p++;
            data_len--;
            state = 3;
            break;

        case 3: // len
            recv_msg[recv_msg_len++] = *p;
            len = *p;
            p++;
            data_len--;
            state = 4;
            break;

        case 4: // len
            recv_msg[recv_msg_len++] = *p;
            len += (*p) * 256;
            if (len > 1024 * 10) {
                state = 0;
                break;
            }
            p++;
            data_len--;
            state = 5;
            break;

        case 5: // id
            recv_msg[recv_msg_len++] = *p;
            p++;
            data_len--;
            state = 6;
            break;

        case 6: // id
            recv_msg[recv_msg_len++] = *p;
            p++;
            data_len--;
            state = 7;
            break;

        case 7: //
            if (len--) {
                recv_msg[recv_msg_len++] = *p;
                p++;
                data_len--;
            } else {
                int i;
                uint8_t crc = 0;
                recv_msg[recv_msg_len++] = *p;
                p++;
                data_len--;
                state = 0;
                for (i = 0; i < recv_msg_len - 1; i++) {
                    crc += recv_msg[i];
                }
                crc = (~crc) + 1;
                if (crc == recv_msg[recv_msg_len - 1]) {
                    rosecho_data_proc(recv_msg, recv_msg_len); //接受消息处理
                } else {
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
