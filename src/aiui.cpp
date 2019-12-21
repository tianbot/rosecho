#include "aiui.h"
#include <stdio.h>
#include <sstream>
#include <stdlib.h>
using namespace std;

Aiui::Aiui(string serial_port)
{
    asrCB_ = NULL;
    answerCB_ = NULL;
    ttsStartCB_ = NULL;
    ttsFinishCB_ = NULL;
    sleepCB_ = NULL;
    wakeCB_ = NULL;
    wifiConnectCB_ = NULL;
    wifiDisconnectCB_ = NULL;
    if (serial_.open(serial_port.c_str(), 115200, 0, 8, 1, 'N',
                     boost::bind(&Aiui::serialDataProc, this, _1, _2)) != true)
    {
        exit(-1);
    }

    ack();
}

void Aiui::asrCallbackRegister(aiui_cb_str cb)
{
    asrCB_ = cb;
}

void Aiui::answerCallbackRegister(aiui_cb_str cb)
{
    answerCB_ = cb;
}

void Aiui::ttsStartCallbackRegister(aiui_cb_noparam cb)
{
    ttsStartCB_ = cb;
}

void Aiui::ttsFinishCallbackRegister(aiui_cb_noparam cb)
{
    ttsFinishCB_ = cb;
}

void Aiui::sleepCallbackRegister(aiui_cb_noparam cb)
{
    sleepCB_ = cb;
}

void Aiui::wakeCallbackRegister(aiui_cb_int cb)
{
    wakeCB_ = cb;
}

void Aiui::wifiDisconnectCallbackRegister(aiui_cb_noparam cb)
{
    wifiDisconnectCB_ = cb;
}

void Aiui::wifiConnectCallbackRegister(aiui_cb_str cb)
{
    wifiConnectCB_ = cb;
}

void Aiui::tts(uint8_t flag, const char *str, const char *emot)
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

void Aiui::wifiCfg(const char *ssid, const char *password, uint8_t mode)
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

void Aiui::wifiStatusCheck(void)
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

void Aiui::cfg(const char *config)
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

void Aiui::disable(void)
{
    cfg("disable");
}

void Aiui::enable(void)
{
    cfg("enable");
}

void Aiui::sleep(void)
{
    cfg("sleep");
}

void Aiui::wakeup(void)
{
    cfg("wakeup");
}

void Aiui::ack(void)
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

void Aiui::serialDataProc(uint8_t *data, unsigned int data_len)
{
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
                    aiuiDataProc(&recv_msg[0], recv_msg.size()); // process recv msg
                }
                else
                {
                    printf("crc error\n");
                }
            }
            break;

        default:
            state = 0;
            break;
        }
    }
}

void Aiui::aiuiDataProc(unsigned char *buf, int len)
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
        printf("gzip error\n");
        return;
    }
    unzip_buf[unzip_len] = '\0';
    cJSON *json, *p;
    char key_asr[4][10] = {"content", "result", "intent", "text"};
    char key_answer[5][10] = {"content", "result", "intent", "answer", "text"};
    json = cJSON_Parse((const char *)unzip_buf);
    if (!json)
    {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
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
            if (asrCB_ != NULL)
            {
                string asrStr(out);
                asrCB_(asrStr);
            }
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
            if (answerCB_ != NULL)
            {
                string answerStr(out);
                answerCB_(answerStr);
            }
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
                            if (ttsStartCB_ != NULL)
                            {
                                ttsStartCB_();
                            }
                        }
                        else
                        {
                            if (ttsFinishCB_ != NULL)
                            {
                                ttsFinishCB_();
                            }
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
                            if (sleepCB_ != NULL)
                            {
                                sleepCB_();
                            }
                        }
                        else if (q->valueint == 4) // wakeup event
                        {
                            q = cJSON_GetObjectItem(p, "info");
                            if (q)
                            {
                                q = cJSON_GetObjectItem(q, "angle");
                                if (q)
                                {
                                    if (wakeCB_ != NULL)
                                    {
                                        wakeCB_(360 - q->valueint);
                                    }
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
                            if (wifiDisconnectCB_ != NULL)
                            {
                                wifiDisconnectCB_();
                            }
                        }
                        else if (q->type == cJSON_True)
                        {
                            q = cJSON_GetObjectItem(p, "ssid");
                            if (q)
                            {
                                if (wifiConnectCB_ != NULL)
                                {
                                    string ssid(q->valuestring);
                                    wifiConnectCB_(ssid);
                                }
                            }
                        }
                    }
                }
            }
        }

        cJSON_Delete(json);
    }
}
