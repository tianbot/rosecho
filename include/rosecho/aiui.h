#ifndef _AIUI_H_
#define _AIUI_H_

#include "cJSON.h"
#include "gzip.h"
#include "serial.h"
#include <vector>
#include "boost/bind.hpp"
#include "boost/function.hpp"

using namespace std;
using namespace boost;

#define OPEN 0
#define WEP 1
#define WPA 2

#define RECV_BUF_LEN 12
#define MSG_NORMAL_LEN 4
#define MSG_EXTRA_LEN 8
#define PACKET_LEN_BIT 4
#define SYNC_HEAD 0xa5
#define SYNC_HEAD_SECOND 0x01

typedef boost::function<void(string str)> aiui_cb_str;
typedef boost::function<void(int n)> aiui_cb_int;
typedef boost::function<void(void)> aiui_cb_noparam;

class Aiui
{
public:
    void enable(void);
    void disable(void);
    void wakeup(void);
    void sleep(void);
    void wifiStatusCheck(void);
    void sleepDelay(void);
    void tts(uint8_t flag, const char *str, const char *emot);
    void wifiCfg(const char *ssid, const char *password, uint8_t mode);
    void asrCallbackRegister(aiui_cb_str cb);
    void answerCallbackRegister(aiui_cb_str cb);
    void ttsStartCallbackRegister(aiui_cb_noparam cb);
    void ttsFinishCallbackRegister(aiui_cb_noparam cb);
    void sleepCallbackRegister(aiui_cb_noparam cb);
    void wakeCallbackRegister(aiui_cb_int cb);
    void wifiDisconnectCallbackRegister(aiui_cb_noparam cb);
    void wifiConnectCallbackRegister(aiui_cb_str cb);
    Aiui(string serial_port);

private:
    void ack(void);
    void serialDataProc(uint8_t *data, unsigned int data_len);
    void aiuiDataProc(unsigned char *buf, int len);
    void cfg(const char *config);
    uint16_t id_;
    Serial serial_;
    bool isAnswerFlag_;
    aiui_cb_str asrCB_;
    aiui_cb_str answerCB_;
    aiui_cb_noparam ttsStartCB_;
    aiui_cb_noparam ttsFinishCB_;
    aiui_cb_noparam sleepCB_;
    aiui_cb_int wakeCB_;
    aiui_cb_noparam wifiDisconnectCB_;
    aiui_cb_str wifiConnectCB_;
};

#endif
