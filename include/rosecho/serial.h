#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "stdint.h"
#include <pthread.h>

typedef void (*serial_recv_cb)(uint8_t *data, unsigned int data_len);

class Serial {
  public:
    int fd;
    bool open(const char *device, int rate, int flow_ctrl, int databits,
              int stopbits, int parity, serial_recv_cb cb);
    int send(uint8_t *data, int len);
    int running;
    serial_recv_cb recv_cb;
    void close(void);

  private:
    bool config(int speed, int flow_ctrl, int databits, int stopbits,
                int parity);
    pthread_t recv_thread;
};

#endif
