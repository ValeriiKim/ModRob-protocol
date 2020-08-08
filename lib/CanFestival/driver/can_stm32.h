#include "config.h"
#include "can_driver.h"

#include "can.h"

unsigned char canInit();
unsigned char canSend(CAN_PORT notused, Message *m);
unsigned char canReceive(Message *m);
unsigned char canChangeBaudRate(CAN_HANDLE fd, char *baud);
 