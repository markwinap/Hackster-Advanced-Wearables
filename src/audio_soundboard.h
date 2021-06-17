#ifndef _SOUNDBOARD_H_
#define _SOUNDBOARD_H_

#include <inttypes.h>
#include <device.h>
#include <drivers/uart.h>/*UART*/

void sendCommand(const struct device *uart_dev, uint8_t *cmd);
void playTrack(const struct device *uart_dev, char *track);

#endif /* _SOUNDBOARD_H_ */