#include "audio_soundboard.h"
#include <string.h>
#include "config.h"

void sendCommand(const struct device *uart_dev, uint8_t *cmd)//Send Any command over UART
{
	int ret = uart_tx(uart_dev, cmd, COMMAND_SIZE, 100);
	if (ret) {
		printk("Error unable to send command (%d)\n", ret);
	}
};

void playTrack(const struct device *uart_dev, char *track)//Send Track over UART
{
	char text[13];
	strcpy(text, track);
	// printk("%s\n", text);
	// printk("%s\n", track);
	// printk("Size (%d)\n", sizeof(text), text);
	int ret = uart_tx(uart_dev, track, 13, 100);
	if (ret) {
		printk("Error unable to send command (%d)\n", ret);
	}
};