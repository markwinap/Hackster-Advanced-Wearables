#ifndef _CONFIG_H_
#define _CONFIG_H_

/* Parameters */
#define UART_BAUDRATE 9600
#define UART_LABEL "UART_1"
#define UART_AUDIO "UART_2"
#define COMMAND_SIZE 2
#define TRACK_SIZE 13

#define SLEEP_TIME_MS 1
/* size of stack area used by each thread */
#define STACKSIZE 2048
/* scheduling priority used by each thread */
#define PRIORITY 7
//TIME BUTTON
#define SW0_NODE DT_ALIAS(sw0)
#define SW0_GPIO_LABEL DT_GPIO_LABEL(SW0_NODE, gpios)
#define SW0_GPIO_PIN DT_GPIO_PIN(SW0_NODE, gpios)
#define SW0_GPIO_FLAGS (GPIO_INPUT | DT_GPIO_FLAGS(SW0_NODE, gpios))
//SPEED BUTTON
#define SW1_NODE DT_ALIAS(sw1)
#define SW1_GPIO_LABEL DT_GPIO_LABEL(SW1_NODE, gpios)
#define SW1_GPIO_PIN DT_GPIO_PIN(SW1_NODE, gpios)
#define SW1_GPIO_FLAGS (GPIO_INPUT | DT_GPIO_FLAGS(SW1_NODE, gpios))
//UV BUTTON
#define SW2_NODE DT_ALIAS(sw2)
#define SW2_GPIO_LABEL DT_GPIO_LABEL(SW2_NODE, gpios)
#define SW2_GPIO_PIN DT_GPIO_PIN(SW2_NODE, gpios)
#define SW2_GPIO_FLAGS (GPIO_INPUT | DT_GPIO_FLAGS(SW2_NODE, gpios))

#endif /* _CONFIG_H_ */