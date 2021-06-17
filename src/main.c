#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>
#include <string.h>
#include <drivers/i2c.h> /*I2C*/
#include <drivers/uart.h>/*UART*/
#include "config.h"
#include "uv_config.h"
#include "audio_soundboard.h"
#include "minmea.h"
// #include <time.h>

//Statically define and initialize a semaphore.
K_SEM_DEFINE(tx_done, 0, 1);
K_SEM_DEFINE(tx_aborted, 0, 1);
K_SEM_DEFINE(rx_rdy, 0, 1);
K_SEM_DEFINE(rx_buf_released, 0, 1);
K_SEM_DEFINE(rx_disabled, 0, 1);

// atically define and initialize a FIFO queue.
K_FIFO_DEFINE(audio_fifo);

volatile size_t read_len;

volatile uint8_t rx_buf[300] = { 0 };

volatile size_t sent;
volatile size_t received;
volatile bool failed_in_isr;

volatile uint8_t seconds;
volatile uint8_t minutes;
volatile uint8_t hours;
volatile float speed;
volatile uint8_t uv;
volatile uint8_t uv_index = 0;
volatile uint8_t kph = 0;

volatile uint8_t prev_uv_index = 0;
volatile uint8_t prev_kph = 0;
volatile uint8_t countdown = 0;

const struct device *uart_dev;
const struct device *uart_audio_dev;
const struct device *i2c_dev;
const struct device *button;
const struct device *speed_button;
const struct device *uv_button;

// Structs
struct audio_data_t {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	uint16_t timer;
	char txt[13];
};

struct rx_source {
	int cnt;
	uint8_t prev;
};

const struct uart_config uart_cfg = { .baudrate = 9600,
				      .parity = UART_CFG_PARITY_NONE,
				      .stop_bits = UART_CFG_STOP_BITS_1,
				      .data_bits = UART_CFG_DATA_BITS_8,
				      .flow_ctrl = UART_CFG_FLOW_CTRL_NONE };

// function declaration
void insertQueueAudio(struct audio_data_t *tx_data);
static struct gpio_callback button_cb_data;
static struct gpio_callback speed_button_cb_data;
static struct gpio_callback uv_button_cb_data;

void insertQueueAudio(struct audio_data_t *tx_data)//Insert into FIFO queue
{
	size_t size = sizeof(struct audio_data_t);
	char *mem_ptr = k_malloc(size);//Allocate memory from the heap.
	__ASSERT_NO_MSG(mem_ptr != 0);
	memcpy(mem_ptr, tx_data, size);
	k_fifo_put(&audio_fifo, mem_ptr);//Add an element to a FIFO queue.
}

void createString(uint8_t a)//Create string command for Audio FX Track
{
	char str[2];
	char phrase[13] = "P        OGG\n";
	sprintf(str, "%d", a);
	uint8_t i;
	uint8_t len = strlen(str);
	//printk("LEN %d -%s-\n", len, str);
	for (i = 0; i < len; ++i) {
		phrase[i + 1] = str[i];
	}
	// printk("-%s-\n", phrase);
	struct audio_data_t tx_data1 = { .timer = a < 11 && a > 19 ? 1000 : 1100 };//Give for time for teens eg fourteen
	strcpy(tx_data1.txt, phrase);
	insertQueueAudio(&tx_data1);
}
void uv_pressed()
{//UV Button Pressed Callback
	struct audio_data_t tx_uv = { .timer = 2000, .txt = "PUV      OGG\n" };
	insertQueueAudio(&tx_uv);
	//Get UV index
	//https://en.wikipedia.org/wiki/Ultraviolet_index#Index_usage
	switch (uv_index) {
	case 0:;
		struct audio_data_t tx_none = { .timer = 1000,
						.txt = "PNONE    OGG\n" };
		insertQueueAudio(&tx_none);
		break;
	case 1:;
	case 2:;
		struct audio_data_t tx_low = { .timer = 1000,
					       .txt = "PLOW     OGG\n" };
		insertQueueAudio(&tx_low);
		break;
	case 3:;
	case 4:;
	case 5:;
		struct audio_data_t tx_moderate = { .timer = 1000,
						    .txt = "PMODERATEOGG\n" };
		insertQueueAudio(&tx_moderate);
		break;
	case 6:;
	case 7:;
		struct audio_data_t tx_high = { .timer = 1000,
						.txt = "PHIGH    OGG\n" };
		insertQueueAudio(&tx_high);
		break;
	case 8:;
	case 9:;
	case 10:;
		struct audio_data_t tx_vhigh = { .timer = 1000,
						 .txt = "PVHIGH    OGG\n" };
		insertQueueAudio(&tx_vhigh);
		break;
	default:;
		struct audio_data_t tx_extreme = { .timer = 1000,
						   .txt = "PEXTREME OGG\n" };
		insertQueueAudio(&tx_extreme);
	}
}
void speed_pressed()//const struct device *dev, struct gpio_callback *cb, uint32_t pins
{//Speed Button Pressed Callback
	// printk("SPEED %.2f, %d\n", speed, kph);
	// struct audio_data_t tx_speed = { .timer = 1200,
	// 				 .txt = "PSPEED   OGG\n" };
	// insertQueueAudio(&tx_speed);
	createString(kph);
	struct audio_data_t tx_kph = { .timer = 1400, .txt = "PKPH     OGG\n" };
	insertQueueAudio(&tx_kph);
}
void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{//Time Buton Pressed Callback
	uint8_t rem = minutes % 10;
	uint8_t csth = (hours < 5) ? (19 + hours) : (hours - 5);//CST
	uint8_t remh = csth % 10;
	printk("%d:%d:%d\n", hours, minutes, seconds);
	printk("%d:%d:%d\n", csth, minutes, seconds);

	if (remh > 0 && csth > 19) {
		createString(csth - remh);
		createString(remh);
	} else {
		createString(csth);
	}

	struct audio_data_t tx_dataHours = { .timer = 800,
					     .txt = "PHOURS   OGG\n" };
	insertQueueAudio(&tx_dataHours);
	if (rem > 0 && minutes > 19) {
		createString(minutes - rem);
		createString(rem);
	} else {
		createString(minutes);
	}
	struct audio_data_t tx_dataMinutes = { .timer = 700,
					       .txt = "PMINUTES OGG\n" };
	insertQueueAudio(&tx_dataMinutes);
}
bool StartsWith(const char *a, const char *b)//Get if string starts with other string usefull for checinkg prefix
{
	if (strncmp(a, b, strlen(b)) == 0)
		return 1;
	return 0;
}
void parseNamea()
{
	char *rest = rx_buf;
	char *nmea;
	// GPGGA - Global Positioning System Fix Data
	// GPGSA - GPS DOP and active satellites
	// GPRMC - Recommended minimum specific GPS/Transit data
	// GPGSV - GPS Satellites in view
	// GPVTG - Track made good and ground speed
	// http://aprs.gids.nl/nmea/

	while ((nmea = strtok_r(rest, "\r\n", &rest))) {
		switch (minmea_sentence_id(nmea, false)) {
		// $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
		// 1    = UTC of Position
		// 2    = Latitude
		// 3    = N or S
		// 4    = Longitude
		// 5    = E or W
		// 6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
		// 7    = Number of satellites in use [not those in view]
		// 8    = Horizontal dilution of position
		// 9    = Antenna altitude above/below mean sea level (geoid)
		// 10   = Meters  (Antenna height unit)
		// 11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
		//        mean sea level.  -=geoid is below WGS-84 ellipsoid)
		// 12   = Meters  (Units of geoidal separation)
		// 13   = Age in seconds since last update from diff. reference station
		// 14   = Diff. reference station ID#
		// 15   = Checksum
		case MINMEA_SENTENCE_GGA: {
			// printk("%s\n", nmea);
			struct minmea_sentence_gga frame;
			if (minmea_parse_gga(&frame, nmea)) {
				seconds = frame.time.seconds;
				minutes = frame.time.minutes;
				hours = frame.time.hours;

				// printk("$xxGGA: fix quality: %d\n",
				//        frame.fix_quality);
				// printk("SATELITES %d \n",
				//        frame.satellites_tracked);
				// printf("ALTITUD3 %.2f \n",
				//        minmea_tofloat(&frame.altitude));
				// printk("SEC %d \n", frame.time.seconds);

			} else {
				printk("$xxGGA sentence is not parsed\n");
			}
		} break;
		// $GPVTG,t,T,,,s.ss,N,s.ss,K*hh
		// 1    = Track made good
		// 2    = Fixed text 'T' indicates that track made good is relative to true north
		// 3    = not used
		// 4    = not used
		// 5    = Speed over ground in knots
		// 6    = Fixed text 'N' indicates that speed over ground in in knots
		// 7    = Speed over ground in kilometers/hour
		// 8    = Fixed text 'K' indicates that speed over ground is in kilometers/hour
		// 9    = Checksum
		case MINMEA_SENTENCE_VTG: {
			// printk("%s\n", nmea);
			struct minmea_sentence_vtg frame;
			if (minmea_parse_vtg(&frame, nmea)) {
				speed = minmea_tofloat(&frame.speed_kph);
				kph = speed;
				// printf("SPEED %.2f \n",
				//        minmea_tofloat(&frame.speed_kph));
				// struct minmea_float true_track_degrees;
				// struct minmea_float magnetic_track_degrees;
				// struct minmea_float speed_knots;
				// struct minmea_float speed_kph;
				// enum minmea_faa_mode faa_mode;

			} else {
				printk("$xxGGA sentence is not parsed\n");
			}
		} break;
		}
	}
}

void uart_read_callback(const struct device *uart_dev, struct uart_event *evt,
			void *user_data)
{
	// List of Events
	//https://docs.zephyrproject.org/latest/reference/peripherals/uart.html?highlight=uart_rx_rdy#c.uart_event_type.UART_TX_DONE
	switch (evt->type) {
	case UART_TX_DONE://Whole TX buffer was transmitted.
		k_sem_give(&tx_done);
		break;
	case UART_RX_RDY://Received data is ready for processing.
		read_len = evt->data.rx.len;
		k_sem_give(&rx_rdy);
		//printk("%s\n", rx_buf);
		// printk("LEN %d\n", evt->data.rx.len);
		// printk("Event %d\n", evt->type);
		parseNamea();
		int ret4 = uart_rx_enable(uart_dev, rx_buf, 300, 50);//Start receiving data through UART. // intuart_rx_enable(conststructdevice*dev, uint8_t*buf, size_tlen, int32_ttimeout)
		if (ret4 != 0) {
			printk("uart_rx_enable Error %d\n", ret4);
			return;
		}
		break;
	case UART_RX_DISABLED://RX has been disabled and can be reenabled.
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}
}
uint16_t read16(const struct device *i2c_dev, uint16_t addr)
{
	uint8_t cmp_data[16];
	uint8_t data[16];
	int ret;
	data[0] = 0x00;
	cmp_data[0] = addr;
	ret = i2c_write_read(i2c_dev, 0x60, &cmp_data[0], 1, &data[0], 2);
	if (ret) {
		printk("Error writing to FRAM! error code (%d)\n", ret);
	}
	return data[0] | data[1] << 8;
}
void write8(const struct device *i2c_dev, uint16_t addr, uint16_t data)
{
	int ret;
	uint8_t cmp_data[16];
	cmp_data[0] = addr;
	cmp_data[1] = data;
	ret = i2c_write(i2c_dev, &cmp_data, 2, SI1145_ADDR);
	if (ret) {
		printk("Error writing code (%d)\n", ret);
		return;
	}
}
static int write_bytes(const struct device *i2c_dev, uint16_t addr,
		       uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* FRAM address */
	wr_addr[0] = (addr >> 8) & 0xFF;
	wr_addr[1] = addr & 0xFF;

	/* Setup I2C messages */

	/* Send the address to write to */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Data to be written, and STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, SI1145_ADDR);
}
static int read_bytes(const struct device *i2c_dev, uint16_t addr,
		      uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* Now try to read back from FRAM */

	/* FRAM address */
	wr_addr[0] = (addr >> 8) & 0xFF;
	wr_addr[1] = addr & 0xFF;

	/* Setup I2C messages */

	/* Send the address to read from */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Read from device. STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, SI1145_ADDR);
}
uint8_t writeParam(const struct device *i2c_dev, uint8_t p, uint8_t v)
{
	write8(i2c_dev, SI1145_REG_PARAMWR, v);
	write8(i2c_dev, SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
	//   return read_bytes(i2c_dev, SI1145_REG_PARAMRD, 1);

	int i, ret;
	uint8_t data[16];
	data[0] = 0x00;
	ret = read_bytes(i2c_dev, SI1145_REG_PARAMRD, &data[0], 1);
	if (ret) {
		printk("Error reading from FRAM! error code (%d)\n", ret);
		return;
	}
	return data[0];
}
void reset(const struct device *i2c_dev)
{
	write8(i2c_dev, SI1145_REG_MEASRATE0, 0);
	write8(i2c_dev, SI1145_REG_MEASRATE1, 0);
	write8(i2c_dev, SI1145_REG_IRQEN, 0);
	write8(i2c_dev, SI1145_REG_IRQMODE1, 0);
	write8(i2c_dev, SI1145_REG_IRQMODE2, 0);
	write8(i2c_dev, SI1145_REG_INTCFG, 0);
	write8(i2c_dev, SI1145_REG_IRQSTAT, 0xFF);

	write8(i2c_dev, SI1145_REG_COMMAND, SI1145_RESET);
	k_msleep(10);
	write8(i2c_dev, SI1145_REG_HWKEY, 0x17);
	k_msleep(10);
}
void SI1145(const struct device *i2c_dev)
{
	reset(i2c_dev);
	// enable UVindex measurement coefficients!
	write8(i2c_dev, SI1145_REG_UCOEFF0, 0x29);
	write8(i2c_dev, SI1145_REG_UCOEFF1, 0x89);
	write8(i2c_dev, SI1145_REG_UCOEFF2, 0x02);
	write8(i2c_dev, SI1145_REG_UCOEFF3, 0x00);

	// enable UV sensor
	writeParam(i2c_dev, SI1145_PARAM_CHLIST,
		   SI1145_PARAM_CHLIST_ENUV | SI1145_PARAM_CHLIST_ENALSIR |
			   SI1145_PARAM_CHLIST_ENALSVIS |
			   SI1145_PARAM_CHLIST_ENPS1);
	// enable interrupt on every sample
	write8(i2c_dev, SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);
	write8(i2c_dev, SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE);

	// program LED current
	write8(i2c_dev, SI1145_REG_PSLED21, 0x03); // 20mA for LED 1 only
	writeParam(i2c_dev, SI1145_PARAM_PS1ADCMUX,
		   SI1145_PARAM_ADCMUX_LARGEIR);
	// prox sensor #1 uses LED #1
	writeParam(i2c_dev, SI1145_PARAM_PSLED12SEL,
		   SI1145_PARAM_PSLED12SEL_PS1LED1);
	// fastest clocks, clock div 1
	writeParam(i2c_dev, SI1145_PARAM_PSADCGAIN, 0);
	// take 511 clocks to measure
	writeParam(i2c_dev, SI1145_PARAM_PSADCOUNTER,
		   SI1145_PARAM_ADCCOUNTER_511CLK);
	// in prox mode, high range
	writeParam(i2c_dev, SI1145_PARAM_PSADCMISC,
		   SI1145_PARAM_PSADCMISC_RANGE |
			   SI1145_PARAM_PSADCMISC_PSMODE);

	writeParam(i2c_dev, SI1145_PARAM_ALSIRADCMUX,
		   SI1145_PARAM_ADCMUX_SMALLIR);
	// fastest clocks, clock div 1
	writeParam(i2c_dev, SI1145_PARAM_ALSIRADCGAIN, 0);
	// take 511 clocks to measure
	writeParam(i2c_dev, SI1145_PARAM_ALSIRADCOUNTER,
		   SI1145_PARAM_ADCCOUNTER_511CLK);
	// in high range mode
	writeParam(i2c_dev, SI1145_PARAM_ALSIRADCMISC,
		   SI1145_PARAM_ALSIRADCMISC_RANGE);

	// fastest clocks, clock div 1
	writeParam(i2c_dev, SI1145_PARAM_ALSVISADCGAIN, 0);
	// take 511 clocks to measure
	writeParam(i2c_dev, SI1145_PARAM_ALSVISADCOUNTER,
		   SI1145_PARAM_ADCCOUNTER_511CLK);
	// in high range mode (not normal signal)
	writeParam(i2c_dev, SI1145_PARAM_ALSVISADCMISC,
		   SI1145_PARAM_ALSVISADCMISC_VISRANGE);

	// measurement rate for auto
	write8(i2c_dev, SI1145_REG_MEASRATE0, 0xFF); // 255 * 31.25uS = 8ms

	// auto run
	write8(i2c_dev, SI1145_REG_COMMAND, SI1145_PSALS_AUTO);
}

void main(void)
{
	printk("Hello World!\n");
	//I2C
	i2c_dev = device_get_binding("I2C_3");
	if (i2c_dev == NULL) {
		printk("Error: I2C didn't find device\n");
		return;
	}
	uint32_t dev_config =
		I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_MASTER;//I2C Standard Speed: 100 kHz and Controller to act as Master.
	int i2c_config = i2c_configure(i2c_dev, dev_config);//Configure operation of a host controller.
	if (i2c_config != 0) {
		printk("Error: %d\n", i2c_config);
		return;
	}
	SI1145(i2c_dev);//Init UV Sensor

	// Input Time Pin
	button = device_get_binding(SW0_GPIO_LABEL);
	if (button == NULL) {
		printk("Error: PIN0 - didn't find %s device\n", SW0_GPIO_LABEL);
		return;
	}

	int ret = gpio_pin_configure(button, SW0_GPIO_PIN, SW0_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n", ret,
		       SW0_GPIO_LABEL, SW0_GPIO_PIN);
		return;
	}

	int ret2 = gpio_pin_interrupt_configure(button, SW0_GPIO_PIN,
						GPIO_INT_EDGE_TO_ACTIVE);
	if (ret2 != 0) {
		printk("gpio_pin_interrupt_configure Error %d: failed to configure interrupt on %s pin %d\n",
		       ret2, SW0_GPIO_LABEL, SW0_GPIO_PIN);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(SW0_GPIO_PIN));
	gpio_add_callback(button, &button_cb_data);
	printk("Set up button at %s pin %d\n", SW0_GPIO_LABEL, SW0_GPIO_PIN);

	//Speed Button
	speed_button = device_get_binding(SW1_GPIO_LABEL);
	if (speed_button == NULL) {
		printk("Error: PIN1 - didn't find %s device\n", SW1_GPIO_LABEL);
		return;
	}
	int speed_config =
		gpio_pin_configure(speed_button, SW1_GPIO_PIN, SW1_GPIO_FLAGS);
	if (speed_config != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       speed_config, SW1_GPIO_LABEL, SW1_GPIO_PIN);
		return;
	}
	int speed_int = gpio_pin_interrupt_configure(speed_button, SW1_GPIO_PIN,
						     GPIO_INT_EDGE_TO_ACTIVE);
	if (speed_int != 0) {
		printk("gpio_pin_interrupt_configure Error %d: failed to configure interrupt on %s pin %d\n",
		       speed_int, SW1_GPIO_LABEL, SW1_GPIO_PIN);
		return;
	}
	gpio_init_callback(&speed_button_cb_data, speed_pressed,
			   BIT(SW1_GPIO_PIN));
	gpio_add_callback(speed_button, &speed_button_cb_data);
	printk("Set up button at %s pin %d\n", SW1_GPIO_LABEL, SW1_GPIO_PIN);

	//UV Button
	uv_button = device_get_binding(SW2_GPIO_LABEL);
	if (uv_button == NULL) {
		printk("Error: PIN2 - didn't find %s device\n", SW2_GPIO_LABEL);
		return;
	}
	int uv_config =
		gpio_pin_configure(uv_button, SW2_GPIO_PIN, SW2_GPIO_FLAGS);//Configure a single pin.
	if (uv_config != 0) {
		printk("Error %d: failed to configure %s pin %d\n", uv_config,
		       SW2_GPIO_LABEL, SW2_GPIO_PIN);
		return;
	}
	int uv_int = gpio_pin_interrupt_configure(uv_button, SW2_GPIO_PIN,
						  GPIO_INT_EDGE_TO_ACTIVE);//Configure pin interrupt on logical level 1 and enables it.
	if (uv_int != 0) {
		printk("gpio_pin_interrupt_configure Error %d: failed to configure interrupt on %s pin %d\n",
		       uv_int, SW2_GPIO_LABEL, SW2_GPIO_PIN);
		return;
	}
	gpio_init_callback(&uv_button_cb_data, uv_pressed, BIT(SW2_GPIO_PIN));//Helper to initialize a struct gpio_callback properly
	gpio_add_callback(uv_button, &uv_button_cb_data);//Set button callback function
	printk("Set up button at %s pin %d\n", SW2_GPIO_LABEL, SW2_GPIO_PIN);

	// UART
	uart_dev = device_get_binding(UART_LABEL);
	if (uart_dev == NULL) {
		printk("device_get_binding Error\n"); 
		return;
	}

	int ret3 = uart_configure(uart_dev, &uart_cfg);//Configure UART device
	if (ret3 != 0) {
		printk("uart_configure Error %d\n", ret3);
		return;
	}

	int ret4 = uart_rx_enable(uart_dev, rx_buf, 300, 50);//Start receiving data through UART. 
	if (ret4 != 0) {
		printk("uart_rx_enable Error %d\n", ret4);
		return;
	}

	int ret5 = uart_callback_set(uart_dev, uart_read_callback, NULL);//Create UART callback for incoming data
	if (ret5 != 0) {
		printk("uart_callback_set Error %d\n", ret5);
		return;
	}
}
void audio_fx_out(void)
{
	while (1) {
		/*Read from and delete from FIFO*/
		//https://docs.zephyrproject.org/latest/reference/kernel/data_passing/fifos.html?highlight=k_fifo_put#c.k_fifo_put_list
		struct audio_data_t *rx_data =
			k_fifo_get(&audio_fifo, K_FOREVER);//Get data from FIFO Queue
		if (rx_data != NULL) {//If queue is not empty
			const struct device *uart_dev;
			uart_dev = device_get_binding(UART_AUDIO);
			if (uart_dev == NULL) {
				printk("Error: UART AUDIO -didn't find device\n"); 
				return;
			}
			playTrack(uart_dev, rx_data->txt);//Send UART command to track
			k_msleep(rx_data->timer);//Sleep for x ms time
		}
		k_free(rx_data);
		//Free memory allocated from heap.
		//https://docs.zephyrproject.org/latest/reference/kernel/memory/heap.html?highlight=k_free#c.k_free
	}
}
//Statically define and initialize a thread.
//https://docs.zephyrproject.org/latest/reference/kernel/threads/index.html?highlight=k_thread_define#c.K_THREAD_DEFINE
K_THREAD_DEFINE(audio_fx_id, STACKSIZE, audio_fx_out, NULL, NULL, NULL,
		PRIORITY, 0, 0);//Thread for send UART commends to AUDIO FX board

void uv_out(void)
{
	uint8_t cmp_data[16];
	uint8_t data[16];
	int i, ret;
	const struct device *i2c_dev;
	i2c_dev = device_get_binding("I2C_3");
	if (i2c_dev == NULL) {
		printk("Error: I2C didn't find device\n");
		return;
	}
	while (1) {
		uv = read16(i2c_dev, 0x2C);
		uv_index = uv / 100;
		k_msleep(1000);//Sleep 1000ms or 1 sec
	}
}
K_THREAD_DEFINE(uv_id, STACKSIZE / 2, uv_out, NULL, NULL, NULL, PRIORITY, 0, 0);//Thread read UV from I2C

void get_data(void)
{
	while (1) {
		uint8_t csth = (hours < 5) ? (19 + hours) : (hours - 5);//GET CST Time
		if(uv_index != prev_uv_index){// If UV index is not the same
			prev_uv_index = uv_index;
			printk("UV Index %d\n", uv_index);
			uv_pressed();//Print current UV Index
		}
		if(kph != prev_kph && kph > 2){// If current speed is > 2 kph
			prev_kph = kph;
			printk("KPH %d\n", kph);
			speed_pressed();// say current speed
		}
		if(countdown > 180){//Wait 15 min  5000 * 180 = 900000 ms
			if(kph < 2 && csth > 8 && csth < 18){//Check if its day time, if time > 8 AM and < 6 PM (we dont want to go out at night )
				struct audio_data_t tx_dataHours = { .timer = 800,
										.txt = "PTIMEWALKOGG\n" };// Audio time to go outside
				insertQueueAudio(&tx_dataHours);
				countdown = 0;// Reset 15 min timer
			}
		}
		countdown = countdown + 1;
		k_msleep(5000);//Sleep 5000ms or 5 sec
	}
}
K_THREAD_DEFINE(data_id, STACKSIZE / 2, get_data, NULL, NULL, NULL, PRIORITY, 0, 0);//Thread for timers