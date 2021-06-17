// https://github.com/adafruit/Adafruit_GPS
#ifndef _ADAFRUIT_GPS_H
#define _ADAFRUIT_GPS_H

#include <inttypes.h>
#include <device.h>
#include <drivers/uart.h>/*UART*/

/* Parameters */
#define UART_BAUDRATE 9600
#define UART_LABEL "UART_1"
#define COMMAND_SIZE 2
#define TRACK_SIZE 13



typedef enum {
  NMEA_BAD = 0, ///< passed none of the checks
  NMEA_HAS_DOLLAR =
      1, ///< has a dollar sign or exclamation mark in the first position
  NMEA_HAS_CHECKSUM = 2,   ///< has a valid checksum at the end
  NMEA_HAS_NAME = 4,       ///< there is a token after the $ followed by a comma
  NMEA_HAS_SOURCE = 10,    ///< has a recognized source ID
  NMEA_HAS_SENTENCE = 20,  ///< has a recognized sentence ID
  NMEA_HAS_SENTENCE_P = 40 ///< has a recognized parseable sentence ID
} nmea_check_t;

class Adafruit_GPS : public Print {
public:
  // Adafruit_GPS.cpp
  bool begin(uint32_t baud_or_i2caddr);



  int thisCheck = 0; ///< the results of the check on the current sentence
  char thisSource[NMEA_MAX_SOURCE_ID] = {
      0}; ///< the first two letters of the current sentence, e.g. WI, GP
  char thisSentence[NMEA_MAX_SENTENCE_ID] = {
      0}; ///< the next three letters of the current sentence, e.g. GLL, RMC
  char lastSource[NMEA_MAX_SOURCE_ID] = {
      0}; ///< the results of the check on the most recent successfully parsed
          ///< sentence
  char lastSentence[NMEA_MAX_SENTENCE_ID] = {
      0}; ///< the next three letters of the most recent successfully parsed
          ///< sentence, e.g. GLL, RMC

  uint8_t hour;          ///< GMT hours
  uint8_t minute;        ///< GMT minutes
  uint8_t seconds;       ///< GMT seconds
  uint16_t milliseconds; ///< GMT milliseconds
  uint8_t year;          ///< GMT year
  uint8_t month;         ///< GMT month
  uint8_t day;           ///< GMT day

  nmea_float_t latitude;  ///< Floating point latitude value in degrees/minutes
                          ///< as received from the GPS (DDMM.MMMM)
  nmea_float_t longitude; ///< Floating point longitude value in degrees/minutes
                          ///< as received from the GPS (DDDMM.MMMM)

  /** Fixed point latitude and longitude value with degrees stored in units of
    1/10000000 of a degree. See pull #13 for more details:
    https://github.com/adafruit/Adafruit-GPS-Library/pull/13 */
  int32_t latitude_fixed;  ///< Fixed point latitude in decimal degrees.
                           ///< Divide by 10000000.0 to get a double.
  int32_t longitude_fixed; ///< Fixed point longitude in decimal degrees
                           ///< Divide by 10000000.0 to get a double.

  nmea_float_t latitudeDegrees;  ///< Latitude in decimal degrees
  nmea_float_t longitudeDegrees; ///< Longitude in decimal degrees
  nmea_float_t geoidheight;      ///< Diff between geoid height and WGS84 height
  nmea_float_t altitude;         ///< Altitude in meters above MSL
  nmea_float_t speed;            ///< Current speed over ground in knots
  nmea_float_t angle;            ///< Course in degrees from true north
  nmea_float_t magvariation; ///< Magnetic variation in degrees (vs. true north)
  nmea_float_t HDOP; ///< Horizontal Dilution of Precision - relative accuracy
                     ///< of horizontal position
  nmea_float_t VDOP; ///< Vertical Dilution of Precision - relative accuracy
                     ///< of vertical position
  nmea_float_t PDOP; ///< Position Dilution of Precision - Complex maths derives
                     ///< a simple, single number for each kind of DOP
  char lat = 'X';    ///< N/S
  char lon = 'X';    ///< E/W
  char mag = 'X';    ///< Magnetic variation direction
  bool fix;          ///< Have a fix?
  uint8_t fixquality;    ///< Fix quality (0, 1, 2 = Invalid, GPS, DGPS)
  uint8_t fixquality_3d; ///< 3D fix quality (1, 3, 3 = Nofix, 2D fix, 3D fix)
  uint8_t satellites;    ///< Number of satellites in use

  uint16_t LOCUS_serial;  ///< Log serial number
  uint16_t LOCUS_records; ///< Log number of data record
  uint8_t LOCUS_type;     ///< Log type, 0: Overlap, 1: FullStop
  uint8_t LOCUS_mode;     ///< Logging mode, 0x08 interval logger
  uint8_t LOCUS_config;   ///< Contents of configuration
  uint8_t LOCUS_interval; ///< Interval setting
  uint8_t LOCUS_distance; ///< Distance setting
  uint8_t LOCUS_speed;    ///< Speed setting
  uint8_t LOCUS_status;   ///< 0: Logging, 1: Stop logging
  uint8_t LOCUS_percent;  ///< Log life used percentage

}

void sendCommand(const struct device *uart_dev, uint8_t *cmd);
void playTrack(const struct device *uart_dev, char *track);
char read(void);

#endif /* _ADAFRUIT_GPS_H */