#include "config.h"

#if defined(HOTTV4_TELEMETRY)

#define HOTTV4_GENERAL_AIR_SENSOR 0xD0
#define HOTTV4_ELECTRICAL_AIR_SENSOR 0xE0
#define HOTTV4_TEXTMODE 0x7F

#if !defined (HOTTV4_TX_DELAY) 
  #define HOTTV4_TX_DELAY 625
#endif

#define ALARM_DRIVE_VOLTAGE 0x10
#define countof(X) ( (size_t) ( sizeof(X)/sizeof*(X) ) )

// Last time telemetry data were updated
uint32_t previousMillis = 0;

/** Stores current altitude level, so telemetry altitude
 * prints relative hight from ground.
 */
static int32_t referenceAltitude = 0;

/**
 * Wrap serial available functions for
 * MEGA boards and remaining boards.
 */
uint8_t hottV4SerialAvailable() {
  #if defined (MEGA)
    return SerialAvailable(3);
  #else
    return SerialAvailable(0);
  #endif
}

/**
 * Enables RX and disables TX
 */
void hottV4EnableReceiverMode() {
  #if defined (MEGA)  
    UCSR3B &= ~_BV(TXEN3);
    UCSR3B |= _BV(RXEN3);
  #else
    UCSR0B &= ~_BV(TXEN0);
    UCSR0B |= _BV(RXEN0);
  #endif
}

/**
 * Enabels TX and disables RX
 */
void hottV4EnableTransmitterMode() {
  #if defined (MEGA)  
    UCSR3B &= ~_BV(RXEN3);
    UCSR3B |= _BV(TXEN3);
  #else
    UCSR0B &= ~_BV(RXEN0);
    UCSR0B |= _BV(TXEN0); 
  #endif
}

/**
 * Writes out given data to data register.
 */
void hottV4SerialWrite(uint8_t data) {
  #if defined (MEGA)
    loop_until_bit_is_set(UCSR3A, UDRE3);
    UDR3 = data;
  #else
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = data;
  #endif
  
  delayMicroseconds(HOTTV4_TX_DELAY);
}

/**
 * Read from Serial interface
 */
uint8_t hottV4SerialRead() {
  #if defined (MEGA)
    return SerialRead(3);
  #else
    return SerialRead(0);
  #endif
}

/**
 * Wait until Data Register is empty and
 * TX register is empty.
 */
void hottV4LoopUntilRegistersReady() {
  delayMicroseconds(HOTTV4_TX_DELAY); 
  
  #if defined (MEGA)
    loop_until_bit_is_set(UCSR3A, UDRE3);
    loop_until_bit_is_set(UCSR3A, TXC3);
  #else
    loop_until_bit_is_set(UCSR0A, UDRE0);
    loop_until_bit_is_set(UCSR0A, TXC0);
  #endif
}

/**
 * Write out given telemetry data to serial interface.
 * Given CRC is ignored and calculated on the fly.
 */ 
void hottV4Write(uint8_t *data, uint8_t length) {
  uint16_t crc = 0;
  
  /* Enables TX / Disables RX */
  hottV4EnableTransmitterMode();
   
  for (uint8_t index = 0; index < length; index++) {  
    crc = crc + data[index]; 
    hottV4SerialWrite(data[index]);    
   }
   
  uint8_t crcVal = crc & 0xFF;
  hottV4SerialWrite(crcVal);

  /* Wait until Data Register and TX REgister is empty */
  hottV4LoopUntilRegistersReady();
  
  /* Enables RX / Disables TX */
  hottV4EnableReceiverMode();
} 

/**
 * Write out given telemetry data to serial interface. Data
 * are treated as in binary sensor format and therefore only the first 45 bytes are written out.
 * CRC is ignored and calculacted on the fly.
 */
void hottV4WriteBinaryFormat(uint8_t *data) {
  hottV4Write(data, 44);
}

/**
 * Triggers an alarm signal
 */
void hottV4TriggerAlarm(uint8_t *data, uint8_t alarm) {
  data[2] = alarm;
}

/**
 * Updates battery voltage telemetry data with given value.
 * Resolution is in 0,1V, e.g. 0x7E == 12,6V.
 * Min. value = 0V
 * Max. value = 25,6V 
 * If value is below HOTTV4_VBATLEVEL_3S, telemetry alarm is triggered
 */
void hottv4UpdateBattery(uint8_t *data) {
  // Only for investigating nasty VBAT reporting bug
  //data[30] = vbat;
  data[30] = vbat; 
  
  // Activate low voltage alarm if above 5.0V
  if (vbat < HOTTV4_VBATLEVEL_3S && vbat > 50) {
    hottV4TriggerAlarm(data, ALARM_DRIVE_VOLTAGE);
  }
}

/**
 * Updates the Altitude value using EstAlt (cm).
 * Result is displayed in meter.
 */
void hottv4UpdateAlt(uint8_t *data) {  
  int32_t alt = ((EstAlt - referenceAltitude) / 100) + 500;
  
  // Sets altitude high and low byte
  data[26] = alt & 0xFF;
  data[27] = (alt >> 8) & 0xFF;
}

/**
 * Call to initialize HOTTV4
 */
void hottv4Init() {
  // Set start altitude for relative altitude calculation
  referenceAltitude = EstAlt;
  hottV4EnableReceiverMode();
  
  #if defined (MEGA)
    /* Enable PullUps on RX3
     * without signal is to weak to be recognized
     */
    DDRJ &= ~(1 << 0);
    PORTJ |= (1 << 0);
  
    SerialOpen(3, 19200);
  #endif
}

/**
 * Initializes parameters e.g. relative high
 */
void hottv4Setup() {
  referenceAltitude = EstAlt;
}

/**
 * Main method to send telemetry data
 */
void hottV4SendTelemetry() {
  if ((millis() - previousMillis) > HOTTV4_UPDATE_INTERVAL) {
    previousMillis = millis();   

    // One-Wire protocoll specific "Idle line"
    // delay(5);
        
    // Check if line is quite to avoid collisions
    if (hottV4SerialAvailable() == 0) {
      uint8_t telemetry_data[] = { 
                  0x7C,
                  HOTTV4_ELECTRICAL_AIR_MODULE, 
                  0x00, /* Alarm */
                  HOTTV4_ELECTRICAL_AIR_SENSOR,
                  0x00, 0x00, /* Alarm Value 1 and 2 */
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Low Voltage Cell 1-7 in 2mV steps */
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* High Voltage Cell 1-7 in 2mV steps */
                  0x00, 0x00, /* Battetry 1 LSB/MSB in 100mv steps, 50 == 5V */
                  0x00, 0x00, /* Battetry 2 LSB/MSB in 100mv steps, 50 == 5V */
                  0x14, /* Temp 1, Offset of 20. 20 == 0C */ 
                  0x14, /* Temp 2, Offset of 20. 20 == 0C */
                  0xF4, 0x01, /* Height. Offset -500. 500 == 0 */
                  0x00, 0x00, /* Current LSB, MSB 1 = 0.1A */
                  0x00, 0x00, /* Drive Voltage */
                  0x00, 0x00,  /* mAh */
                  0x48, 0x00, /* m2s */ 
                  0x78, /* m3s */
                  0x00, 0x00, /* RPM. 10er steps, 300 == 3000rpm */
                  0x00, /* Electric minutes */
                  0x00, /* Electric seconds */
                  0x00, /* Speed */
                  0x00, /* Version Number */
                  0x7D, /* End sign */
                  0x00 /* Checksum */
                };
      
    #if defined(VBAT)
      hottv4UpdateBattery(telemetry_data);
    #endif
    
    #if defined(BMP085) || defined(MS561101BA) || defined (FREEIMUv043)
      hottv4UpdateAlt(telemetry_data);
    #endif
    
      // Write out telemetry data as Electric Air Module to serial           
      hottV4WriteBinaryFormat(telemetry_data);
    }
  }
}

/* #####################################################################
 *                HoTTv4 Text Mode
 * ##################################################################### */

/**
 * Sends a char
 * @param inverted Char is getting displayed inverted if > 0
 */
uint8_t hottV4SendChar(char c, uint8_t inverted) {
  // Add 128 for inverse display
  uint8_t inverse = (inverted > 0) ? 128 : 0;
  uint8_t data = c + inverse;
  
  hottV4SerialWrite(data);  
  
  return data;
}

/**
 * Sends a null terminated string
 * @param inverted Word is getting displayed inverted if > 0
 *
 * @return crc value
 */
uint16_t hottV4SendWord(char *w, uint8_t inverted) {
  uint16_t crc = 0;
  
  for (uint8_t index = 0; ; index++) {
    if (w[index] == 0x0) {
      break;
    } else { 
      crc += hottV4SendChar(w[index], inverted);  
    }
  }
  
  return crc;
}

/**
 * Sends one text line consiting of 21 digits.
 * @param text Preamble text e.g. ROLL
 * @param p P value which is displayed
 * @param i I value which is displayed
 * @param d D value which is displayed
 * @param selectedCol If > 0 text, p, i, or d column will be selected
 *
 * @return crc value
 */
uint16_t hottV4SendFormattedTextline(char *text, int8_t p, int8_t i, int8_t d, int8_t selectedCol) {
  uint16_t crc = 0;
  
  char label[8];
  char formated_P[5];
  char formated_I[6];
  char formated_D[4];

  char selectionIndicator = (selectedCol > 0) ? '>' : ' ';

  // Unfortunately no floats are supported in Arduinos snprintf http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1164927646     
  snprintf(label, 8, "%c%s", selectionIndicator, text);
  snprintf(formated_P, 5, "%2d.%01d", p / 10, p % 10);
  snprintf(formated_I, 6, "0.%03d", i);
  snprintf(formated_D, 4, "%3d", d);
  
  crc += hottV4SendWord(label, 0);
  crc += hottV4SendWord(formated_P, 2 == selectedCol);
  crc += hottV4SendChar(' ', 0);
  crc += hottV4SendWord(formated_I, 3 == selectedCol);
  crc += hottV4SendChar(' ', 0);
  crc += hottV4SendWord(formated_D, 4 == selectedCol);
  
  return crc;
}

/**
 * Sends one line of text max. 21 chars. If less than 21 chars, rest
 * is filled with whitespace chars.
 */
uint16_t hottV4SendTextline(char *line) {
  uint16_t crc = 0;
  uint8_t useZeroBytes = 0;
  
  for (uint8_t index = 0; index < 21; index++) {
    if (line[index] == 0x0 || useZeroBytes) {
      useZeroBytes = 1;
      crc += hottV4SendChar(0x20, 0);
    } else {      
      crc += hottV4SendChar(line[index], 0);
    }
  }
  
  return crc;
}

/**
 * Sends the complete 8x21 digit text block
 * @param selectedRow Which row will display the selection indicator
 * @param selectedCol Which col of the selected row will bei selected
 *
 * @return crc value
 */
uint16_t hottV4SendFormattedTextblock(int8_t selectedRow, int8_t selectedCol) {
  static char *labels[] = {"ROLL :", "PITCH:", "YAW  :", "ALT  :", "GPS  :", "LEVEL:", "MAG  :"};
  static int label2Index[] = {0, 1, 2, PIDALT, PIDGPS, PIDLEVEL, PIDMAG}; 
  
  uint16_t crc = 0;
  crc += hottV4SendTextline(" MultiWii MEETS HoTT");
  
  for (int8_t index = 0; index < countof(labels); index++) {
    int8_t col = ((index + 1) == selectedRow) ? selectedCol : 0;
    uint8_t i = label2Index[index];
    crc += hottV4SendFormattedTextline(labels[index], P8[i], I8[i], D8[i], col);
  }
   
  return crc; 
}

/**
 * Send Header for Text Mode
 */
uint16_t hottV4SendHeader() {
  uint16_t crc = 0;
  
  hottV4SerialWrite(0x7B);
  crc += 0x7B;
  
  hottV4SerialWrite(0xE0);
  crc += 0xE0;
  
  uint8_t alarm = 0x00;
  hottV4SerialWrite(alarm);
  crc += alarm;
  
  return crc;
}

/**
 * Sends the complete text mode frame
 */
void hottV4SendText(int8_t selectedRow, int8_t selectedCol) {
  hottV4EnableTransmitterMode();
  uint16_t crc = 0;

  crc += hottV4SendHeader();
  crc += hottV4SendFormattedTextblock(selectedRow, selectedCol);
  
  hottV4SerialWrite(0x7D);
  crc += (0x7D);
  
  uint8_t checksum = crc & 0xFF;
  hottV4SerialWrite(checksum);
  
  hottV4LoopUntilRegistersReady();
  hottV4EnableReceiverMode();
}

/**
 * Main method to send PID settings
 */
void hottV4SendSettings() {
  // Saftey measure because it takes way to long to send data in text mode
  // furthermore PID settings will be editable in future and this is something you 
  // dont wanna do up in the air.
  if (!armed) {
    static int8_t row = 1;
    static int8_t col = 1;
  
    while(hottV4SerialAvailable() <= 1) {}
  
    uint8_t data = hottV4SerialRead();
    delay(5);
      
    if (data == 0xEF) {
      hottV4SendText(row, col);
    } else if (data == 0xEB && row > 1) {
      hottV4SendText(--row, col);
    } else if (data == 0xED && row < 7) {
      hottV4SendText(++row, col);
    } else if (data == 0xE9) {
      col = (col < 4) ? col+1 : 1;
    }
  }
}

#endif
