#include "config.h"

#if defined(HOTTV4_TELEMETRY)

#define HOTTV4_GENERAL_AIR_SENSOR 0xD0
#define HOTTV4_ELECTRICAL_AIR_SENSOR 0xE0

#define NO 0
#define YES 1

#if !defined (HOTTV4_TX_DELAY) 
  #define HOTTV4_TX_DELAY 625
#endif

#define ALARM_DRIVE_VOLTAGE 0x10

// Last time telemetry data were updated
static uint32_t previousMillis = 0;

/** Stores current altitude level, so telemetry altitude
 * prints relative hight from ground.
 */
static int32_t referenceAltitude = 0;

/**
 * Wrap serial available functions for
 * MEGA boards and remaining boards.
 */
static uint8_t hottV4SerialAvailable() {
  #if defined (MEGA)
    return SerialAvailable(3);
  #else
    return SerialAvailable(0);
  #endif
}

/**
 * Enables RX and disables TX
 */
static void hottV4EnableReceiverMode() {
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
static void hottV4EnableTransmitterMode() {
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
static void hottV4SerialWrite(uint8_t data) {
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
static uint8_t hottV4SerialRead() {
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
static void hottV4LoopUntilRegistersReady() {
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
static void hottV4SendBinary(uint8_t *data, uint8_t length) {
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
 * Triggers an alarm signal
 */
static void hottV4TriggerAlarm(uint8_t *data, uint8_t alarm) {
  data[2] = alarm;
}

/**
 * Updates battery voltage telemetry data with given value.
 * Resolution is in 0,1V, e.g. 0x7E == 12,6V.
 * Min. value = 0V
 * Max. value = 25,6V 
 * If value is below HOTTV4_VBATLEVEL_3S, telemetry alarm is triggered
 */
static void hottv4UpdateBattery(uint8_t *data) {
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
static void hottv4UpdateAlt(uint8_t *data) {  
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
      hottV4SendBinary(telemetry_data, 44);
    }
  }
}

/* ##################################################################### *
 *                HoTTv4 Text Mode                                       *
 * ##################################################################### */

// Number of Rows without Header that can be displayed
#define ROWS 7

// Defines which controller values can be changed for ROLL, PITCH, YAW, etc.
typedef enum {
  HoTTv4ControllerValuesP = 1 << 0,
  HoTTv4ControllerValuesPID = 1 << 1,
} HoTTv4ControllerValues;

// Defines structure of Preamble text, the corresponding PID index, which PID values can be changed.
typedef struct {
  char *label;
  uint8_t pidIndex;
  HoTTv4ControllerValues controllerValue;
} HoTTv4TextModeData;

/**
 * All 7 lines that are displayed
 */
static HoTTv4TextModeData settings[] = { {"ROLL :", 0, HoTTv4ControllerValuesPID}, 
                                         {"PITCH:", 1, HoTTv4ControllerValuesPID},
                                         {"YAW  :", 2, HoTTv4ControllerValuesPID},
                                         {"ALT  :", PIDALT, HoTTv4ControllerValuesPID},
                                         {"GPS  :", PIDGPS, HoTTv4ControllerValuesPID},
                                         {"LEVEL:", PIDLEVEL, HoTTv4ControllerValuesPID},
                                         {"MAG  :", PIDMAG, HoTTv4ControllerValuesP} }; 
 
 /**
 * Sends a char
 * @param inverted Char is getting displayed inverted if > 0
 */
static uint8_t hottV4SendChar(char c, uint8_t inverted) {
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
static uint16_t hottV4SendWord(char *w, uint8_t inverted) {
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
 * Sends the given P Value as a formatted text element, e.g. 4.5
 * @return crc value
 */
static uint16_t hottV4SendFormattedPValue(int8_t p, int8_t inverted) {
  char formatted_P[5];
  // Unfortunately no floats are supported in Arduinos snprintf http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1164927646     
  snprintf(formatted_P, 5, "%2d.%01d", p / 10, p % 10);
  
  return hottV4SendWord(formatted_P, inverted);
}

/**
 * Sends the given I Value as a formatted text element, e.g. 0.012
 * @return crc value
 */
static uint16_t hottV4SendFormattedIValue(int8_t i, int8_t inverted) {
  char formatted_I[6];
  snprintf(formatted_I, 6, "0.%03d", i);
  
  return hottV4SendWord(formatted_I, inverted);
}

/**
 * Sends the given D Value as a formatted text element, e.g. 23
 * @return crc value
 */
static uint16_t hottV4SendFormattedDValue(int8_t d, int8_t inverted) {
  char formatted_D[4];
  snprintf(formatted_D, 4, "%3d", d);
  
  return hottV4SendWord(formatted_D, inverted);
}

/**
 * Sends one text line consiting of 21 digits.
 * @param textData Cotains all data needed to display PID values
 * @param selectedCol If > 0 text, p, i, or d column will be selected
 *
 * @return crc value
 */
static uint16_t hottV4SendFormattedTextline(void* data, int8_t selectedCol) {
  uint16_t crc = 0;
  
  // void * to fix this stupid arduino prototyping f***  http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1195036223
  HoTTv4TextModeData *textData = (HoTTv4TextModeData *)data;
  uint8_t index = textData->pidIndex;
  
  char label[8];
  char selectionIndicator = (selectedCol > 0) ? '>' : ' ';

  snprintf(label, 8, "%c%s", selectionIndicator, textData->label);  
  crc += hottV4SendWord(label, 0);
  
  if (textData->controllerValue & (HoTTv4ControllerValuesP | HoTTv4ControllerValuesPID)) {
      crc += hottV4SendFormattedPValue(P8[index], 2 == selectedCol);
  } else {
    crc += hottV4SendWord(" -- ", 0);
  }
 
  crc += hottV4SendChar(' ', 0);
  
  if (textData->controllerValue & (HoTTv4ControllerValuesPID)) {
    crc += hottV4SendFormattedIValue(I8[index], 3 == selectedCol);
  } else {
    crc += hottV4SendWord(" --- ", 0);
  }
  
  crc += hottV4SendChar(' ', 0);
  
  if (textData->controllerValue & (HoTTv4ControllerValuesPID)) {
    crc += hottV4SendFormattedDValue(D8[index], 4 == selectedCol);
  } else {
    crc += hottV4SendWord(" - ", 0);
  }
  
  return crc;
}

/**
 * Sends one line of text max. 21 chars. If less than 21 chars, rest
 * is filled with whitespace chars.
 */
static uint16_t hottV4SendTextline(char *line) {
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
static uint16_t hottV4SendFormattedTextblock(int8_t selectedRow, int8_t selectedCol) {
  uint16_t crc = 0;
  crc += hottV4SendTextline(" MultiWii MEETS HoTT");
  
  for (int8_t index = 0; index < ROWS; index++) {
    int8_t col = ((index + 1) == selectedRow) ? selectedCol : 0;    
    crc += hottV4SendFormattedTextline(&settings[index], col);
  }
   
  return crc; 
}

/**
 * Send Header for Text Mode
 */
static uint16_t hottV4SendTextModeHeader() {
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
static void hottV4SendText(int8_t selectedRow, int8_t selectedCol) {
  hottV4EnableTransmitterMode();
  uint16_t crc = 0;

  crc += hottV4SendTextModeHeader();
  crc += hottV4SendFormattedTextblock(selectedRow, selectedCol);
  
  hottV4SerialWrite(0x7D);
  crc += (0x7D);
  
  uint8_t checksum = crc & 0xFF;
  hottV4SerialWrite(checksum);
  
  hottV4LoopUntilRegistersReady();
  hottV4EnableReceiverMode();
}

/**
 * Updates the system wide PID settings.
 * @param row Which row is currently selected in the menu --> ROLL, PITCH, YAW, etc.
 * @param col Which column is currently selected -> P, I, D
 * @parma val Adds val to currently selected PID value
 */
static void hottV4UpdatePIDValueBy(int8_t row, int8_t col, int8_t val) {
  int8_t pidIndex = settings[row - 1].pidIndex;
  
  switch (col) {
    case 2:
      P8[pidIndex] += val;
      P8[pidIndex] = constrain(P8[pidIndex], 0, 200);
      break;
    case 3:
      I8[pidIndex] += val;
      I8[pidIndex] = constrain(I8[pidIndex], 0, 250);
      break;
    case 4:
      D8[pidIndex] += val;
      D8[pidIndex] = constrain(D8[pidIndex], 0, 100);
      break;
  } 
}

/**
 * Determines if current selected col can be edited.
 * Returns 1 if given col can be edited, everything else otherwise.
 */
static uint8_t isInEditingMode(int8_t col) {
  return col > 1;
}

/**
 * Determines if given row can be selected as next line.
 * Returns 1 if next line can be selected, everything else otherwise.
 */
static uint8_t canSelectNextLine(int8_t row) {
  return (row > 0) && (row < 8);
}

/**
 * Determines next valid col number in respect of current selected
 * col and Controller value. 
 *
 * @return Number of next valid col.
 */
static uint8_t nextCol(int8_t currentCol, int8_t controllerValue) {
  HoTTv4ControllerValues val = (HoTTv4ControllerValues)controllerValue;
  
  switch(val) {
    case HoTTv4ControllerValuesP:
      return (currentCol < 2) ? currentCol+1 : 1;
      break;
    case HoTTv4ControllerValuesPID:
      return  (currentCol < 4) ? currentCol+1 : 1;
      break;
    default:
      return 1;
  }
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
    static uint8_t dirty = 0;
    
    delay(5);
    uint8_t data = hottV4SerialRead();
    
    switch (data) {
      case 0xEB:
        if (NO == isInEditingMode(col)) {
          if (canSelectNextLine(row-1)) {
            row--;
          }
        } else {
          hottV4UpdatePIDValueBy(row, col, -1);
          dirty = 1;
        }
      break;
      case 0xED:
        if (NO == isInEditingMode(col)) {
          if (canSelectNextLine(row+1)) {
            row++; 
          }
        } else {
          hottV4UpdatePIDValueBy(row, col, 1);
          dirty = 1;
        }
      break;
      case 0xE9:
        HoTTv4ControllerValues controllerValue = settings[row].controllerValue;
        col = nextCol(col, controllerValue);
      
        if (dirty > 0) {
          dirty = 0;
          writeParams();
        }
      break;
    }
      
    hottV4SendText(row, col);
  }
}
#endif
