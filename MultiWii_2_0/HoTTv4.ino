#include "config.h"

#if defined(HOTTV4_TELEMETRY)

#define HOTTV4_GENERAL_AIR_SENSOR 0xD0
#define HOTTV4_ELECTRICAL_AIR_SENSOR 0xE0

#if !defined (HOTTV4_TX_DELAY) 
  #define HOTTV4_TX_DELAY 600
#endif

#define ALARM_DRIVE_VOLTAGE 0x10

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
uint8_t serialAvailable() {
  #if defined (MEGA)
    return SerialAvailable(3);
  #else
    return SerialAvailable(0);
  #endif
}

/**
 * Enables RX and disables TX
 */
void enableReceiverMode() {
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
void enableTransmitterMode() {
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
void serialWrite(uint8_t data) {
  #if defined (MEGA)
    loop_until_bit_is_set(UCSR3A, UDRE3);
    UDR3 = data;
  #else
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = data;
  #endif
}

/**
 * Wait until Data Register is empty and
 * TX register is empty.
 */
void loopUntilRegistersReady() {
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
void hottV4_write(uint8_t *data, uint8_t length) {
  uint16_t crc = 0;
  
  /* Enables TX / Disables RX */
  enableTransmitterMode();
   
  for (uint8_t index = 0; index < length; index++) {  
    crc = crc + data[index]; 
    serialWrite(data[index]);
    
    delayMicroseconds(HOTTV4_TX_DELAY); 
   }
   
  uint8_t crcVal = crc & 0xFF;
  serialWrite(crcVal);

  /* Wait until Data Register and TX REgister is empty */
  loopUntilRegistersReady();
  
  /* Enables RX / Disables TX */
  enableReceiverMode();
} 

/**
 * Write out given telemetry data to serial interface. Data
 * are treated as binary sensor format and therefore only the first 45 bytes are written out.
 * CRC is ignored and calculacted on the fly.
 */
void hottV4_binary_format_write(uint8_t *data) {
  hottV4_write(data, 44);
}

/**
 * Triggers an alarm signal
 */
void hottV4_triggerAlarm(uint8_t *data, uint8_t alarm) {
  data[2] = alarm;
}

/**
 * Updates battery voltage telemetry data with given value.
 * Resolution is in 0,1V, e.g. 0x7E == 12,6V.
 * Min. value = 0V
 * Max. value = 25,6V 
 * If value is below HOTTV4_VBATLEVEL_3S, telemetry alarm is triggered
 */
void hottv4_updateBattery(uint8_t *data) {
  data[30] = vbat; 
  
  // Activate low voltage alarm and above 5.0V
  if (vbat < HOTTV4_VBATLEVEL_3S && vbat > 50) {
    hottV4_triggerAlarm(data, ALARM_DRIVE_VOLTAGE);
  }
}

/**
 * Updates the Altitude value using EstAlt (cm).
 * Result is displayed in meter.
 */
void hottv4_updateAlt(uint8_t *data) {  
  int32_t alt = ((EstAlt - referenceAltitude) / 100) + 500;
  
  data[26] = alt & 0xFF;
  data[27] = (alt >> 8) & 0xFF;
}

/**
 * Call to initialize HOTTV4
 */
void hottv4Init() {
  // Set start altitude for relative altitude calculation
  referenceAltitude = EstAlt;
  enableReceiverMode();
  
  #if defined (MEGA)
    // Enable PullUps on RX3
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

void hottV4UpdateTelemetry() {
  if ((millis() - previousMillis) > HOTTV4_UPDATE_INTERVAL) {
    previousMillis = millis();   

    // One-Wire protocoll specific "Idle line"
    // delay(5);
        
    // Check if line is quite to avoid collisions
    if (serialAvailable() == 0) {
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
      hottv4_updateBattery(telemetry_data);
    #endif
    
    #if defined(BMP085) || defined(MS561101BA) || defined (FREEIMUv043)
      hottv4_updateAlt(telemetry_data);
    #endif
    
      // Write out telemetry data as General Air Module to serial           
      hottV4_binary_format_write(telemetry_data);
    }
  }
}
#endif
