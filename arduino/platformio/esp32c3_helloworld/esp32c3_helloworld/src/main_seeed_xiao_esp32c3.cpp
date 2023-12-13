#include <Arduino.h>
#include <ESP32MX1508.h>
#include <Adafruit_BusIO_Register.h>
#include "sbus.h"
#include <HardwareSerial.h>

HardwareSerial MySerial0(0);

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&MySerial0, D7, D6, true, false);

/* SBUS data */
bfs::SbusData data;

#define PINA D5
#define PINB D4
#define CH1 0 // 16 Channels (0-15) are availible
#define CH2 1 // Make sure each pin is a different channel and not in use by other PWM devices (servos, LED's, etc)

// Optional Parameters
#define RES 8     // Resolution in bits:  8 (0-255),  12 (0-4095), or 16 (0-65535)
#define FREQ 5000 // PWM Frequency in Hz

MX1508 motorA(PINA, PINB, CH1, CH2); // Default-  8 bit resoluion at 2500 Hz

long sbusPrevPacketTime;
bool sbusLost = false;

#define SBUS_VAL_MIN 176
#define SBUS_VAL_MAX 1800
#define SBUS_VAL_CENTER 988
#define SBUS_VAL_DEADBAND 5
#define SBUS_LOST_TIMEOUT 1000

void setup()
{
  Serial.begin(115200);
  // while (!Serial)
  // {
  // }

  /* Begin the SBUS communication */
  sbus_rx.Begin();
  // sbus_tx.Begin();

  // by default, let's have the program assume sbus is lost
  sbusPrevPacketTime = -SBUS_LOST_TIMEOUT;

  Serial.println("Hello World!");
}

void loop()
{
  // read SBUS
  if (sbus_rx.Read())
  {
    sbusPrevPacketTime = millis();
    sbusLost = false;

    /* Grab the received data */
    data = sbus_rx.data();
    /* Display the received data */
    for (int8_t i = 0; i < data.NUM_CH; i++)
    {
      if (Serial)
      {
        Serial.print(data.ch[i]);
        Serial.print("\t");
      }
    }
    if (Serial)
    {
      Serial.println();
    }
  }

  // if SBUS lost, reset the channels
  if (millis() - sbusPrevPacketTime > SBUS_LOST_TIMEOUT)
  {
    if (!sbusLost)
    {
      Serial.print("No SBUS packet received for 1 second >> setting first 4 channels to ");
      Serial.println((SBUS_VAL_MIN + SBUS_VAL_MAX) / 2);
      sbusLost = true;
    }
  }

  if (sbusLost)
  {
    for (int8_t i = 0; i < data.NUM_CH; i++)
    {
      data.ch[i] = SBUS_VAL_MIN;
      if (i < 4)
      {
        data.ch[i] = (SBUS_VAL_MIN + SBUS_VAL_MAX) / 2;
      }
    }
  }

  long motorVal = constrain(map(data.ch[2], SBUS_VAL_MIN, SBUS_VAL_MAX, -50, 50), -255, 255);

  if (abs(motorVal) < SBUS_VAL_DEADBAND)
  {
    motorA.motorStop(); // Soft Stop    -no argument
  }
  if (motorVal < -SBUS_VAL_DEADBAND)
  {
    motorA.motorRev(-motorVal); // Pass the speed to the motor: 0-255 for 8 bit resolution
  }
  if (motorVal > SBUS_VAL_DEADBAND)
  {
    motorA.motorGo(motorVal); // Pass the speed to the motor: 0-255 for 8 bit resolution
  }

  // delay a little.
  delay(1000 / 150);
}