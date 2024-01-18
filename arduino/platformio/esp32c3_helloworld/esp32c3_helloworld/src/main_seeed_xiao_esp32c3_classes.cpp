#include <Arduino.h>
#include <ESP32MX1508.h>
#include <Adafruit_BusIO_Register.h>
#include "sbus.h"
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <FastLED.h>

// variables
#include "vars/motorcontroller.h"
#include "vars/neopixels.h"
#include "vars/sbusrx.h"


// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&MySerial0, D7, D6, true, false);

MX1508 motorA(PIN_MOTOR1_A, PIN_MOTOR1_B, CH_MOTOR1_1, CH_MOTOR1_2, RES, 2500); // Default-  8 bit resoluion at 2500 Hz
MX1508 motorB(PIN_MOTOR2_A, PIN_MOTOR2_B, CH_MOTOR2_1, CH_MOTOR2_2, RES, 2500); // Default-  8 bit resoluion at 2500 Hz

float sinCounter = 0.0;
float sinCounterIncrement = 0.05;

bool doSineMovement = false;

byte hVal = 0;
byte vVal = 0;

void setup()
{
  Serial.begin(115200);

  /* Begin the SBUS communication */
  sbus_rx.Begin();
  // sbus_tx.Begin();

  // by default, let's have the program assume sbus is lost
  sbusPrevPacketTime = -SBUS_LOST_TIMEOUT;

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

  Serial.println("Hello World!");
}

void loop()
{
  // pixels.clear(); // Set all pixel colors to 'off'

  // read SBUS
  if (sbus_rx.Read())
  {
    sbusPrevPacketTime = millis();
    if (sbusLost)
    {
      Serial.println("Regained SBUS connection");
      sbusLost = false;
    }

    /* Grab the received data */
    data = sbus_rx.data();

    /* Display the received data */
    if (millis() - sbusPacketPrintPrevTime > SBUS_PACKET_PRINT_INTERVAL)
    {
      if (Serial)
      {
        for (int8_t i = 0; i < data.NUM_CH; i++)
        {
          Serial.print(data.ch[i]);
          Serial.print("\t");
        }
        Serial.println();
      }

      sbusPacketPrintPrevTime = millis();
    }
  }

  // if SBUS lost, reset the channels
  if (millis() - sbusPrevPacketTime > SBUS_LOST_TIMEOUT)
  {
    if (!sbusLost)
    {
      Serial.print("Lost SBUS connection >> setting first 4 channels to ");
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

  byte motorPowerRange = 50;
  if (data.ch[TX_AUX2] > SBUS_SWITCH_MAX_THRESHOLD) {
    motorPowerRange = 50;
  } else if (data.ch[TX_AUX2] > SBUS_SWITCH_MIN_THRESHOLD) {
    motorPowerRange = 35;
  } else {
    motorPowerRange = 20;
  }

  int16_t motor1Val = constrain(map(data.ch[TX_ROLL], SBUS_VAL_MIN, SBUS_VAL_MAX, -motorPowerRange, motorPowerRange), -255, 255);
  int16_t motor2Val = constrain(map(data.ch[TX_PITCH], SBUS_VAL_MIN, SBUS_VAL_MAX, -motorPowerRange, motorPowerRange), -255, 255);

  // hVal = constrain(map(data.ch[TX_YAW], SBUS_VAL_MIN, SBUS_VAL_MAX, 0, 250), 0, 250);
  hVal = constrain(map(data.ch[TX_YAW], SBUS_VAL_MIN, SBUS_VAL_MAX, 0, 250), 0, 250);
  vVal = constrain(map(data.ch[TX_THROTTLE], SBUS_VAL_MIN, SBUS_VAL_MAX, 0, 225), 0, 225);

  //*
  // sets variable hVal according to position of tx_aux1
  if (data.ch[TX_AUX1] < SBUS_SWITCH_MIN_THRESHOLD)
  {
    // Serial.println("lock in hue val");
    hVal = 32;
    // hVal = 32;
  }
  //*/

  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(hVal, 0, 25 + vVal));
  }
  pixels.show(); // Send the updated pixel colors to the hardware.
  // for (int i = 0; i < NUM_LEDS; i++)
  // {
  //   leds[i] = CHSV(hVal, 240, 25 + vVal);
  // }
  // FastLED.show();

  doSineMovement = (data.ch[TX_AUX4] < SBUS_SWITCH_MIN_THRESHOLD) ? true : false;

  int16_t throttleVal = motor2Val;
  float mix = constrain(map(data.ch[TX_ROLL], SBUS_VAL_MIN, SBUS_VAL_MAX, 0, 1000), 250, 750) / 1000.0; // gives a range of .25-.75
  motor1Val = (throttleVal * (1.0 - mix)) * 2;
  motor2Val = (throttleVal * mix) * 2;

  sinCounterIncrement = map(data.ch[TX_THROTTLE], SBUS_VAL_MIN, SBUS_VAL_MAX, 200, 1000) / 5000.0;
  float sinMult1 = (sin(sinCounter) + 1.0);
  float sinMult2 = (sin(sinCounter + HALF_PI) + 1.0);
  sinCounter += sinCounterIncrement;
  // Serial.println("sinCounterIncrement: " + String(sinCounterIncrement));

  if (doSineMovement)
  {
    motor1Val *= (sinMult1 *.75) + .25;
    motor2Val *= (sinMult2 *.75) + .25;
  }

  motor1Val = constrain(motor1Val, -255, 255);
  motor2Val = constrain(motor2Val, -255, 255);

  // MOTOR 1
  if (abs(motor1Val) < SBUS_VAL_DEADBAND)
  {
    motorA.motorStop(); // Soft Stop    -no argument
  }
  if (motor1Val < -SBUS_VAL_DEADBAND)
  {
    motorA.motorRev(-motor1Val); // Pass the speed to the motor: 0-255 for 8 bit resolution
  }
  if (motor1Val > SBUS_VAL_DEADBAND)
  {
    motorA.motorGo(motor1Val); // Pass the speed to the motor: 0-255 for 8 bit resolution
  }

  // MOTOR 2
  if (abs(motor2Val) < SBUS_VAL_DEADBAND)
  {
    motorB.motorStop(); // Soft Stop    -no argument
  }
  if (motor2Val < -SBUS_VAL_DEADBAND)
  {
    motorB.motorRev(-motor2Val); // Pass the speed to the motor: 0-255 for 8 bit resolution
  }
  if (motor2Val > SBUS_VAL_DEADBAND)
  {
    motorB.motorGo(motor2Val); // Pass the speed to the motor: 0-255 for 8 bit resolution
  }

  // delay a little.
  delay(1000 / 200);
}