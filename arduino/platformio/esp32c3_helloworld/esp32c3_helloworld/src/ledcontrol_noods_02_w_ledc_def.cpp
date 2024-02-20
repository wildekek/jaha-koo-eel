#include <Arduino.h>
#include <Wire.h>

// Exploring using direct ledc register control (instead of analogWrite)
//

#define nood1a D1
#define nood1b D10

const uint8_t nood1a_chan = 0;
const uint8_t nood1b_chan = 1;

// setting PWM properties
const uint32_t freq = 1000;
const uint8_t resolution = 8;

// to prevent the motors from running
#define motor1a_pin D5
#define motor1b_pin D4
#define motor2a_pin D3
#define motor2b_pin D2

const uint8_t motor1a_chan = 2;
const uint8_t motor1b_chan = 3;
const uint8_t motor2a_chan = 4;
const uint8_t motor2b_chan = 5;

#define targLedcPin nood1b
#define targLedcChan nood1b_chan

void setup()
{
  Serial.begin(115200);

  delay(100);

  // not sure if this is necessary...
  ledcDetachPin(nood1a);
  ledcDetachPin(nood1b);
  ledcDetachPin(motor1a_pin);
  ledcDetachPin(motor1a_pin);
  ledcDetachPin(motor1b_pin);
  ledcDetachPin(motor2a_pin);
  ledcDetachPin(motor2b_pin);

  //*
  // to prevent the motors from running
  pinMode(motor1a_pin, OUTPUT);
  pinMode(motor1b_pin, OUTPUT);
  pinMode(motor2a_pin, OUTPUT);
  pinMode(motor2b_pin, OUTPUT);
  digitalWrite(motor1a_pin, 0);
  digitalWrite(motor1b_pin, 0);
  digitalWrite(motor2a_pin, 0);
  digitalWrite(motor2b_pin, 0);
  //*/

  /*
  ledcSetup(motor1a_chan, freq, resolution);
  ledcAttachPin(motor1a_pin, motor1a_chan);
  ledcSetup(motor1b_chan, freq, resolution);
  ledcAttachPin(motor1b_pin, motor1b_chan);
  ledcSetup(motor2a_chan, freq, resolution);
  ledcAttachPin(motor2a_pin, motor2a_chan);
  ledcSetup(motor2b_chan, freq, resolution);
  ledcAttachPin(motor2b_pin, motor2b_chan);

  ledcWrite(motor1a_chan, 127);
  ledcWrite(motor1b_chan, 127);
  ledcWrite(motor2a_chan, 127);
  ledcWrite(motor2b_chan, 127);
  //*/

  // pinMode(targLedcPin, OUTPUT);
  pinMode(nood1a, OUTPUT);
  pinMode(nood1b, OUTPUT);

  ledcSetup(nood1a_chan, freq, resolution);
  ledcAttachPin(nood1a, nood1a_chan);

  ledcSetup(nood1b_chan, freq, resolution);
  ledcAttachPin(nood1b, nood1b_chan);

  Serial.println("main_ledcontrol_noods_w_ledc_def - hello");
}

void loop()
{
  //*
  ledcWrite(nood1a_chan, 255);
  ledcWrite(nood1b_chan, 220);
  delay(200);
  ledcWrite(nood1b_chan, 127);
  delay(200);
  ledcWrite(nood1b_chan, 0);
  delay(500);

  ledcWrite(nood1b_chan, 255);
  ledcWrite(nood1a_chan, 220);
  delay(200);
  ledcWrite(nood1a_chan, 127);
  delay(200);
  ledcWrite(nood1a_chan, 0);
  delay(500);
  //*/

  /*
  ledcWrite(nood1a_chan, 220);
  delay(200);
  ledcWrite(nood1a_chan, 127);
  delay(200);
  ledcWrite(nood1a_chan, 0);
  delay(500);

  ledcWrite(nood1a_chan, 32);
  delay(200);
  ledcWrite(nood1a_chan, 127);
  delay(200);
  ledcWrite(nood1a_chan, 220);
  delay(500);
  //*/

  Serial.println(millis());
}