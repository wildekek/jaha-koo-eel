#include <Arduino.h>
#include <Wire.h>

// Exploring using direct ledc register control (instead of analogWrite)
//

#define nood1a D10
#define nood1b D1
#define nood2a D8
#define nood2b D0

const uint8_t nood1a_chan = 0;
const uint8_t nood1b_chan = 1;
const uint8_t nood2a_chan = 2;
const uint8_t nood2b_chan = 3;

// setting PWM properties
const uint32_t freq = 2500;
const uint8_t resolution = 8;

// to prevent the motors from running
#define motor1a_pin D5
#define motor1b_pin D4
#define motor2a_pin D3
#define motor2b_pin D2

const uint8_t motor1_chan = 4;
const uint8_t motor2_chan = 5;

void setup()
{
  Serial.begin(115200);

  delay(100);

  // not sure if this is necessary...
  ledcDetachPin(nood1a);
  ledcDetachPin(nood1b);
  ledcDetachPin(nood2a);
  ledcDetachPin(nood2b);
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

  pinMode(nood1a, OUTPUT);
  pinMode(nood1b, OUTPUT);
  pinMode(nood2a, OUTPUT);
  pinMode(nood2b, OUTPUT);

  ledcSetup(nood1a_chan, freq, resolution);
  ledcAttachPin(nood1a, nood1a_chan);

  ledcSetup(nood1b_chan, freq, resolution);
  ledcAttachPin(nood1b, nood1b_chan);

  ledcSetup(nood2a_chan, freq, resolution);
  ledcAttachPin(nood2a, nood2a_chan);

  ledcSetup(nood2b_chan, freq, resolution);
  ledcAttachPin(nood2b, nood2b_chan);

  Serial.println("ledcontrol_noods_02_w_ledc_def - hello");
}

uint16_t delayTime = 2500;
void loop()
{
  // nood1a on
  Serial.println("nood1a on");
  ledcWrite(nood1a_chan, 0);
  ledcWrite(nood1b_chan, 255);
  ledcWrite(nood2a_chan, 255);
  ledcWrite(nood2b_chan, 255);

  delay(delayTime);

  // nood1b on
  Serial.println("nood1b on");
  ledcWrite(nood1a_chan, 255);
  ledcWrite(nood1b_chan, 0);
  ledcWrite(nood2a_chan, 255);
  ledcWrite(nood2b_chan, 255);

  delay(delayTime);

  // nood2a on
  Serial.println("nood2a on");
  ledcWrite(nood1a_chan, 255);
  ledcWrite(nood1b_chan, 255);
  ledcWrite(nood2a_chan, 0);
  ledcWrite(nood2b_chan, 255);

  delay(delayTime);

  // nood2b on
  Serial.println("nood2b on");
  ledcWrite(nood1a_chan, 255);
  ledcWrite(nood1b_chan, 255);
  ledcWrite(nood2a_chan, 255);
  ledcWrite(nood2b_chan, 0);

  delay(delayTime);

  // nood1a+b on
  Serial.println("nood1a+b on");
  ledcWrite(nood1a_chan, 0);
  ledcWrite(nood1b_chan, 0);
  ledcWrite(nood2a_chan, 255);
  ledcWrite(nood2b_chan, 255);

  delay(delayTime);

  // nood2a+b on
  Serial.println("nood2a+b on");
  ledcWrite(nood1a_chan, 255);
  ledcWrite(nood1b_chan, 255);
  ledcWrite(nood2a_chan, 0);
  ledcWrite(nood2b_chan, 0);

  delay(delayTime);

  // noods all on
  Serial.println("noods on");
  ledcWrite(nood1a_chan, 0);
  ledcWrite(nood1b_chan, 0);
  ledcWrite(nood2a_chan, 0);
  ledcWrite(nood2b_chan, 0);

  delay(delayTime);

  // all off
  Serial.println("noods off");
  ledcWrite(nood1a_chan, 255);
  ledcWrite(nood1b_chan, 255);
  ledcWrite(nood2a_chan, 255);
  ledcWrite(nood2b_chan, 255);

  delay(delayTime);

}