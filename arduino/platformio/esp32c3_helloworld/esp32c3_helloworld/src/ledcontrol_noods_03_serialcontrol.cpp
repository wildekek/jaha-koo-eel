#include <Arduino.h>
#include <Wire.h>

// Exploring using direct ledc register control (instead of analogWrite)
//

#define nood1a D8
#define nood1b D1
#define nood2a D10
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

void allOn();
void allOff();
void noodOn(uint8_t chan, uint8_t value);
void checkIncomingSerial();

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

  allOn();

  Serial.println("ledcontrol_noods_03_serialcontrol - hello");
  Serial.println("q / w = all on / off");
  Serial.println("a / s = select pink / blue noods");
  Serial.println("1/2/3/4 = select individual noods");
}

uint16_t delayTime = 2500;
void loop()
{
  checkIncomingSerial();

  delay(1000 / 200);
}

void allOff()
{
  ledcWrite(nood1a_chan, 255);
  ledcWrite(nood1b_chan, 255);
  ledcWrite(nood2a_chan, 255);
  ledcWrite(nood2b_chan, 255);
}
void allOn()
{
  ledcWrite(nood1a_chan, 0);
  ledcWrite(nood1b_chan, 0);
  ledcWrite(nood2a_chan, 0);
  ledcWrite(nood2b_chan, 0);
}

void noodOn(uint8_t chan, uint8_t value)
{
  ledcWrite(chan, value);
}

void checkIncomingSerial()
{
  if (Serial.available() > 0)
  {
    char inChar = Serial.read();
    switch (inChar)
    {
    case 'q':
      Serial.println("all on");
      allOn();
      break;

    case 'w':
      Serial.println("all off");
      allOff();
      break;

    case 'a':
      Serial.println("nood 1 on (pink)");
      allOff();
      noodOn(nood1a_chan, 0);
      noodOn(nood1b_chan, 0);
      break;

    case 's':
      Serial.println("nood 2 on (blue)");
      allOff();
      noodOn(nood2a_chan, 0);
      noodOn(nood2b_chan, 0);
      break;

    case '1':
      Serial.println("nood 1a on (pink front)");
      allOff();
      noodOn(nood1a_chan, 0);
      break;
    case '2':
      Serial.println("nood 1b on (pink rear)");
      allOff();
      noodOn(nood1b_chan, 0);
      break;
    case '3':
      Serial.println("nood 2a on (blue front)");
      allOff();
      noodOn(nood2a_chan, 0);
      break;
    case '4':
      Serial.println("nood 2b on (blue rear)");
      allOff();
      noodOn(nood2b_chan, 0);
      break;
    }

    while (Serial.available() > 0)
    {
      Serial.read();
    }
  }
}
