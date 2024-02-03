#include <Arduino.h>

#define analogInPin 3
#define analogOut_1_Pin 5
#define analogOut_2_Pin 7
#define analogOut_3_Pin 9
#define analogOut_4_Pin 11

void setup()
{
  Serial.begin(115200);

  pinMode(analogInPin, INPUT);
  pinMode(analogOut_1_Pin, OUTPUT);
  pinMode(analogOut_2_Pin, OUTPUT);
  pinMode(analogOut_3_Pin, OUTPUT);
  pinMode(analogOut_4_Pin, OUTPUT);

  Serial.println(".... ready for action");
}

float sinIncrement = 0.001;
float sinVal = 0.0;
void loop()
{
  sinIncrement = map(analogRead(analogInPin), 0, 8192, 7000, 12000) / 100000.0;
  // sinIncrement = .1; // 0.09

  sinVal += sinIncrement;

  float phase_offset = map(analogRead(analogInPin), 0, 8192, 0, 900) / 1000.0;;
  // phase_offset = 0.15;

  float nood1Sinval = sin(sinVal);
  float nood2Sinval = sin(sinVal + ((1 * phase_offset) * TWO_PI));
  float nood3Sinval = sin(sinVal + ((2 * phase_offset) * TWO_PI));
  float nood4Sinval = sin(sinVal + ((3 * phase_offset) * TWO_PI));

  byte nood1_pwmVal = (byte)(nood1Sinval * 127.0 + 127.0);
  byte nood2_pwmVal = (byte)(nood2Sinval * 127.0 + 127.0);
  byte nood3_pwmVal = (byte)(nood3Sinval * 127.0 + 127.0);
  byte nood4_pwmVal = (byte)(nood4Sinval * 127.0 + 127.0);

  analogWrite(analogOut_1_Pin, nood1_pwmVal);
  analogWrite(analogOut_2_Pin, nood2_pwmVal);
  analogWrite(analogOut_3_Pin, nood3_pwmVal);
  analogWrite(analogOut_4_Pin, nood4_pwmVal);

  delay(1000 / 60);
}