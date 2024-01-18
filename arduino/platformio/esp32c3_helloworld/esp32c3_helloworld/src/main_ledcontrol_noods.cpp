#include <Arduino.h>
#include <Wire.h>

// Simple sketch testing analogwrite for the noods.
// ... it works :)

#define whiteNood D0
#define blueNood D1

void setup()
{
  Serial.begin(115200);
  pinMode(whiteNood, OUTPUT);
  pinMode(blueNood, OUTPUT);
}

void loop()
{
  analogWrite(blueNood, 0);
  analogWrite(whiteNood, 32);
  delay(500);
  analogWrite(whiteNood, 127);
  delay(500);
  analogWrite(whiteNood, 255);
  delay(1000);

  analogWrite(whiteNood, 0);
  analogWrite(blueNood, 32);
  delay(500);
  analogWrite(blueNood, 127);
  delay(500);
  analogWrite(blueNood, 255);
  delay(1000);

}