#include <Arduino.h>
#include <ESP32MX1508.h>
#include <Adafruit_BusIO_Register.h>

#define PINA D5
#define PINB D4

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello World!");

  pinMode(PINA, OUTPUT);
  pinMode(PINB, OUTPUT);
}

void loop()
{
  Serial.println("pinA HIGH, pinB LOW");
  digitalWrite(PINA, HIGH);
  digitalWrite(PINB, LOW);
  delay(1000);
  Serial.println("pinA LOW, pinB HIGH");
  digitalWrite(PINA, LOW);
  digitalWrite(PINB, HIGH);
  delay(500);
}
