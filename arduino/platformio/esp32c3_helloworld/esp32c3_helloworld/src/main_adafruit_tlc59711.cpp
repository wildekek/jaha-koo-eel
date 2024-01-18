#include <Arduino.h>
#include <ESP32MX1508.h>
#include <Adafruit_BusIO_Register.h>

#include "Adafruit_TLC59711.h"
#include <SPI.h>

void colorWipe(uint16_t r, uint16_t g, uint16_t b, uint8_t wait);
void increaseBrightness();

// How many boards do you have chained?
#define NUM_TLC59711 1

#define data D10
#define clock D8

Adafruit_TLC59711 tlc = Adafruit_TLC59711(NUM_TLC59711, clock, data);
// Adafruit_TLC59711 tlc = Adafruit_TLC59711(NUM_TLC59711);

void setup()
{
  Serial.begin(115200);

  Serial.println("TLC59711 test");

  tlc.begin();
  tlc.write();
}

void loop()
{
  colorWipe(65535, 0, 0, 100); // "Red" (depending on your LED wiring)
  delay(200);
  colorWipe(0, 65535, 0, 100); // "Green" (depending on your LED wiring)
  delay(200);
  colorWipe(0, 0, 65535, 100); // "Blue" (depending on your LED wiring)
  delay(200);

  increaseBrightness();
}

// Fill the dots one after the other with a color
void colorWipe(uint16_t r, uint16_t g, uint16_t b, uint8_t wait)
{
  for (uint16_t i = 0; i < 8 * NUM_TLC59711; i++)
  {
    tlc.setLED(i, r, g, b);
    tlc.write();
    delay(wait);
  }
}

// All RGB Channels on full colour
// Cycles trough all brightness settings from 0 up to 127
void increaseBrightness()
{
  for (uint16_t i = 0; i < 8 * NUM_TLC59711; i++)
  {
    tlc.setLED(i, 65535, 65535, 65535);
  }
  for (int i = 0; i < 128; i++)
  {
    tlc.simpleSetBrightness(i);
    tlc.write();
    delay(100);
  }
}