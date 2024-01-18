#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <FastLED.h>

#define PIN 17      // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 2 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void BlinkLed(byte num) // Basic blink function
{
  for (byte i = 0; i < num; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello World");

  BlinkLed(2);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

byte cycle = 0;
void loop()
{
  pixels.clear(); // Set all pixel colors to 'off'

  //*
  uint32_t h = 0;
  switch (cycle)
  {
  case 0:
    h = pixels.ColorHSV(0, 255, 255);
    pixels.fill(h);
    for (int i = 0; i < NUMPIXELS; i++)
    {
      // pixels.setPixelColor(i, pixels.Color(150, 0, 0));
    }
    break;
  case 1:
    h = pixels.ColorHSV(85 << 8, 255, 255);
    pixels.fill(h);
    for (int i = 0; i < NUMPIXELS; i++)
    {
      // pixels.setPixelColor(i, pixels.Color(0, 150, 0));
    }
    break;
  case 2:
    h = pixels.ColorHSV(155 << 8, 255, 255);
    pixels.fill(h);
    for (int i = 0; i < NUMPIXELS; i++)
    {
      // pixels.setPixelColor(i, pixels.Color(0, 0, 150));
    }
    break;
  case 3:
    h = pixels.ColorHSV(200 << 8, 255, 255);
    pixels.fill(h);
    for (int i = 0; i < NUMPIXELS; i++)
    {
      // pixels.setPixelColor(i, pixels.Color(150, 150, 0));
    }
    break;
  }
  pixels.show(); // Send the updated pixel colors to the hardware.
  //*/

  delay(1000 / 200);

  EVERY_N_SECONDS(1)
  {
    cycle++;
    cycle %= 4;
  }
}