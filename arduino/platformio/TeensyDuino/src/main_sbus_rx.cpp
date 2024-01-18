#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "sbus.h"

// SoftwareSerial mySerial(37, 39); // RX, TX

// HardwareSerial MySerial0(0);

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1, true, false);

#define PIN 17      // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 2 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

/* SBUS data */
bfs::SbusData data;

u_long sbusPrevPacketTime;
bool sbusLost = false;

#define SBUS_VAL_MIN 176
#define SBUS_VAL_MAX 1800
#define SBUS_VAL_CENTER 988
#define SBUS_VAL_DEADBAND 5
#define SBUS_LOST_TIMEOUT 100
#define SBUS_SWITCH_MIN 192
#define SBUS_SWITCH_MAX 1792
#define SBUS_SWITCH_MIN_THRESHOLD 1400
#define SBUS_SWITCH_MAX_THRESHOLD 550

#define TX_ROLL 0
#define TX_PITCH 1
#define TX_THROTTLE 2
#define TX_YAW 3
#define TX_AUX1 4
#define TX_AUX2 5
#define TX_AUX3 6
#define TX_AUX4 7

#define SBUS_PACKET_PRINT_INTERVAL 100 // ms
u_long sbusPacketPrintPrevTime = 0;


// define the methods used in this sketch (prevents compile errors)
void processMIDI(void);
void printBytes(const byte *data, unsigned int size);

byte ledPWMVal = 0;
byte aux1_val = 0;
byte aux2_val = 0;

uint16_t neopixelHVal = 0;
byte neopixelSVal = 0;
byte neopixelVVal = 0;

byte neopixel_rVal = 0;
byte neopixel_gVal = 0;
byte neopixel_bVal = 0;

#define LED_PIN 10
#define NOOD_BLUE_PIN 23
#define NOOD_WARMWHITE_PIN 22

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

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(NOOD_1_PIN, OUTPUT);
  pinMode(NOOD_2_PIN, OUTPUT);

  BlinkLed(2);

    /* Begin the SBUS communication */
  sbus_rx.Begin();

  // by default, let's have the program assume sbus is lost
  sbusPrevPacketTime = -SBUS_LOST_TIMEOUT;


  // from https://www.pjrc.com/teensy/teensy31.html
  analogWriteResolution(12);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

void loop()
{
  /*
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  */

  analogWrite(A14, ledPWMVal << 4);

  analogWrite(LED_PIN, ledPWMVal);
  analogWrite(NOOD_1_PIN, aux1_val);
  analogWrite(NOOD_2_PIN, aux2_val);


  //*
  // read SBUS
  if (sbus_rx.Read())
  {
    sbusPrevPacketTime = millis();
    if (sbusLost)
    {
      Serial.println("Regained SBUS connection");
      sbusLost = false;
    }

    data = sbus_rx.data();

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
  //*/




  pixels.clear(); // Set all pixel colors to 'off'
  uint32_t ledstripVal = 0;
  // ledstripVal = pixels.ColorHSV(neopixelHVal, neopixelSVal, neopixelVVal);
  ledstripVal = pixels.Color (neopixel_rVal, neopixel_gVal, neopixel_bVal);
  // hsvVal = pixels.ColorHSV(0, 255, 255);
  pixels.fill(ledstripVal);
  pixels.show(); // Send the updated pixel colors to the hardware.

  // usbMIDI.read() needs to be called rapidly from loop().  When
  // each MIDI messages arrives, it return true.  The message must
  // be fully processed before usbMIDI.read() is called again.
  if (usbMIDI.read())
  {
    processMIDI();
  }
}

void processMIDI(void)
{
  byte type, channel, data1, data2, cable;

  // fetch the MIDI message, defined by these 5 numbers (except SysEX)
  //
  type = usbMIDI.getType();       // which MIDI message, 128-255
  channel = usbMIDI.getChannel(); // which MIDI channel, 1-16
  data1 = usbMIDI.getData1();     // first data byte of message, 0-127
  data2 = usbMIDI.getData2();     // second data byte of message, 0-127
  cable = usbMIDI.getCable();     // which virtual cable with MIDIx8, 0-7

  // uncomment if using multiple virtual cables
  // Serial.print("cable ");
  // Serial.print(cable, DEC);
  // Serial.print(": ");

  // print info about the message
  //
  switch (type)
  {
  case usbMIDI.NoteOff: // 0x80
    Serial.print("Note Off, ch=");
    Serial.print(channel, DEC);
    Serial.print(", note=");
    Serial.print(data1, DEC);
    Serial.print(", velocity=");
    Serial.println(data2, DEC);
    break;

  case usbMIDI.NoteOn: // 0x90
    Serial.print("Note On, ch=");
    Serial.print(channel, DEC);
    Serial.print(", note=");
    Serial.print(data1, DEC);
    Serial.print(", velocity=");
    Serial.println(data2, DEC);
    break;

  case usbMIDI.AfterTouchPoly: // 0xA0
    Serial.print("AfterTouch Change, ch=");
    Serial.print(channel, DEC);
    Serial.print(", note=");
    Serial.print(data1, DEC);
    Serial.print(", velocity=");
    Serial.println(data2, DEC);
    break;

  case usbMIDI.ControlChange: // 0xB0
    // Serial.print("Control Change, ch=");
    // Serial.print(channel, DEC);
    // Serial.print(", control=");
    // Serial.print(data1, DEC);
    // Serial.print(", value=");
    // Serial.println(data2, DEC);

    if (channel == 1 && data1 == 0)
    {
      ledPWMVal = data2 * 2;
    }

    if (channel == 1 && data1 == 1)
    {
      aux1_val = data2 * 2;
    }
    if (channel == 1 && data1 == 2)
    {
      aux2_val = data2 * 2;
    }

    if (channel == 1 && data1 == 10)
    {
      neopixelHVal = (data2 * 2) << 8;
    }
    if (channel == 1 && data1 == 11)
    {
      neopixelSVal = data2 * 2;
    }
    if (channel == 1 && data1 == 12)
    {
      neopixelVVal = data2 * 2;
    }

    if (channel == 1 && data1 == 20)
    {
      neopixel_rVal = data2 * 2;
    }
    if (channel == 1 && data1 == 21)
    {
      neopixel_gVal = data2 * 2;
    }
    if (channel == 1 && data1 == 22)
    {
      neopixel_bVal = data2 * 2;
    }
    break;

  case usbMIDI.ProgramChange: // 0xC0
    Serial.print("Program Change, ch=");
    Serial.print(channel, DEC);
    Serial.print(", program=");
    Serial.println(data1, DEC);
    break;

  case usbMIDI.AfterTouchChannel: // 0xD0
    Serial.print("After Touch, ch=");
    Serial.print(channel, DEC);
    Serial.print(", pressure=");
    Serial.println(data1, DEC);
    break;

  case usbMIDI.PitchBend: // 0xE0
    Serial.print("Pitch Change, ch=");
    Serial.print(channel, DEC);
    Serial.print(", pitch=");
    Serial.println(data1 + data2 * 128, DEC);
    break;

  case usbMIDI.SystemExclusive: // 0xF0
    // Messages larger than usbMIDI's internal buffer are truncated.
    // To receive large messages, you *must* use the 3-input function
    // handler.  See InputFunctionsComplete for details.
    Serial.print("SysEx Message: ");
    printBytes(usbMIDI.getSysExArray(), data1 + data2 * 256);
    Serial.println();
    break;

  case usbMIDI.TimeCodeQuarterFrame: // 0xF1
    Serial.print("TimeCode, index=");
    Serial.print(data1 >> 4, DEC);
    Serial.print(", digit=");
    Serial.println(data1 & 15, DEC);
    break;

  case usbMIDI.SongPosition: // 0xF2
    Serial.print("Song Position, beat=");
    Serial.println(data1 + data2 * 128);
    break;

  case usbMIDI.SongSelect: // 0xF3
    Serial.print("Sond Select, song=");
    Serial.println(data1, DEC);
    break;

  case usbMIDI.TuneRequest: // 0xF6
    Serial.println("Tune Request");
    break;

  case usbMIDI.Clock: // 0xF8
    Serial.println("Clock");
    break;

  case usbMIDI.Start: // 0xFA
    Serial.println("Start");
    break;

  case usbMIDI.Continue: // 0xFB
    Serial.println("Continue");
    break;

  case usbMIDI.Stop: // 0xFC
    Serial.println("Stop");
    break;

  case usbMIDI.ActiveSensing: // 0xFE
    Serial.println("Actvice Sensing");
    break;

  case usbMIDI.SystemReset: // 0xFF
    Serial.println("System Reset");
    break;

  default:
    Serial.println("Opps, an unknown MIDI message type!");
  }
}

void printBytes(const byte *data, unsigned int size)
{
  while (size > 0)
  {
    byte b = *data++;
    if (b < 16)
      Serial.print('0');
    Serial.print(b, HEX);
    if (size > 1)
      Serial.print(' ');
    size = size - 1;
  }
}
