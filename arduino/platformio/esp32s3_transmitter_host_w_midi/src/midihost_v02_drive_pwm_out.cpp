#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

void handleNoteOn(byte channel, byte pitch, byte velocity);
void handleNoteOff(byte channel, byte pitch, byte velocity);
void handleControlChange(byte channel, byte data1, byte data2);

#define rcOutPin1 5
#define rcOutPin2 4

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  usb_midi.setStringDescriptor("TinyUSB MIDI");

  // Initialize MIDI, and listen to all MIDI channels
  // This will also call usb_midi's begin()
  MIDI.begin(MIDI_CHANNEL_OMNI);

  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.setHandleControlChange(handleControlChange);

  // wait until device mounted
  while (!TinyUSBDevice.mounted())
    delay(1);

  Serial.println("hello world - ready for action");
  digitalWrite(LED_BUILTIN, HIGH);

  // define pwm out pins
  ledcSetup(0, 10000, 8); // Setup channel at specified Hz with 8 (0-255), 12 (0-4095), or 16 (0-65535) bit resolution
  ledcSetup(1, 10000, 8); // Setup channel at specified Hz with 8 (0-255), 12 (0-4095), or 16 (0-65535) bit resolution

  ledcAttachPin(rcOutPin1, 0);
  ledcAttachPin(rcOutPin2, 1);
}


void loop()
{
  // read any new MIDI messages
  // DO NOT INTRODUCE A DELAY -> the usbMIDI.read() needs to be called rapidly from loop()
  MIDI.read();
}

void handleControlChange(byte channel, byte data1, byte data2)
{
  Serial.println("Receive CC >>  channel: " + String(channel) + ", data1: " + String(data1) + ", data2: " + String(data2));

  // CC0
  if (channel == 1 && data1 == 0)
  {
      // rcChannels[AILERON] = map(data2, 0, 127, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
      ledcWrite(rcOutPin1, data2 * 2);
      Serial.println("Receive CC >>  writing to pin: " + String(rcOutPin1) + ", value: " + String(data2 * 2));
  }

  // CC1
  if (channel == 1 && data1 == 1)
  {
      // rcChannels[ELEVATOR] = map(data2, 0, 127, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
      ledcWrite(rcOutPin2, data2 * 2);
      Serial.println("Receive CC >>  writing to pin: " + String(rcOutPin2) + ", value: " + String(data2 * 2));
  }

  /*
  if (channel == 1 && data1 == 2)
  {
      rcChannels[THROTTLE] = map(data2, 0, 127, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
  }

  if (channel == 1 && data1 == 3)
  {
      rcChannels[RUDDER] = map(data2, 0, 127, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
  }
  //*/
}

void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  // Log when a note is pressed.
  Serial.print("Note on: channel = ");
  Serial.print(channel);

  Serial.print(" pitch = ");
  Serial.print(pitch);

  Serial.print(" velocity = ");
  Serial.println(velocity);
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  // Log when a note is released.
  Serial.print("Note off: channel = ");
  Serial.print(channel);

  Serial.print(" pitch = ");
  Serial.print(pitch);

  Serial.print(" velocity = ");
  Serial.println(velocity);
}
