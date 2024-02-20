#include <Wire.h>

// Exploring using direct ledc register control (instead of analogWrite)
//

#define nood1a D1
#define nood1b D10

const uint8_t nood1a_chan = 9;
const uint8_t nood1b_chan = 10;

// setting PWM properties
const uint32_t freq = 2500;
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

bool tryDetach(uint8_t pin) {
  if (ledcRead(pin) != 0) {
    ledcDetachPin(pin);
  }
}

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 2; i < 11; i++) {
    tryDetach(i);
  }
  // ledcDetachPin(2);
  // ledcDetachPin(3);
  // ledcDetachPin(4);
  // ledcDetachPin(5);
  // ledcDetachPin(6);
  // ledcDetachPin(7);
  // ledcDetachPin(8);
  // ledcDetachPin(9);
  // ledcDetachPin(10);

  /*
  // to prevent the motors from running
  pinMode(motor1a_pin, OUTPUT);
  pinMode(motor1b_pin, OUTPUT);
  pinMode(motor2a_pin, OUTPUT);
  pinMode(motor2b_pin, OUTPUT);

  // to prevent the motors from running
  analogWrite(motor1a_pin, 127);
  analogWrite(motor1b_pin, 127);
  analogWrite(motor2a_pin, 127);
  analogWrite(motor2b_pin, 127);
  //*/

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


  pinMode(nood1a, OUTPUT);
  pinMode(nood1b, OUTPUT);

  ledcSetup(nood1a_chan, freq, resolution);
  ledcAttachPin(nood1a, nood1a_chan);

  ledcSetup(nood1b_chan, freq, resolution);
  ledcAttachPin(nood1b, nood1b_chan);

  Serial.println("hello world");
}

void loop() {
  ledcWrite(nood1a, 255);
  ledcWrite(nood1b, 220);
  delay(200);
  ledcWrite(nood1b, 127);
  delay(200);
  ledcWrite(nood1b, 0);
  delay(500);

  ledcWrite(nood1b, 255);
  ledcWrite(nood1a, 220);
  delay(200);
  ledcWrite(nood1a, 127);
  delay(200);
  ledcWrite(nood1a, 0);
  delay(500);
}