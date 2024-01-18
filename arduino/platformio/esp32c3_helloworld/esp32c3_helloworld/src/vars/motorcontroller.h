#define PIN_MOTOR1_A D5
#define PIN_MOTOR1_B D4
#define CH_MOTOR1_1 2 // 16 Channels (0-15) are availible
#define CH_MOTOR1_2 3 // Make sure each pin is a different channel and not in use by other PWM devices (servos, LED's, etc)

#define PIN_MOTOR2_A D3
#define PIN_MOTOR2_B D2
#define CH_MOTOR2_1 4 // 16 Channels (0-15) are availible
#define CH_MOTOR2_2 5 // Make sure each pin is a different channel and not in use by other PWM devices (servos, LED's, etc)

// Optional Parameters
#define RES 8     // Resolution in bits:  8 (0-255),  12 (0-4095), or 16 (0-65535)
#define FREQ 5000 // PWM Frequency in Hz

