

/* Debug Messages */
#ifdef DEBUG_BUILD
  #define DEBUG(x) Serial.println(x)
#else
  #define DEBUG(x) do {} while (0)
#endif

// #define TEST_MODE

// Motor Pins
#define LEFT_MOTOR_PIN1 18
#define LEFT_MOTOR_PIN2 19

#define RIGHT_MOTOR_PIN1 16
#define RIGHT_MOTOR_PIN2 17

// Encoder Pins
#define FRONT_LEFT_ENCODER_A 26 
#define FRONT_LEFT_ENCODER_B 27 

#define FRONT_RIGHT_ENCODER_A 23 
#define FRONT_RIGHT_ENCODER_B 32 

#define REAR_LEFT_ENCODER_A 14 
#define REAR_LEFT_ENCODER_B 13 

#define REAR_RIGHT_ENCODER_A 25 
#define REAR_RIGHT_ENCODER_B 33 

// PWM Pins
#define LEFT_PWM_PIN 2
#define RIGHT_PWM_PIN 4
#define PWM_FREQ 5000
#define PWM_BIT_RESOLUTION 8
#define PWM_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define PWM_CLOCK_CONFIG LEDC_USE_APB_CLK
#define PWM_CLOCK_FREQ APB_CLK_FREQ
#define PWM_LEFT_CHANNEL LEDC_CHANNEL_0
#define PWM_RIGHT_CHANNEL LEDC_CHANNEL_1
#define PWM_LEFT_TIMER LEDC_TIMER_0
#define PWM_RIGHT_TIMER LEDC_TIMER_1

// Encoder pulse count
#define PULSE_COUNT_GA25_370 370

// I2C Comms
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// Orintation Sensor
#define SENSOR_9AXIS_ADDRESS 0x28