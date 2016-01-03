#include <TimerOne.h>
#include <NewPing.h>
#include <Wire.h>

// Default analog write value of motors
#define MOTOR_DEFAULT_SPEED 150

// Min distance to detected object in cm
#define OBJECT_DETECTION_THRESHOLD 20

// Baudrate
#define BAUD_RATE 9600

// Pin to which buzzer is connected
#define BUZZER_PIN 8

// Pins to which motors are connected
#define MOTOR_1_POSITIVE 4
#define MOTOR_1_NEGATIVE 7

#define MOTOR_2_POSITIVE 9
#define MOTOR_2_NEGATIVE 10

#define MOTOR_1_ENABLE_PIN 5
#define MOTOR_2_ENABLE_PIN 6

// Setup the pins for Ultrasonic sensor
NewPing sonar(12,13,100);

//  Varaibles for Pulse rate monitoring
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0.
int blinkPin = 8;                 // Pin to blink led or beep buzzer at each beat.

// These variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                 // Used to hold the pulse rate.
volatile int Signal;              // Holds the incoming raw data.
volatile int IBI = 600;           // Holds the time between beats, the Inter-Beat Interval.
volatile boolean Pulse = false;   // True when pulse wave is high, False when it's low.
volatile boolean QS = false;      // Becomes true when Arduoino finds a beat.

// Flag for switching modes
bool joystick_enable_flag = false;

// Variable for storinng data from serial
byte serial_data = 's';

void setup()
{
    // Output Pins
    pinMode(MOTOR_1_POSITIVE,OUTPUT);
    pinMode(MOTOR_1_ENABLE_PIN,OUTPUT);
    pinMode(MOTOR_2_ENABLE_PIN,OUTPUT);
    pinMode(MOTOR_1_NEGATIVE,OUTPUT);
    pinMode(BUZZER_PIN,OUTPUT);
    pinMode(MOTOR_2_POSITIVE,OUTPUT);
    pinMode(MOTOR_2_NEGATIVE,OUTPUT);
    
    // Input pins for reading joystick
    pinMode(A1,INPUT); // Switch
    pinMode(A2,INPUT); // Y Axis
    pinMode(A3,INPUT); // X Axis

    // Stop The motors, if it is currently running
    halt();

    // Ultrasonic Sensor VCC (5V)
    pinMode(11,OUTPUT);
    digitalWrite(11,HIGH);

    // Initialize the Accelerometer
    InitAccelerometer();

    // Initialize the Pulse rate sensor (data pin, buzzer pin)
    pulseRateMonitorInit(0,2);

    // Begin Serial Communication with Raspberry Pi
    // Flush the buffer to avoid unwanted junk.
    Serial.begin(BAUD_RATE);
    Serial.flush();
}


void loop()
{
    // If Joystick mode enabled
    if(joystick_enable_flag == true)
    {  
      short x_axis = analogRead(A3);
      short y_axis = analogRead(A2);
      uint8_t motor_speed = 0;
      if(y_axis < 400)
      {
        motor_speed = map(y_axis, 0, 400, 255, 0);
        Serial.print("Go Up: ");Serial.println(motor_speed);
        forward(motor_speed);
      }
      else if(y_axis > 600)
      {
        motor_speed = map(y_axis, 600, 1023, 0, 255);
        Serial.print("Go Down: ");Serial.println(motor_speed);
        backwards(motor_speed);
      }
      else if(x_axis < 400)
      {
        motor_speed = map(x_axis, 0, 400, 255, 0);
        Serial.print("Go Left: ");Serial.println(motor_speed);
        anticlockwise(motor_speed);
      }
      else if(x_axis > 600)
      {
        motor_speed = map(x_axis, 600, 1023, 0, 255);
        Serial.print("Go Right: ");Serial.println(motor_speed);
        clockwise(motor_speed);
      }
      else
      {
        Serial.println("Halted...");
        halt();
      }
    }   
    else // IOT Mode
    {
      if(Serial.available())
      {
          // When serial data is available, read one byte at a time
          serial_data = Serial.read();
  
          // Figure out what to do...
          action(serial_data);
      }
      
      // Stop if object detected while moving forward 
      if(objectDetected(OBJECT_DETECTION_THRESHOLD) && serial_data == 'u')
      {
          halt();
      }
    }

    // When No data is available on Serial port
    // Send Pulse rate data to Raspberry Pi
    sendPulseRate();
    
    // Code Block for switching modes based on joystick switch press
    // If Switch Pressed for more than 2sec mode will be toggled    
    if(analogRead(A1) == 0)
    {
      unsigned long start_time = millis();
      while(analogRead(A1) == 0)
      {
        unsigned long time_elapsed = millis() - start_time;
        Serial.println(time_elapsed);
        if(time_elapsed > 2000)
        {
           if(joystick_enable_flag == true)
           {
            Serial.println("Switching to Normal Mode...");
            digitalWrite(BUZZER_PIN,HIGH);
            delay(500);
            digitalWrite(BUZZER_PIN,LOW);
            Serial.flush();
            joystick_enable_flag = false;
            
           }
           else
           {
            Serial.println("Switching to Joystick Mode...");
            digitalWrite(BUZZER_PIN,HIGH);
            delay(500);
            digitalWrite(BUZZER_PIN,LOW);
            joystick_enable_flag = true;
           }
           break;
        }
      }
    }    
}
void action(byte serial_data)
{
    switch(serial_data)
    {
        // If character F received move forward
        case 'u':
        {
            // Move if no object detected within 20cm
            if(!objectDetected(OBJECT_DETECTION_THRESHOLD))
            {
              int8_t angle = getPitchAngle();
              forward(MOTOR_DEFAULT_SPEED + angle);
            }
        }
        break;
    
        // If character B received move backward
        case 'd':
        {
            int8_t angle = getPitchAngle();
            backwards(MOTOR_DEFAULT_SPEED - angle);
        }
        break;
    
        // If character L received move anticlockwise
        case 'l':
        {
            int8_t angle = getPitchAngle();
            anticlockwise(MOTOR_DEFAULT_SPEED + angle);
        }
        break;
    
        // If character R received move clockwise
        case 'r':
        {
            int8_t angle = getPitchAngle();
            clockwise(MOTOR_DEFAULT_SPEED + angle);
        }
        break;
    
        // Any other character stop
        default:
            halt();
        break;
    }
}
void forward(uint8_t motor_speed)
{
    digitalWrite(MOTOR_1_POSITIVE,LOW);
    digitalWrite(MOTOR_1_NEGATIVE,HIGH);
    
    digitalWrite(MOTOR_2_POSITIVE,LOW);
    digitalWrite(MOTOR_2_NEGATIVE,HIGH);
    
    analogWrite(MOTOR_1_ENABLE_PIN,motor_speed);
    analogWrite(MOTOR_2_ENABLE_PIN,motor_speed);
}
void backwards(uint8_t motor_speed)
{
    digitalWrite(MOTOR_1_POSITIVE,HIGH);
    digitalWrite(MOTOR_1_NEGATIVE,LOW);
    
    digitalWrite(MOTOR_2_POSITIVE,HIGH);
    digitalWrite(MOTOR_2_NEGATIVE,LOW);
    
    analogWrite(MOTOR_1_ENABLE_PIN,motor_speed);
    analogWrite(MOTOR_2_ENABLE_PIN,motor_speed);    
}
void anticlockwise(uint8_t motor_speed)
{
    digitalWrite(MOTOR_1_POSITIVE,LOW);
    digitalWrite(MOTOR_1_NEGATIVE,HIGH);
    
    digitalWrite(MOTOR_2_POSITIVE,HIGH);
    digitalWrite(MOTOR_2_NEGATIVE,LOW);
    
    analogWrite(MOTOR_1_ENABLE_PIN,motor_speed);
    analogWrite(MOTOR_2_ENABLE_PIN,motor_speed);    
}
void clockwise(uint8_t motor_speed)
{
    digitalWrite(MOTOR_1_POSITIVE,HIGH);
    digitalWrite(MOTOR_1_NEGATIVE,LOW);
    
    digitalWrite(MOTOR_2_POSITIVE,LOW);
    digitalWrite(MOTOR_2_NEGATIVE,HIGH);
    
    analogWrite(MOTOR_1_ENABLE_PIN,motor_speed);
    analogWrite(MOTOR_2_ENABLE_PIN,motor_speed);    
}
void halt()
{
    digitalWrite(MOTOR_1_POSITIVE,LOW);
    digitalWrite(MOTOR_1_NEGATIVE,LOW);
    
    digitalWrite(MOTOR_2_POSITIVE,LOW);
    digitalWrite(MOTOR_2_NEGATIVE,LOW);
    
    analogWrite(MOTOR_1_ENABLE_PIN,0);
    analogWrite(MOTOR_2_ENABLE_PIN,0);
}
bool objectDetected(uint8_t threshold)
{
    uint8_t distance = (sonar.ping() / US_ROUNDTRIP_CM);
    if( distance <= threshold && distance != 0 )
    {
       digitalWrite(BUZZER_PIN,HIGH);
       return true;
    }
    else
    {
       digitalWrite(BUZZER_PIN,LOW);
       return false;
    }
}
void InitAccelerometer()
{
    Wire.begin();

    // I2C Address of MPU 6050
    Wire.beginTransmission(0x68);

    // PWR_MGMT_1 register (Power Management)
    Wire.write(0x6B);

    // Set to zero (Wakes up the MPU-6050)
    Wire.write(0);

    Wire.endTransmission(true);
}

float getAccZ()
{
    Wire.beginTransmission(0x68);

    // Read From Register 0x3F (ACCEL_ZOUT_H)
    Wire.write(0x3F);

    Wire.endTransmission(false);

    // Request a total of 2 registers
    Wire.requestFrom(0x68, 2, true);

    // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    return (((Wire.read() << 8) | (Wire.read())) * 0.0006125);
}
float getAccY()
{
    Wire.beginTransmission(0x68);

    // Read From Register 0x3D (ACCEL_YOUT_H)
    Wire.write(0x3D);

    Wire.endTransmission(false);

    // Request a total of 2 registers
    Wire.requestFrom(0x68, 2, true);

    // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    return (((Wire.read() << 8) | (Wire.read())) * 0.0006125);
}

float getAccX()
{
    Wire.beginTransmission(0x68);

    // Read From Register 0x3B (ACCEL_XOUT_H)
    Wire.write(0x3B);

    Wire.endTransmission(false);

    // Request a total of 2 registers
    Wire.requestFrom(0x68, 2, true);

    // 0x3B (ACCEL_ZOUT_H) & 0x3C (ACCEL_ZOUT_L)
    return (((Wire.read() << 8) | (Wire.read())) * 0.0006125);
}
float getPitchAngle()
{
    float AccX = getAccX();
    float AccY = getAccY();
    float AccZ = getAccZ();

    // Return the Angle with the horizontal
    // +ve value for uphill, -ve value for downhill
    return (atan(AccX/sqrt((AccY*AccY) + (AccZ*AccZ))) * 9.8 * 9.8) + 4.5;
}
int8_t sendPulseRate()
{
    if (QS == true)
    {
        // Send the Pulse rate to Raspberry Pi
        Serial.print('*');
        Serial.print(BPM);
        Serial.print('#');
        
        // Reset the Quantified Self flag for next time
        QS = false;
    }
}
void pulseRateMonitorInit(int pulse_pin, int blink_pin)
{
    int pulsePin = pulse_pin;
    int blinkPin = blink_pin;
    
    // Set the Buzzer Pin as Output
    pinMode(blinkPin,OUTPUT);
    
    // Initialize Timer 1 to interrupt every 2ms
    Timer1.initialize(2000);
    
    // Attaches callback() as a timer overflow interrupt
    // Called every 2ms
    Timer1.attachInterrupt(callback);  
}
