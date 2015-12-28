#include <TimerOne.h>
#include <NewPing.h>
#include <Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(2,3); // RX, TX

// Default analog write value of motors
#define MOTOR_DEFAULT_SPEED 150

// Min distance to detected object in cm
#define OBJECT_DETECTION_THRESHOLD 20

// Baudrate
#define BAUD_RATE 9600

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

void setup()
{
    // Motor Control Pins
    pinMode(5,OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(9,OUTPUT);
    pinMode(10,OUTPUT);

    // Stop The motors, if it is currently running
    halt();

    // Ultrasonic Sensor VCC (5V)
    pinMode(11,OUTPUT);
    digitalWrite(11,HIGH);

    // Initialize the Accelerometer
    InitAccelerometer();

    // Initialize the Pulse rate sensor (data pin, buzzer pin)
    pulseRateMonitorInit(0,8);

    // Begin Serial Communication with Raspberry Pi
    // Flush the buffer to avoid unwanted junk.
    Serial.begin(BAUD_RATE);
    Serial.flush();

    bluetooth.begin(BAUD_RATE);
    bluetooth.flush();
}

void loop()
{
    if(Serial.available())
    {
        // When serial data is available, read one byte at a time
        byte serial_data = Serial.read();

        // Figure out what to do...
        action(serial_data);
    }

    // When No data is available on Serial port
    // Send Pulse rate data to Raspberry Pi
    sendPulseRate();
    
}
void action(byte serial_data)
{
    switch(serial_data)
    {
        // If character F received move forward
        case 'F':
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
        case 'B':
        {
            // Move if no object detected within 20cm
            if(!objectDetected(OBJECT_DETECTION_THRESHOLD))
            {
              int8_t angle = getPitchAngle();
              backwards(MOTOR_DEFAULT_SPEED - angle);
            }
        }
        break;
    
        // If character L received move anticlockwise
        case 'L':
        {
            // Move if no object detected within 20cm
            if(!objectDetected(OBJECT_DETECTION_THRESHOLD))
            {
              int8_t angle = getPitchAngle();
              anticlockwise(MOTOR_DEFAULT_SPEED + angle);
            }
        }
        break;
    
        // If character R received move clockwise
        case 'R':
        {
            // Move if no object detected within 20cm
            if(!objectDetected(OBJECT_DETECTION_THRESHOLD))
            {
              int8_t angle = getPitchAngle();
              clockwise(MOTOR_DEFAULT_SPEED + angle);
            }
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
    digitalWrite(5,LOW);
    analogWrite(6,motor_speed);
    digitalWrite(9,LOW);
    analogWrite(10,motor_speed);
}
void backwards(uint8_t motor_speed)
{
    analogWrite(5,motor_speed);
    digitalWrite(6,LOW);
    analogWrite(9,motor_speed);
    digitalWrite(10,LOW);
}
void anticlockwise(uint8_t motor_speed)
{
    digitalWrite(5,LOW);
    analogWrite(6,motor_speed);
    analogWrite(9,motor_speed);
    digitalWrite(10,LOW);
}
void clockwise(uint8_t motor_speed)
{
    analogWrite(5,motor_speed);
    digitalWrite(6,LOW);
    digitalWrite(9,LOW);
    analogWrite(10,motor_speed);
}
void halt()
{
    digitalWrite(5,LOW);
    digitalWrite(6,LOW);
    digitalWrite(9,LOW);
    digitalWrite(10,LOW);
}
bool objectDetected(uint8_t threshold)
{
    uint8_t distance = (sonar.ping() / US_ROUNDTRIP_CM);
    if( distance <= threshold && distance != 0 )
    {
       return true;
    }
    else
    {
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
        Serial.print('B');Serial.println(BPM);
        
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
