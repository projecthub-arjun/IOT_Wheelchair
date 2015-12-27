#include <NewPing.h>

// Setup the pins for Ultrasonic sensor
NewPing sonar(12,13,100);

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

  // Begin Serial Communication with Raspberry Pi
  // Flush the buffer to avoid unwanted junk.
  Serial.begin(9600);
  Serial.flush();
}

void loop() 
{
  if(Serial.available())
  {
    // When serial data is available, read one byte at a time
    byte serial_data = Serial.read();
    
    // Figure out what to do...
    switch(serial_data)
    {
      // If character F received move forward
      case 'F':
      {
        // Move if no object detected within 20cm
        if(!objectDetected(20))
        {
          forward(100);
        }
      }
      break;
      
      // If character B received move backward
      case 'B':
      {
        // Move if no object detected within 20cm
        if(!objectDetected(20))
        {      
          backwards(100);
        }
      }
      break;
      
      // If character L received move anticlockwise
      case 'L':
      {
        // Move if no object detected within 20cm
        if(!objectDetected(20))
        {      
          anticlockwise(100);
        }
      }
      break;
      
      // If character R received move clockwise
      case 'R':
      {
        // Move if no object detected within 20cm
        if(!objectDetected(20))
        {
          clockwise(100);
        }
      }
      break;
      
      // Any other character stop
      default:
      halt();
      break;
    }
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
   uint8_t distance = (sonar.ping() / US_ROUNDTRIP_CM );
   if( distance <= threshold && distance != 0 )
   {
     return true;
   }
   else
   {
     return false;
   }
 }
