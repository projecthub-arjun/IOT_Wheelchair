// This acts as a slave node for controlling devices from the main server
#include <SoftwareSerial.h>

// Devices are connected to these pins
#define DEVICE_1 2
#define DEVICE_2 3
#define DEVICE_3 4
#define DEVICE_4 13

// The Bluetooth module is connected to these pins
SoftwareSerial bluetooth(10, 11); // RX, TX

bool device_1_state = LOW;
bool device_2_state = LOW;
bool device_3_state = LOW;
bool device_4_state = LOW;

void setup() 
{
  bluetooth.begin(9600);
  Serial.begin(9600);
  
  // Initialize the device pins as OUTPUT
  pinMode(DEVICE_1,OUTPUT);
  pinMode(DEVICE_2,OUTPUT);
  pinMode(DEVICE_3,OUTPUT);
  pinMode(DEVICE_4,OUTPUT);
}

unsigned long start_time = millis();
void loop() 
{
  if(bluetooth.available())
  {
    char received_data = bluetooth.read();
    switch(received_data)
    {
      case 'a':
        device_1_state = !device_1_state;
        Serial.print("Switching Device 1 : ");Serial.println(device_1_state);
        digitalWrite(DEVICE_1,device_1_state);
      break;
      
      case 'b':
        device_2_state = !device_2_state;
        Serial.print("Switching Device 2 : ");Serial.println(device_2_state);
        digitalWrite(DEVICE_2,device_2_state);
      break;
      
      case 'c':
        device_3_state = !device_3_state;
        Serial.print("Switching Device 3 : ");Serial.println(device_3_state);
        digitalWrite(DEVICE_3,device_3_state);
      break;

      case 'd':
        device_4_state = !device_4_state;
        Serial.print("Switching Device 4 : ");Serial.println(device_4_state);
        digitalWrite(DEVICE_4,device_4_state);
      break;
      
      case 'x':
        device_1_state = LOW;
        device_2_state = LOW;
        device_3_state = LOW;
        device_4_state = LOW;
        digitalWrite(DEVICE_1,device_1_state);
        digitalWrite(DEVICE_2,device_2_state);
        digitalWrite(DEVICE_3,device_3_state);
        digitalWrite(DEVICE_4,device_4_state);
      break;
    }
  }
  if(millis() - start_time > 5000)
  {
    Serial.print("Syncing Devices : ");
    bluetooth.print('*');
    bluetooth.print(device_1_state);
    bluetooth.print(device_2_state);
    bluetooth.print(device_3_state);
    bluetooth.print(device_4_state);
    bluetooth.print('#');
    start_time = millis();
  }
}
