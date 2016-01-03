// This sketch could be used for configuring HC-05 bluetooth mode as master
// and make it connect to other HC-06 modules
#include <SoftwareSerial.h>

#define ROBOT_NAME "IOT_MASTER_NODE"

// In command mode HC-05 defaults to this baudrate
#define BLUETOOTH_SPEED 38400 //This is the default baudrate that HC-05 uses

// Pins to which HC-05 is connected
SoftwareSerial mySerial(10, 11); // RX, TX

//The posible baudrates are for normal mode:
//  AT+UART=1200,0,0 -------1200
//  AT+UART=2400,0,0 -------2400
//  AT+UART=4800,0,0 -------4800
//  AT+UART=9600,0,0 -------9600  (default)
//  AT+UART=19200,0,0 ------19200
//  AT+UART=38400,0,0 ------38400
//  AT+UART=57600,0,0 ------57600 
//  AT+UART=115200,0,0 -----115200
//  AT+UART=230400,0,0 -----230400
//  AT+UART=460800,0,0 -----460800
//  AT+UART=921600,0,0 -----921600
//  AT+UART=1382400,0,0 ----1382400

void setup() 
{
  // Connect Key pin (PIN 34) here
  pinMode(9, OUTPUT);  
  digitalWrite(9, HIGH);
  
  Serial.begin(9600);
  while (!Serial) 
  {
  }
  Serial.println("Starting Configuration");
  mySerial.begin(BLUETOOTH_SPEED);
  delay(1000);

  // Should respond with OK
  mySerial.print("AT\r\n");
  waitForResponse();

  // Should respond with its version
  //  mySerial.print("AT+VERSION\r\n");
  //  waitForResponse();

  // Set the name of bluetooth device
  //  String rnc = String("AT+NAME=") + String(ROBOT_NAME) + String("\r\n"); 
  //  mySerial.print(rnc);
  //  waitForResponse();

  // Connect to any slave device
  mySerial.print("AT+CMODE=1\r\n");
  waitForResponse();
  
  // Initialize the CMDOE just set
  mySerial.print("AT+INIT\r\n");
  waitForResponse();
  
  // Search for nearby devices
  mySerial.print("AT+INQ\r\n");
  waitForResponse();
  
  Serial.println("Done!");
}

void waitForResponse() 
{
    delay(1000);
    while (mySerial.available()) 
    {
      Serial.write(mySerial.read());
    }
    Serial.write("\n");
}

void loop() 
{

}
