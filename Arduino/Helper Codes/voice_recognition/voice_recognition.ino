#include <SoftwareSerial.h>

SoftwareSerial bluetooth(2,3); // RX, TX

// The Keywords for recognizing the action command from bluetooth
#define FORWARD_KEYWORD   "forward"
#define BACKWARD_KEYWORD  "back"
#define LEFT_KEYWORD      "left"
#define RIGHT_KEYWORD     "right"
#define HALT_KEYWORD      "stop"

void setup() 
{
  Serial.begin(9600);
  bluetooth.begin(9600);
}

void loop() 
{
    parseBluetooth();
}

void parseBluetooth()
{
    char raw_string[100];
    uint8_t i = 0, len, n, x, y;
    String recognized;
    if(bluetooth.available() > 2)
    {
        //Wait for the entire raw_string to come into buffer
        delay(100);                           
        n = bluetooth.available();

        // Read the bytes from buffer
        for(i = 0; i < n; i++)
        {
          raw_string[i]=bluetooth.read();
        }
        len = n;
        raw_string[n]='\0';

        //Storing the read raw_string in a string variable
        recognized = raw_string;

        // Extracting the required string ignoring * and #
        recognized = recognized.substring(1, n-1);
        len-=2;

        // Extracted String
        Serial.println(recognized); 

        if(strstr(raw_string,FORWARD_KEYWORD))
        {
          Serial.println("F");
        }
        else if(strstr(raw_string,BACKWARD_KEYWORD))
        {
          Serial.println("B");
        }
        else if(strstr(raw_string,LEFT_KEYWORD))
        {
          Serial.println("R");
        }
        else if(strstr(raw_string,RIGHT_KEYWORD))
        {
          Serial.println("L");
        }
        else if(strstr(raw_string,HALT_KEYWORD))
        {
          Serial.println("S");
        }
        else
        {
          Serial.println("Unrecognized");
        }
    }
}

