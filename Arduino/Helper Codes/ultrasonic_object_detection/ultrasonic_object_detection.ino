#include <NewPing.h>
NewPing sonar(12,13,100);
void setup() 
{
  pinMode(11,OUTPUT);
  digitalWrite(11,HIGH);
  Serial.begin(9600);
}

void loop() 
{
  Serial.print("OBJECT DETECTED "); Serial.println(objectDetected(20));
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
