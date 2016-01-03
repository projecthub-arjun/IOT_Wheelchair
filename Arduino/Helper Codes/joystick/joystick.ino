void setup() 
{

  pinMode(11,OUTPUT); // VCC
  digitalWrite(11,HIGH);
  
  Serial.begin(9600);

}

// put your main code here, to run repeatedly:
bool joystick_enable_flag = false;
  
void loop() 
{

  if(analogRead(A1) == 0)
  {
    unsigned long start_time = millis();
    while(analogRead(A1) == 0)
    {
      if(millis() - start_time > 2000)
      {
         if(joystick_enable_flag == true)
         {
          Serial.println("Switching to Normal Mode...");
          joystick_enable_flag = false;
         }
         else
         {
          Serial.println("Switching to Joystick Mode...");
          joystick_enable_flag = true;
         }
         break;
      }
    }
  }
  
  if(joystick_enable_flag == true)
  {  
    short x_axis = analogRead(A3);
    short y_axis = analogRead(A2);
    uint8_t motor_speed = 0;
    if(y_axis < 400)
    {
      motor_speed = map(y_axis, 0, 400, 255, 0);
      Serial.print("Go Up: ");Serial.println(motor_speed);
    }
    else if(y_axis > 600)
    {
      motor_speed = map(y_axis, 600, 1023, 0, 255);
      Serial.print("Go Down: ");Serial.println(motor_speed);
    }
    else if(x_axis < 400)
    {
      motor_speed = map(x_axis, 0, 400, 255, 0);
      Serial.print("Go Left: ");Serial.println(motor_speed);
    }
    else if(x_axis > 600)
    {
      motor_speed = map(x_axis, 600, 1023, 0, 255);
      Serial.print("Go Right: ");Serial.println(motor_speed);
    }
    else
    {
      Serial.println("Halted...");
    }
  }
  
  delay(100);
}
