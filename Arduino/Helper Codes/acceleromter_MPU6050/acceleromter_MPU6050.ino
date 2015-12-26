#include<Wire.h>
void setup()
{
  InitAccelerometer();
  Serial.begin(9600);
}
void loop()
{
  Serial.print( "Angle = " ); Serial.println( getPitchAngle() );
  delay(250);
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
  
  return (atan(AccX/sqrt((AccY*AccY) + (AccZ*AccZ))) * 9.8 * 9.8) + 4.5;
}
