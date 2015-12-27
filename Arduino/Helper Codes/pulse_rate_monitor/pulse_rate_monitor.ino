//  VARIABLES
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 8;                // pin to blink led at each beat

// these variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, the Inter-Beat Interval
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

void setup()
{
  Serial.begin(115200);             // we agree to talk fast!
  pulseRateMonitorInit(0,8); 
}

void loop()
{
  sendPulseRate();
}

int8_t sendPulseRate()
{
  if (QS == true)
  {
    Serial.print('B');Serial.println(BPM);
    // reset the Quantified Self flag for next time
    QS = false;
  } 
}
void pulseRateMonitorInit(int pulse_pin, int blink_pin)
{
  int pulsePin = pulse_pin; 
  int blinkPin = blink_pin;
  pinMode(blinkPin,OUTPUT);
  interruptSetup();
}





