volatile int rate[10];                    // used to hold last ten IBI values
volatile unsigned long sampleCounter = 0; // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;  // used to find the inter beat interval
volatile int P = 512;                     // used to find peak in pulse wave
volatile int T = 512;                     // used to find trough in pulse wave
volatile int thresh = 512;                // used to find instant moment of heart beat
volatile int amp = 100;                   // used to hold amplitude of pulse waveform
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = true;       // used to seed rate array so we startup with reasonable BPM

// THIS IS THE TIMER 1 INTERRUPT SERVICE ROUTINE.
// Timer 1 makes sure that we take a reading every 2 miliseconds
// Triggered when Timer1 counts to 124
void callback()
{
  Signal = analogRead(pulsePin);              // read the Pulse Sensor
  sampleCounter += 2;                         // keep track of the time in mS with this variable
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

  //  find the peak and trough of the pulse wave
  // avoid dichrotic noise by waiting 3/5 of last IBI
  if (Signal < thresh && N > (IBI / 5) * 3) 
  { 
    // T is the trough
    if (Signal < T) 
    {                       
      T = Signal;                         // keep track of lowest point in pulse wave
    }
  }

  // thresh condition helps avoid noise
  if (Signal > thresh && Signal > P) 
  {        
    P = Signal;                             // P is the peak
  }                                        // keep track of highest point in pulse wave

  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  // avoid high frequency noise
  if (N > 250) 
  {                                  
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI / 5) * 3) ) 
    {
      Pulse = true;                               // set the Pulse flag when we think there is a pulse
      digitalWrite(blinkPin, HIGH);               // turn on pin 13 LED
      IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
      lastBeatTime = sampleCounter;               // keep track of time for next pulse

      // if it's the first time we found a beat, if firstBeat == TRUE
      if (firstBeat) 
      {                       
        firstBeat = false;                 // clear firstBeat flag
        return;                            // IBI value is unreliable so discard it
      }
      // if this is the second beat, if secondBeat == TRUE
      if (secondBeat) 
      {                      
        secondBeat = false;                 // clear secondBeat flag
        // seed the running total to get a realisitic BPM at startup
        for (int i = 0; i <= 9; i++) 
        {   
          rate[i] = IBI;
        }
      }

      // keep a running total of the last 10 IBI values
      word runningTotal = 0;                   // clear the runningTotal variable

      // shift data in the rate array
      for (int i = 0; i <= 8; i++) 
      {          
        rate[i] = rate[i + 1];            // and drop the oldest IBI value
        runningTotal += rate[i];          // add up the 9 oldest IBI values
      }

      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values
      BPM = 60000 / runningTotal;             // how many beats can fit into a minute? that's BPM!
      QS = true;                              // set Quantified Self flag
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }
  }

  // when the values are going down, the beat is over
  if (Signal < thresh && Pulse == true) 
  {    
    digitalWrite(blinkPin, LOW);           // turn off pin 13 LED
    Pulse = false;                         // reset the Pulse flag so we can do it again
    amp = P - T;                           // get amplitude of the pulse wave
    thresh = amp / 2 + T;                  // set thresh at 50% of the amplitude
    P = thresh;                            // reset these for next time
    T = thresh;
  }

  // if 2.5 seconds go by without a beat
  if (N > 2500) 
  {                            
    thresh = 512;                          // set thresh default
    P = 512;                               // set P default
    T = 512;                               // set T default
    lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date
    firstBeat = true;                      // set these to avoid noise
    secondBeat = true;                     // when we get the heartbeat back
  }
}




