// ee572 control system
// mr gg am bc

#include <TimerOne.h>

#define IRPIN A9
#define MOTOR 6


#define IR_PIN    A9
#define IR_SAMPLE_PERIOD 1000 // ISR PERIOD IN MICROSECONDS
#define IR_SAMPLE_COUNT 200
float irSamples[IR_SAMPLE_COUNT];
unsigned long irSampleIndex = 0;
volatile unsigned long lastIrMinTimestamp = 0;
volatile unsigned long lastRotationPeriod = 0;
float currentRotationFreq = 0;

unsigned long lastFreqUpdate = 0;

// LED BLINK CONTROL VARIABLES
bool ledState = LOW;
unsigned long lastStrobeTime = 0; 
unsigned long strobeOffTime = 120; 
unsigned long strobeOnTime =  5;

double blinkFreq = 1.0 / ((strobeOnTime + strobeOffTime)/1000.0);

volatile bool started = 0;
volatile bool newFreq = 0;

int state = 0;
unsigned long lastLowTime = 0;
volatile unsigned long diffTime = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(A9, INPUT);
  pinMode(6, OUTPUT);

  Serial.begin(115200);
  delay(2000);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(checkSensor);
  //Timer1.attachInterrupt(sampleIRSensor);
  lastLowTime = millis();

  
  analogWrite(MOTOR, 255);
}

void checkSensor() {
  int val = analogRead(IRPIN) >> 5;
  if( state == 0 ){
    if( val < (600 >> 5) ){
      state = 1;
      //lastLowTime = millis();
    }
  } else if( state == 1) {
    if( val > (700>>5) ){
      lastLowTime = millis();
      state = 2;
    }
  } else if( state == 2) {
    if( val < (600>>5) ){
      diffTime = millis() - lastLowTime;
      state = 0;
    }
  }
}

void sampleIRSensor()
{
  static bool state = 0;
  uint16_t n = (irSampleIndex++) % 200;
  irSamples[n] = (map(analogRead(IR_PIN), 0, 1023, 0, 10000)-5000)/10000.0;

  // with ir in from -0.5-0.5, dIr_dt can range from -50 to 50 
  if( irSampleIndex > 0 ){
    // derivative 2 most recent IR readings


    float dirdt = (irSamples[n]-irSamples[n-1]) / (IR_SAMPLE_PERIOD * 0.000001);
    unsigned long rn = millis();

    //Serial.print("dirdt: ");
    //Serial.println(dirdt);

    // trigger once a negative slope above a threshold is reached
    // when a positive dirdt is found it will be at the valley of the signal dip
    switch( state ){
    case 0:
      if( dirdt < -50 ){
        state = 1;
      }
      break;
    case 1:
      if( dirdt > 0 ){
        lastRotationPeriod = rn - lastIrMinTimestamp;
        lastIrMinTimestamp = rn;
        state = 0;
        currentRotationFreq = 0.5 * (float)(1000.0/lastRotationPeriod);
        //Serial.println(currentRotationFreq);
        if( !started ) started = 1;
        //newFreq = 1;
      }
    } 
  }
}


void loop() {
// put your main code here, to run repeatedly:

//  

  

 if( millis() - 1000 > lastFreqUpdate ){

  if( Serial.available() ) analogWrite(MOTOR, Serial.parseInt());

//    noInterrupts();
//    double t = currentRotationFreq;
//    interrupts();
//    
//    Serial.print("current rot freq: ");
//    Serial.println(t);
    lastFreqUpdate = millis();
  unsigned long temp = 0;
  noInterrupts();
  temp = diffTime;
  interrupts();

  float rotFreq = 1.0/(2.0*(float)(diffTime)*0.001);
  float rpm = rotFreq * 60;
  Serial.println(rpm);
  }


  
}
