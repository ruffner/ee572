// EE572 control system
// Matt Ruffner
// Alex Mueller
// Ben Cummins
// Galvin Greene

#include <TimerOne.h>

#define IRPIN A9
#define MOTOR 6

// MOTOR PWM VALUES WERE ITERATED THROUGH, WITH THE CORRESPONDING RESULTING 
// RPM VALUE LOGGED. THESE VALUES WERE LOADED INTO MATLAB AND FIT WITH A 
// POLYNOMIAL. THE FOLLOWING ARRAY IS THE POLYNOMIAL EVALUATED AT VARIOUS 
// PWM VALUES TO GET A SMOOTHER SPEED CURVE.

PROGMEM float RPMMap[175] = {
2.847108e+02,2.918626e+02,2.989418e+02,3.059491e+02,3.128849e+02,
3.197497e+02,3.265440e+02,3.332684e+02,3.399233e+02,3.465093e+02,
3.530267e+02,3.594762e+02,3.658583e+02,3.721733e+02,3.784219e+02,
3.846046e+02,3.907218e+02,3.967740e+02,4.027618e+02,4.086856e+02,
4.145460e+02,4.203434e+02,4.260784e+02,4.317514e+02,4.373630e+02,
4.429136e+02,4.484038e+02,4.538341e+02,4.592049e+02,4.645168e+02,
4.697702e+02,4.749657e+02,4.801038e+02,4.851849e+02,4.902096e+02,
4.951784e+02,5.000917e+02,5.049501e+02,5.097541e+02,5.145042e+02,
5.192008e+02,5.238446e+02,5.284359e+02,5.329753e+02,5.374632e+02,
5.419003e+02,5.462870e+02,5.506237e+02,5.549110e+02,5.591495e+02,
5.633395e+02,5.674816e+02,5.715763e+02,5.756241e+02,5.796255e+02,
5.835810e+02,5.874911e+02,5.913564e+02,5.951772e+02,5.989541e+02,
6.026877e+02,6.063783e+02,6.100266e+02,6.136330e+02,6.171979e+02,
6.207221e+02,6.242058e+02,6.276496e+02,6.310541e+02,6.344197e+02,
6.377469e+02,6.410363e+02,6.442882e+02,6.475033e+02,6.506821e+02,
6.538249e+02,6.569324e+02,6.600051e+02,6.630433e+02,6.660478e+02,
6.690188e+02,6.719570e+02,6.748628e+02,6.777368e+02,6.805794e+02,
6.833912e+02,6.861726e+02,6.889242e+02,6.916464e+02,6.943398e+02,
6.970049e+02,6.996421e+02,7.022519e+02,7.048349e+02,7.073916e+02,
7.099224e+02,7.124279e+02,7.149086e+02,7.173649e+02,7.197974e+02,
7.222066e+02,7.245930e+02,7.269570e+02,7.292992e+02,7.316201e+02,
7.339201e+02,7.361999e+02,7.384598e+02,7.407004e+02,7.429222e+02,
7.451256e+02,7.473113e+02,7.494796e+02,7.516312e+02,7.537664e+02,
7.558858e+02,7.579900e+02,7.600793e+02,7.621543e+02,7.642155e+02,
7.662634e+02,7.682985e+02,7.703214e+02,7.723324e+02,7.743322e+02,
7.763211e+02,7.782998e+02,7.802686e+02,7.822282e+02,7.841790e+02,
7.861216e+02,7.880563e+02,7.899838e+02,7.919044e+02,7.938189e+02,
7.957275e+02,7.976308e+02,7.995294e+02,8.014237e+02,8.033143e+02,
8.052016e+02,8.070861e+02,8.089683e+02,8.108488e+02,8.127280e+02,
8.146065e+02,8.164847e+02,8.183632e+02,8.202424e+02,8.221228e+02,
8.240050e+02,8.258894e+02,8.277766e+02,8.296671e+02,8.315613e+02,
8.334597e+02,8.353629e+02,8.372714e+02,8.391856e+02,8.411061e+02,
8.430334e+02,8.449679e+02,8.469102e+02,8.488608e+02,8.508201e+02,
8.527887e+02,8.547670e+02,8.567557e+02,8.587551e+02,8.607658e+02,
8.627883e+02,8.648230e+02,8.668706e+02,8.689314e+02,8.710060e+02
};

// VARS FOR DETERMINING CURRENT RPM
unsigned long lastFreqUpdate = 0;
float rpm = 0;
int state = 0;
unsigned long lastLowTime = 0;
volatile unsigned long diffTime = 0;

// CONTROL PARAMETER
double Setpoint;

// PID STATE VARS
float yk=0.0;
float yk_minus1=0.0;
float yk_minus2=0.0;
float wk=0.0;
float wk_minus1=0.0;
float wk_minus2=0.0;

// PID TF COEFFICIENTS
float b0=0.9842;
float b1=-1.1461;
float b2=0.3337;
float a0=1.0;
float a1=-1.0;
float a2=0.0;

void setup() {
  // PIN DIRECTION DEFINITIONS
  pinMode(IRPIN, INPUT);
  pinMode(MOTOR, OUTPUT);
  pinMode(13, OUTPUT); // LED

  Serial.begin(115200);
  delay(2000);

  // RPM SENSING SETUP
  // 1 MILLISECOND SAMPLING PERIOD REQUIRED 
  // TO GET ACCURATE RPM SENSING
  Timer1.initialize(1000);
  // ISR CALLBACK
  Timer1.attachInterrupt(checkSensor);
  // TIMESTAMP FOR RPM SENSING
  lastLowTime = millis();

  // INITIAL LOW MOTOR SPEED
  analogWrite(MOTOR, 80);

  unsigned long n=millis();
  while(millis()-n < 1000){
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }

  // INITIAL TARGET RPM OF 300
  Setpoint = 300;

  wk=rpm;
}

// 3 STATE THRESHOLD BASED RPM SENSING FOR
// ANALOG REFLECTANCE SENSOR
void checkSensor() {
  int val = analogRead(IRPIN) >> 5;
  if( state == 0 ){
    if( val < (600 >> 5) ){
      state = 1;
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


void loop() {
  
  // RUN CODE INSIDE THIS IF EVERY 100 MILLISECONDS (Ts)
  if( millis() - 100 > lastFreqUpdate ){
    if( Serial.available() ){
      Setpoint = Serial.parseInt();
    }
    
    // UPDATE TIMESTAMP TO KEEP TRACK OF Ts
    lastFreqUpdate = millis();
    unsigned long temp = 0;
    // ATOMIC READ OF VOLATILE VARIABLE INSIDE ISR
    noInterrupts();
    temp = diffTime;
    interrupts();

    // COMPUTE CURRENT RPM
    rpm = 1.0/(2.0*(float)(diffTime)*0.001) * 60;
    
    // COMPUTE THE OUTPUT
    yk=(b0*wk+b1*wk_minus1+b2*wk_minus2-a1*yk_minus1-a2*yk_minus2)/(a0);

    // SET THE INPUT
    wk=Setpoint-rpm;
       
    double targetRPM = yk;

    // TAB DELIMITTED SERIAL LOGGING FOR PLOTTING IN MATLAB
    Serial.print(Setpoint);Serial.print("\t");
    Serial.print(rpm);Serial.print("\t");
    Serial.print(wk);Serial.print("\t");
    Serial.println(yk);


    // LOOKUP TARGET RPM
    for( int i=0;i<174;i++ ){
      if( RPMMap[i] <= targetRPM && RPMMap[i+1] > targetRPM ){
        // DRIVE MOTOR AT NEW SPEED
        analogWrite(MOTOR, i+81);
      }
    }

    // UPDATE INPUT/OUTPUT STATE VARIABLES
    wk_minus2=wk_minus1;
    wk_minus1=wk;
    yk_minus2=yk_minus1;
    yk_minus1=yk;
  }

}
