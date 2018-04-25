// ee572 control system
// mr gg am bc

#include <TimerOne.h>
//#include <PID_v1.h>

#define IRPIN A9
#define MOTOR 6

PROGMEM float RPMMap[175] = {
/* 81 */ 187.36,
/* 82 */ 191.60,
/* 83 */ 207.32,
/* 84 */ 220.47,
/* 85 */ 228.35,
/* 86 */ 229.12,
/* 87 */ 247.24,
/* 88 */ 256.46,
/* 89 */ 268.24,
/* 90 */ 272.73,
/* 91 */ 273.73,
/* 92 */ 272.73,
/* 93 */ 295.95,
/* 94 */ 308.99,
/* 95 */ 322.32,
/* 96 */ 335.75,
/* 97 */ 349.69,
/* 98 */ 357.72,
/* 99 */ 372.74,
/* 100 */ 372.22,
/* 101 */ 377.39,
/* 102 */ 377.39,
/* 103 */ 386.16,
/* 104 */ 407.86,
/* 105 */ 406.53,
/* 106 */ 409.85,
/* 107 */ 424.58,
/* 108 */ 431.06,
/* 109 */ 430.43,
/* 110 */ 438.90,
/* 111 */ 461.69,
/* 112 */ 449.12,
/* 113 */ 449.80,
/* 114 */ 477.49,
/* 115 */ 478.49,
/* 116 */ 483.13,
/* 117 */ 491.09,
/* 118 */ 503.42,
/* 119 */ 493.44,
/* 120 */ 490.22,
/* 121 */ 500.99,
/* 122 */ 540.76,
/* 123 */ 545.49,
/* 124 */ 547.47,
/* 125 */ 546.46,
/* 126 */ 546.46,
/* 127 */ 547.47,
/* 128 */ 546.46,
/* 129 */ 547.47,
/* 130 */ 545.45,
/* 131 */ 546.46,
/* 132 */ 548.48,
/* 133 */ 546.46,
/* 134 */ 547.47,
/* 135 */ 545.45,
/* 136 */ 546.46,
/* 137 */ 546.46,
/* 138 */ 545.45,
/* 139 */ 545.45,
/* 140 */ 545.45,
/* 141 */ 587.50,
/* 142 */ 598.82,
/* 143 */ 594.12,
/* 144 */ 589.41,
/* 145 */ 610.10,
/* 146 */ 619.90,
/* 147 */ 638.41,
/* 148 */ 618.62,
/* 149 */ 619.90,
/* 150 */ 619.90,
/* 151 */ 614.80,
/* 152 */ 612.30,
/* 153 */ 613.52,
/* 154 */ 635.87,
/* 155 */ 646.62,
/* 156 */ 635.64,
/* 157 */ 659.74,
/* 158 */ 678.79,
/* 159 */ 672.73,
/* 160 */ 672.73,
/* 161 */ 660.87,
/* 162 */ 665.22,
/* 163 */ 672.73,
/* 164 */ 665.22,
/* 165 */ 666.73,
/* 166 */ 677.41,
/* 167 */ 707.64,
/* 168 */ 704.32,
/* 169 */ 699.41,
/* 170 */ 697.67,
/* 171 */ 702.66,
/* 172 */ 694.50,
/* 173 */ 702.66,
/* 174 */ 728.22,
/* 175 */ 729.97,
/* 176 */ 721.25,
/* 177 */ 721.25,
/* 178 */ 728.22,
/* 179 */ 735.37,
/* 180 */ 751.92,
/* 181 */ 742.68,
/* 182 */ 750.00,
/* 183 */ 748.17,
/* 184 */ 748.17,
/* 185 */ 740.85,
/* 186 */ 742.68,
/* 187 */ 742.68,
/* 188 */ 739.02,
/* 189 */ 742.68,
/* 190 */ 748.17,
/* 191 */ 746.34,
/* 192 */ 750.09,
/* 193 */ 755.77,
/* 194 */ 748.17,
/* 195 */ 763.46,
/* 196 */ 781.38,
/* 197 */ 775.30,
/* 198 */ 771.26,
/* 199 */ 769.23,
/* 200 */ 783.40,
/* 201 */ 775.30,
/* 202 */ 781.38,
/* 203 */ 785.43,
/* 204 */ 787.45,
/* 205 */ 789.58,
/* 206 */ 802.28,
/* 207 */ 791.61,
/* 208 */ 793.74,
/* 209 */ 791.61,
/* 210 */ 791.61,
/* 211 */ 785.43,
/* 212 */ 791.61,
/* 213 */ 808.68,
/* 214 */ 798.01,
/* 215 */ 795.87,
/* 216 */ 810.81,
/* 217 */ 808.68,
/* 218 */ 804.41,
/* 219 */ 798.01,
/* 220 */ 798.01,
/* 221 */ 806.54,
/* 222 */ 813.06,
/* 223 */ 806.54,
/* 224 */ 806.54,
/* 225 */ 804.41,
/* 226 */ 804.41,
/* 227 */ 808.68,
/* 228 */ 804.41,
/* 229 */ 798.01,
/* 230 */ 806.54,
/* 231 */ 819.82,
/* 232 */ 842.86,
/* 233 */ 840.48,
/* 234 */ 842.86,
/* 235 */ 842.86,
/* 236 */ 838.10,
/* 237 */ 838.10,
/* 238 */ 833.46,
/* 239 */ 845.24,
/* 240 */ 835.71,
/* 241 */ 847.62,
/* 242 */ 845.24,
/* 243 */ 842.86,
/* 244 */ 845.24,
/* 245 */ 847.62,
/* 246 */ 852.38,
/* 247 */ 857.14,
/* 248 */ 869.75,
/* 249 */ 867.23,
/* 250 */ 864.71,
/* 251 */ 864.71,
/* 252 */ 867.23,
/* 253 */ 867.23,
/* 254 */ 869.75,
/* 255 */ 879.83};


unsigned long lastFreqUpdate = 0;

float rpm = 0;
int state = 0;
unsigned long lastLowTime = 0;
volatile unsigned long diffTime = 0;

// PID PARAMETERS
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
//PID myPID(&Input, &Output, &Setpoint, .0375, .1486, .0024, DIRECT);
//float a0,a1,a2,b0,b1,b2,yk,yk_minus1,yk_minus2,wk,wk_minus1,wk_minus2;

float yk=0.0;
float yk_minus1=0.0;
float yk_minus2=0.0;
float wk=0.0;
float wk_minus1=0.0;
float wk_minus2=0.0;
float b0=1;
float b1=-0.4151;
float b2=0.0431;
float a0=1;
float a1=41.8514;
float a2=-42.2235;

void setup() {
  // put your setup code here, to run once:
  pinMode(A9, INPUT);
  pinMode(6, OUTPUT);

  Serial.begin(115200);
  delay(2000);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(checkSensor);
  lastLowTime = millis();

  
  analogWrite(MOTOR, 80);

  unsigned long n=millis();
  while(millis()-n < 1000){
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
  }


  //initialize the variables we're linked to
  Input = map(rpm,200,800,0,255);
  Setpoint = map(300,200,800,0,255); // target rpm=500

  

  //turn the PID on
//  myPID.SetMode(AUTOMATIC);
  
}

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
    wk=abs(Setpoint-yk);
    Serial.print("wk ");
    Serial.println(wk);
    yk=(b0*wk+b1*wk_minus1+b2*wk_minus2-a1*yk_minus1-a2*yk_minus2)/(a0);
    Serial.print("yk ");
    Serial.println(yk);
    Output=yk;

  if( millis() - 100 > lastFreqUpdate ){
    if( Serial.available() ){
      //analogWrite(MOTOR, Serial.parseInt());
      int v = Serial.parseInt();
      Setpoint = map(v,200,800,0,255);
      Serial.print("New setpoint(rpm):");
      Serial.println(v);
      Serial.print("New setpoint (mapped): ");
      Serial.println(Setpoint);
    }
  
    lastFreqUpdate = millis();
    unsigned long temp = 0;
    noInterrupts();
    temp = diffTime;
    interrupts();
  
    float rotFreq = 1.0/(2.0*(float)(diffTime)*0.001);
    rpm = rotFreq * 60;

    Serial.println();

    Serial.print("Target: ");
    Serial.println(Setpoint);
    Serial.print("Target rpm: ");
    Serial.println(map(Setpoint,0,255,200,800));
    
    Serial.print("Current rpm: ");
    Serial.println(rpm);
  
    //if( abs(Input-rpm) <10 ){
      Input = map(rpm,200,800,0,255);
    //}
    
    //double gap = abs(Setpoint-Input); //distance away from setpoint

    
//    myPID.Compute();

    double targetRPM = map(Output,0,255,200,800);

    Serial.print("new output: ");
    Serial.println(Output);
    Serial.print("mapped output: ");
    Serial.println(targetRPM);

    // find pwm associated with target rpm
    for( int i=0;i<174;i++ ){
      if( RPMMap[i] < targetRPM && RPMMap[i+1] > targetRPM ){
        analogWrite(MOTOR, i+80);
      }
    }
    
    wk_minus2=wk_minus1;
    Serial.print("wk_minus2: ");
    Serial.println(wk_minus2);
    wk_minus1=wk;
    Serial.print("wk_minus1: ");
    Serial.println(wk_minus1);
    yk_minus2=yk_minus1;
    Serial.print("yk_minus2: ");
    Serial.println(yk_minus2);
    yk_minus1=yk;
    Serial.print("yk_minus1: ");
    Serial.println(yk_minus1);
    delay(300);
  }

}
