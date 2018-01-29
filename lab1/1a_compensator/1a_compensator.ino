// matt ruffner
// lab 1 
// ee572 spring 2018

// update period in milliseconds
unsigned long cyclePeriodMS = 10;

// use ones since no initial values given
float a0=1, b0=1, a1=1, b1=1;

// previous and current in/out states
float yk=0.0,   wk=0.0;
float ykm1=0.0, wkm1=0.0;

// input and output pins
byte inputPin  = A0; // analogRead()
byte outputPin = 10; // PWM analogWrite()

unsigned long lastStart = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Serial.print("Starting program");

  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);
}

void loop() {
  // save cycle start time
  lastStart = millis();

  // rescale out 10 bit ADC reading to 0-5 volts
  wk = map((float)analogRead(inputPin),0,1023,0,5.0);
  yk = (1.0/a0)*(wk*b0 + wkm1*b1 - ykm1*a1);

  // output current state after rescaling for 8 bit DAC
  analogWrite(outputPin, map(yk,0,5.0,0,255));

  // save current states
  wkm1 = wk;
  ykm1 = yk;

  // delay until time to start next cycle
  while( millis() - lastStart < cyclePeriodMS );
}
