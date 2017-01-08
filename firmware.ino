/*
 2 degree of freedom seat mover based on monster shield (https://www.sparkfun.com/products/10182)
 2017.01.08 Joao Coutinho https://b.joaoubaldo.com
*/


#define RV  1  // VNH2SP30 param
#define FW  2  // VNH2SP30 param

#define motLeft 1  // Left motor index
#define motRight 0  // Right motor index

#define potL A4  // Left potentiometer pin
#define potR A5  // Right potentiometer pin

int shortDist = 40;
int mediumDist = 70;

int pwmMax = 250;  // Max speed for both motors
float pwmLow = 0.8;  // Percent of speed for short distances
float pwmMid = 0.9;  // Percent of speed for medium distances

boolean revL = false;  // Reverse left potentiometer values
int potMiniL=100;  // Left pot minimum value
int potMaxiL=850;  // Left pot maximum value

const boolean revR = true;  // Reverse right potentiometer values
int potMiniR=100;  // Right pot minimum value
int potMaxiR=900;  // Right pot minimum value

int toleration = 20;  // dead zone - if motors need to move less or equal than this value, they don't move.


/* Configuration end */


/* VNH2SP30 pin definitions */
int inApin[2] = {
  7, 4}; // INA: Clockwise input
int inBpin[2] = {
  8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {
  5, 6}; // PWM input
int cspin[2] = {
  2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {
  0, 1}; // EN: Status of switches output (Analog pin)
int statpin = 13; //not explained by Sparkfun

int DataValueL=0;
int DataValueR=0;

void setup() {
  Serial.begin(115200);

  pinMode(statpin, OUTPUT); //not explained by Sparkfun
  digitalWrite(statpin, LOW);

  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }

  // Stop motors
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  
  center_motors();
}

void loop() {
  byte Data[3] = {'0', '0', '0'};

  if (Serial.available() > 2) {
    Data[0]=Serial.read();
    if (Data[0]=='R'){
      Data[1]=Serial.read();
      Data[2]=Serial.read();
      DataValueR=NormalizeData(Data, potMiniR, potMaxiR, revR);
    }
    else if (Data[0]=='L') {
      Data[1]=Serial.read();
      Data[2]=Serial.read();
      DataValueL=NormalizeData(Data, potMiniL, potMaxiL, revL);
    }
    else if (Data[0]=='T') {
      Data[1]=Serial.read();
      Data[2]=Serial.read();
      if (Data[1]=='e' && Data[2]=='l') {
        Serial.print("Current: ");
        Serial.print(analogRead(potL));
        Serial.print(" R: ");
        Serial.println(analogRead(potR));

        Serial.print("L: ");
        Serial.print(potMiniL);
        Serial.print(" ");
        Serial.print(potMaxiL);
        Serial.print(" R: ");
        Serial.print(potMiniR);
        Serial.print(" ");
        Serial.println(potMaxiR);
      }
    }
  }
  else if (Serial.available() > 16)
    Serial.flush();

  motorMotion(motLeft, analogRead(potL), DataValueL, potMiniL, potMaxiL, toleration);
  delay(10);
  motorMotion(motRight, analogRead(potR), DataValueR, potMiniR, potMaxiR, toleration);
}

void center_motors() {
  byte l[3] = {'R', '7', 'F'};
  byte r[3] = {'R', '7', 'F'};

  DataValueL=NormalizeData(l, potMiniL, potMaxiL, revL);
  DataValueR=NormalizeData(r, potMiniR, potMaxiR, revR);
}

void motorMotion(int numMot,int actualPos,int targetPos, int potMini, int potMaxi, int tol) {
  int gap;
  int pwm;
  int brakingDistance=0;

  targetPos=constrain(targetPos,potMini+brakingDistance,potMaxi-brakingDistance);

  gap=abs(targetPos-actualPos);

  if (gap <= tol) {
    motorOff(numMot);
  }
  else {
    pwm=pwmMax;
    if (gap>shortDist) 
      pwm = (int)pwmMax*pwmLow;
    if (gap>mediumDist) 
      pwm = (int)pwmMax*pwmMid;

    if ((actualPos<potMini) || (actualPos<targetPos))
      motorGo(numMot, FW, pwm);
    else if ((actualPos>potMaxi) || (actualPos>targetPos))
      motorGo(numMot, RV, pwm);
  }
}

void motorOff(int motor) {
  digitalWrite(inApin[motor], LOW);
  digitalWrite(inBpin[motor], LOW);
  analogWrite(pwmpin[motor], 0);
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm) {
  if (motor <= 1)
  {
    if (direct <= 4)
    {
      // Set inA[motor]
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct== 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}

int NormalizeData(byte x[3], int potMini, int potMaxi, boolean rev) {
  int result;

  if (x[2]==13) //only a LSB and Carrier Return
  {
    x[2]=x[1]; //move MSB to LSB
    x[1]='0'; //clear MSB
  }
  for (int i=1; i<3; i++)
  {
    if (x[i]>47 && x[i]<58 ){//for xA to xF
      x[i]=x[i]-48;
    }
    if ( x[i]>64 && x[i]<71 ){//for x0 to x9
      x[i]=x[i]-55;
    }
  }

  if (rev)
    result=map(255-(x[1]*16+x[2]),0,255,potMini,potMaxi);
  else
    result=map((x[1]*16+x[2]),0,255,potMini,potMaxi);
    
  return result;
}
