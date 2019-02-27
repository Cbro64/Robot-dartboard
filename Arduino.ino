//Limit pin variable setup
int L1H = 2;
int L1T = 3; 
int L2H = 4;
int L2T = 5;

//Relay pin variable setup
int rePin1 = 6;
//int rePin2 = 7;

//Variable setup
int safeMode = 1;

//Motor pin variable setup
int pulsePin = 12;
int dirPin = 10;
int currentPos = 0;

//Serial variable setup
byte rx_byte = 0;

ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
 {  
     if (safeMode == 1) {
      digitalWrite(rePin1, LOW);
      while(1)  {
            digitalWrite(13,HIGH);
            delay(200);
            digitalWrite(13,LOW);
            delay(200);
      }
     }
 }  

void setup() {
  //Starting serial communication
  Serial.begin(9600);
  
  //Limit pin setup
  pinMode(L1H, OUTPUT);
  pinMode(L1T, INPUT_PULLUP);
  pinMode(L2H, OUTPUT);
  pinMode(L2T, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  digitalWrite(L1H, LOW);
  digitalWrite(L2H, LOW);

  //Relay pin setup
  pinMode(rePin1, OUTPUT);
  //pinMode(rePin2, OUTPUT);
  digitalWrite(rePin1, HIGH);
  //digitalWrite(rePin2, HIGH);
  
  // Motor pin setup
  pinMode(pulsePin, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(11,LOW);
  digitalWrite(9,LOW);
  digitalWrite(dirPin,HIGH);

  //Interupt setup
  pciSetup(3);
  pciSetup(5);
}

void loop() {
  if (Serial.available()) {
  rx_byte = Serial.read();
  }

  if (rx_byte == 108)  {
    motorMove(-3000,100);
    delay(3000);
    motorMove(0,20);
    delay(100);
  }

  else if (rx_byte == 114)  {
    motorMove(3000,100);
    delay(3000);
    motorMove(0,20);
    delay(100);
  }
}

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void motorMove(int Pos, int Speed) {
  int i = 0;
  int dir = 1;
  int difPos = 0;
  int pulses = 0;
  int spd = 0;

  spd = map(Speed, 1, 100, 400, 3);

  difPos = Pos - currentPos;

  pulses = abs(difPos);

  if(difPos < 0) {dir = -1; digitalWrite(dirPin, LOW);}
  else {dir = 1; digitalWrite(dirPin, HIGH);}


  while(i<pulses)  {
    digitalWrite(pulsePin,HIGH);
    delayMicroseconds(spd);
    i += 1;
    currentPos = currentPos + dir;
    digitalWrite(pulsePin,LOW);
    delayMicroseconds(spd);
  }
  
  i = 0;
}
