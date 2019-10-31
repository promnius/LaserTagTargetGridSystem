
int TIMEOFF = 400; // time in uS
int TIMEZERO = 600;
int TIMEONE = 900;
int TIMEHEADER = 1200;

int pinIR = 3;

void setup() {
  // for teensy 3.2:
  // FTM0  drives pins 5, 6, 9, 10, 20, 21, 22, 23
  // FTM1  drives pins 3, 4
  // FTM2  drives pins 25, 32
  analogWriteFrequency(pinIR, 38000);

}

void loop() {
  // put your main code here, to run repeatedly:
  transmitDummyPacket();
  delay(2000);

}

void transmitDummyPacket(){
  pulseHeader();
  pulseOne();
  pulseZero();
  pulseOne();
  pulseZero();
  pulseOne();
  pulseZero();
  pulseOne();
  pulseZero();
  pulseOne();
  pulseZero();
  pulseOne();
  pulseOne();
}

void pulseOne(){
  analogWrite(pinIR, 128);
  delayMicroseconds(TIMEONE);
  analogWrite(pinIR, 0);
  delayMicroseconds(TIMEOFF);
}

void pulseZero(){
  analogWrite(pinIR, 128);
  delayMicroseconds(TIMEZERO);
  analogWrite(pinIR, 0);
  delayMicroseconds(TIMEOFF);
}

void pulseHeader(){
  analogWrite(pinIR, 128);
  delayMicroseconds(TIMEHEADER);
  analogWrite(pinIR, 0);
  delayMicroseconds(TIMEOFF);
}
