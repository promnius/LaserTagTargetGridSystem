// TargetCell
// Laser tag high resolution grid of receivers project for blaster calibration and/or hit location information.
// code for a single sub unit of sensors.
// Kyle Mayer
// October 2019


// TODO STILL:
// &&packet validation code (something crude is implemented, but we need to identify what a real packet looks like)
// odds and ends, see all caps comments: timeout code (on an unfinished message) for improved error handling, max bit duration code (more timeouts), etc.
// Support for non-teensy drivers: other than going slower, may need to move the serial stuff to a new port, and handle it in interrupts (teensy
    // does some dark magic to handle uarts). maybe an esp32 is a good candidate? ATMEGA328, but may be slow? or an STM32? need something fast-ish and cheap, no
    // other real hardware needs.
    
#include <WS2812Serial.h>
#define RED    0x160000
#define GREEN  0x001600
#define BLUE   0x000016
#define YELLOW 0x101400
#define PINK   0x120009
#define ORANGE 0x100400
#define WHITE  0x101010

int Debug = 0; // 3 is most useful. 4 is verbose. 5 is max. 2 is kind of broken without any delays. any debugging kind of breaks real time, therefore packet decoding is broken.

// SENSOR VARIABLE DECLARATION
boolean IRReceiverStates[64]; // what state is the reciever in, so next time we sample it, we will know if it changed
unsigned long IRReceiverLastTransition[64]; // when is the time stamp of the last change, so if it changes again we'll know how long it was on
unsigned long IRReceiverDataPackets[64]; // when we get a valid 0 or 1, we add it to the data packet for that sensor. We use a crude data type 
// for storing this for simplicity, but this limits packet length to 32 bytes
int IRReceiverDataPointer[64]; // where in the packet are we adding characters?
const int ValidCodeBufferLength = 20;
unsigned long ValidCodes[ValidCodeBufferLength]; // this puts a hard cap on how many valid codes we can store, but since we should be polled frequently, this is not too important
int ValidCodesPointer = 0;
byte IRReceiverDisplayStatus[64]; // for each sensor, if we were going to create a display representing the last unit of time, what would that look like?
// 0 for no data, 1 for data but no valid packets, 2 for valid but incomplete packets, 3 for complete packets, etc. Could create codes for all sorts of info
// like team, damage, player who hit it, etc. but 0-3 compresses well for sending topside.
int recieverLEDStatus[64];
int LEDDisplay[256]; // There are a lot of duplicated registers here/ intermediate registers used only for math . . . this is just a mapped version of recieverLEDStatus, broken
// out for the indevidual LEDs. It should be identical to the drawing buffers used by the LED library.

// PIN DECLARATIONS
int pinANALOGSELECT0 = 12;
int pinANALOGSELECT1 = 13;
int pinANALOGSELECT2 = 14;
int pinsSENSORS[] = {15,16,17,18,19,20,21,22};
int pinHeartbeat = 5;
int pinLEDDATA = 1;

// PROGRAM CONSTANTS
unsigned int headerThreshold = 1200; // time in us required for a valid signal
unsigned int oneThreshold = 700;
unsigned int zeroThreshold = 400;

// GLOBAL VARIABLES
int myByte = 0; 
int dataByte = 0;
int dataCounter = 0;
int nowSensorCounter = 0;
unsigned long nowTime = 0;
int nowState = 0;
boolean booHeartbeatState = LOW;

// TIMING VARIABLES
unsigned long lngScanTime = 0;
unsigned long lngTimeStart = 0;
unsigned long lngScanEndTime = 0;
unsigned long lngCommsEndTime = 0;
unsigned long lngCommsTime = 0;
unsigned long lngHeartbeatTimer = 0;
unsigned long lngCommsStartTime = 0;

static const int numled = 256; // do I really need two different numbers here??
// FANCY VARIABLES FOR LEDs
byte rawDrawingMemory[numled*3];         //  3 bytes per LED, for doing non-destructive math before gamma correction
byte drawingMemory[numled*3];         //  3 bytes per LED, actual data mask
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED, unpacked data mask for non-blocking serial print
WS2812Serial leds(numled, displayMemory, drawingMemory, pinLEDDATA, WS2812_RGB);

const uint8_t PROGMEM gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

// There is probably a better, faster way of doing this using linear transforms . . . 
// but a look up table is dumb and simple and works and will work for any crazy layout you 
// want.
const uint8_t PROGMEM LEDMapping[] = {
     0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0,
     8, 8, 9, 9,10,10,11,11,12,12,13,13,14,14,15,15,15,15,14,14,13,13,12,12,11,11,10,10, 9, 9, 8, 8,
    16,16,17,17,18,18,19,19,20,20,21,21,22,22,23,23,23,23,22,22,21,21,20,20,19,19,18,18,17,17,16,16,
    24,24,25,25,26,26,27,27,28,28,29,29,30,30,31,31,31,31,30,30,29,29,28,28,27,27,26,26,25,25,24,24,
    32,32,33,33,34,34,35,35,36,36,37,37,38,38,39,39,39,39,38,38,37,37,36,36,35,35,34,34,33,33,32,32,
    40,40,41,41,42,42,43,43,44,44,45,45,46,46,47,47,47,47,46,46,45,45,44,44,43,43,42,42,41,41,40,40,
    48,48,49,49,50,50,51,51,52,52,53,53,54,54,55,55,55,55,54,54,53,53,52,52,51,51,50,50,49,49,48,48,
    56,56,57,57,58,58,59,59,60,60,61,61,62,62,63,63,63,63,62,62,61,61,60,60,59,59,58,58,57,57,56,56};

int maxBrightnessScalar = 4;

IntervalTimer myTimer;

int selfContained = 1;

void setup() {
  pinMode(pinHeartbeat, OUTPUT);
  pinMode(pinANALOGSELECT0, OUTPUT);
  pinMode(pinANALOGSELECT1, OUTPUT);
  pinMode(pinANALOGSELECT2, OUTPUT);

  leds.begin();
  int microsec = 1500000 / leds.numPixels();
  colorWipe(RED, microsec);
  leds.clear();
  leds.show();
 
  Serial2.begin(1000000); // we'll see how fast we can go
  if (Debug>0){Serial.begin(9600);Serial.print("Setup Finished. Debug level: "); Serial.println(Debug);} // for debugging
  lngHeartbeatTimer = millis();

  delay(100);
  for (int counter = 0; counter < 8; counter ++){
    pinMode(pinsSENSORS[counter], INPUT);
    delay(100);
    if (Debug>3){
      Serial.print("Sensor pin: ");
      Serial.println(pinsSENSORS[counter]);
      Serial.print("Sensor Status: ");
      Serial.println(digitalRead(pinsSENSORS[counter]));}
  }
  delay(100);
  
  myTimer.priority(200); // lower than most timers so micros and millis still work.
  myTimer.begin(updateLEDs, 30000); // 30hz
}

void loop() {
  
  if (millis() - lngHeartbeatTimer > 1000){
    booHeartbeatState = !booHeartbeatState;
    digitalWrite(pinHeartbeat, booHeartbeatState);
    lngHeartbeatTimer = millis();
    printDisplayStatus();
    
  }
  
  if(Debug>1){Serial.println("Loop, Scanning Sensors");}
  if(Debug>2){delay(3000);}
  lngTimeStart = micros();
  for(int i = 0; i < 8; i ++){
    setMux(i);
    sampleSensors(i);
  }
  lngScanEndTime = micros();
  lngScanTime = lngScanEndTime - lngTimeStart;
  if(Debug>1){Serial.print("Time to scan sensors: "); Serial.print(lngScanTime); Serial.println(" uS.");}

  
  // check comms status, handle requests if need be
  // THIS SECTION MAY NEED SOME TIMOUT HANDLING OR SERIAL BUFFER MODIFICATION, AND OF COURSE THIS CAN
  // BREAK WITH DEBUGGING ON.
  lngCommsStartTime = micros();
  while(Serial2.available()){
    myByte = Serial2.read();
    //Serial.println(myByte);
    if (myByte == 0xF0){ // stop byte (visual data request) received
      if(Debug>1){Serial.println("Stop byte for visual data request received, Dumping data.");}
      printVisualData();
      clearMyVisualData();
      Serial2.write(myByte); // retransmit stop byte
    } else if(myByte == 0xF7){ // reserved for stop byte (valid code request), not implemented yet.
      
    }else{
      if(Debug>1){Serial.print("Retransmitting Message: "); Serial.println(myByte, BIN);}
      Serial2.write(myByte); // simply pass through data
    }
  }

  lngCommsEndTime = micros();
  lngCommsTime = lngCommsEndTime - lngCommsStartTime;

  if(Debug>1){Serial.print("Time to handle Comms requests: "); Serial.print(lngCommsTime); Serial.println(" uS.");}
}

void updateLEDs(){
  for (int i = 0; i < 64; i ++){
    if (recieverLEDStatus[i] > 0){recieverLEDStatus[i] = recieverLEDStatus[i] - 1;}
  }
  for (int i = 0; i < 64; i ++){
    if (IRReceiverDisplayStatus[i] > 0){recieverLEDStatus[i] = 255;}
  }
  mapLEDsToSensors();
  for (int i = 0; i < 256; i++){ // num LED
    drawingMemory[i*3] = LEDDisplay[i]; //MAPPEDLEDIntensity[i];
  }
  clearMyVisualData();
  gammaCorrect();
  leds.show();
}

// ------------------------------ THIS IS THE ONLY FUNCTION THAT NEEDS TO CHANGE TO UPDATE THE WAY LEDS MAP ---------------------------
// this could include using a new panel, or it could include adding antialiasing. The mapping can be done via lookup table or via math,
// although antialiasing may be trickier via look up table
void mapLEDsToSensors(){
  for (int i = 0; i < 256; i++){
    LEDDisplay[i] = recieverLEDStatus[pgm_read_byte(&LEDMapping[i])];
    //((i/4)%8)
    //LEDdisplay[i] = recieverLEDStatus
  }
}

void gammaCorrect(){
  for (int i = 0; i < numled*3; i++){
    drawingMemory[i] = drawingMemory[i] / maxBrightnessScalar;
    drawingMemory[i] = pgm_read_byte(&gamma8[drawingMemory[i]]);
  }
}

// allows a single function call for setting a mux channel. Note: this function currently
// only supports a single group of muxes (all using the same select lines).
// Note there are OOP libraries for analog muxes but this code is really simple.
// channel input is 0-7
void setMux(int channel){
  // if we are worried about speed, these digital writes need to be replaced. (and from hardware, pin selects
  // need to end up on one port!)
  if (channel%2 == 1){digitalWrite(pinANALOGSELECT0, HIGH);} else{digitalWrite(pinANALOGSELECT0, LOW);}
  if ((channel/2)%2 == 1){digitalWrite(pinANALOGSELECT1, HIGH);} else{digitalWrite(pinANALOGSELECT1, LOW);}
  if ((channel/4)%2 == 1){digitalWrite(pinANALOGSELECT2, HIGH);} else{digitalWrite(pinANALOGSELECT2, LOW);}
  // This hardware is only for an 8 to 1 mux
  //if ((channel/8)%2 == 1){digitalWrite(pinANALOGSELECT3, HIGH);} else{digitalWrite(pinANALOGSELECT3, LOW);}
  delayMicroseconds(8); // give settling/ capacitor charge shuffle equilization time.
}

// prints all interesting information in a condensed form up to the topside.
void printVisualData(){
  Serial2.write(0xF1); // reserved header to say new cell start. Could someday also include a cell ID,
  // but for now we don't have code for dynamic cell ID config.
  dataCounter = 0;
  for (int i = 0; i < 21; i ++){ // 22 is a magic number for 64 sensors, each with 2 bits of data, sending only
    // 6 out of 8 bits per byte since 1 bit is needed for reserved bytes and data packes better into even numbers
    // (we could transmit 9 bit words, but for now this is sufficient), this requires 22 bytes.
    dataByte = 0x00;
    dataByte = dataByte | IRReceiverDisplayStatus[dataCounter];
    dataCounter ++;
    dataByte = dataByte | IRReceiverDisplayStatus[dataCounter] << 2;
    dataCounter ++;
    dataByte = dataByte | IRReceiverDisplayStatus[dataCounter] << 4;
    dataCounter ++;
    Serial2.write(dataByte);
  }
  dataByte = 0x00;
  dataByte = dataByte | IRReceiverDisplayStatus[dataCounter];
  Serial2.write(dataByte);

  /* THIS WILL NOW BE HANDLED IN A SEPARATE CALL, WHERE A SEPARATE RESERVED CODE IS SENT TO REQUEST THE VALID CODES.
   *  THIS LETS ME FINISH DEVELOPMENT WITHOUT HAVING TO IMPLEMENT OR TEST THIS.
  for (int i = 0; i < ValidCodesPointer; i ++){
    // maybe add another reserved byte here to make decoding slightly easier??
    Serial2.print(ValidCodes[i]); // we'll have to see if this formats correctly or not
  }
  */
}

// clears any data registers that are sent topside, so after they are sent we won't be repeating ourselves.
// it is up to topside to maintain persistent data if desired. We only want to send the data again if our
// sensors pick it up again.
void clearMyVisualData(){
  if(Debug>1){Serial.println("Clearing Visual Data!");}
  if(Debug>1){Serial.println("Last Display Frame:");}
  printDisplayStatus();
  
  for (int i = 0; i < 64; i ++){
    //if(Debug>1){Serial.print(IRReceiverDisplayStatus[i]);Serial.print(",");}
    //if(Debug>1){if(((i+1)%8)==0){Serial.println();}}
    IRReceiverDisplayStatus[i] = 0; // we've already sent any display info, if the sensors are still receiving data
    // we'll reset this and send it again, otherwise it is up to topside to do a faded or persistent animation
  } 

  /* THIS IS NOW TO BE HANDLED SEPARATELY
  for (int i = 0; i < ValidCodeBufferLength; i ++){ // same deal here, if partial packets are formed, that info is preserved
    // in the IRReceiverDataPackets
    ValidCodes[i] = 0;
  }
  ValidCodesPointer = 0;*/ 
}

// this function grabs the current values for each pin on the micro for the current mux setting, then figures out
// if anything interesting is happening ie data recieved and what kind of data and valid packets, etc.
void sampleSensors(int channel){
  for (int counter = 0; counter < 8; counter ++){
    nowSensorCounter = channel + counter*8;
    nowTime = micros();
    nowState = digitalRead(pinsSENSORS[counter]);
    nowState = !nowState; // sensors idle high and are active low, but my logic was for the inverse.
    if (nowState == 1 && Debug>4){Serial.print("Sensor: "); Serial.print(channel);Serial.print(",");Serial.print(nowSensorCounter);Serial.println(" is a 1");}
    if (nowState == 0 && Debug>4){Serial.print("Sensor: "); Serial.print(channel);Serial.print(",");Serial.print(nowSensorCounter);Serial.println(" is a 0");}
    if (nowState != IRReceiverStates[nowSensorCounter]){ // data value has changed
      if (nowState == 0){ // data changed from 1 to 0, so end of a burst
        if (nowTime - IRReceiverLastTransition[nowSensorCounter] > headerThreshold){ // new packet forming
          // DO WE WANT TO CHECK FOR MAX TIMEOUT TOO??
          IRReceiverDataPackets[nowSensorCounter] = 0; // reset any existing packets. If they weren't complete, it's too late now!
          IRReceiverDataPointer[nowSensorCounter] = 0;
          if(Debug>3){Serial.print("Sensor "); Serial.print(nowSensorCounter); Serial.println(" Detected a Header");}
          
        }
        else if (nowTime - IRReceiverLastTransition[nowSensorCounter] > oneThreshold){
          IRReceiverDataPackets[nowSensorCounter] = IRReceiverDataPackets[nowSensorCounter] | (0x0001 << IRReceiverDataPointer[nowSensorCounter]);
          IRReceiverDataPointer[nowSensorCounter] ++;
          if(Debug>3){Serial.print("Sensor "); Serial.print(nowSensorCounter); Serial.println(" Detected a One");}
        }
        else if (nowTime - IRReceiverLastTransition[nowSensorCounter] > zeroThreshold){
          // this is actually unnecessary because it is already 0 by default.
          IRReceiverDataPackets[nowSensorCounter] = IRReceiverDataPackets[nowSensorCounter] & (0xFFFE << IRReceiverDataPointer[nowSensorCounter]);
          IRReceiverDataPointer[nowSensorCounter] ++;
          if(Debug>3){Serial.print("Sensor "); Serial.print(nowSensorCounter); Serial.println(" Detected a Zero");}
        }
        if (nowTime - IRReceiverLastTransition[nowSensorCounter] > zeroThreshold){ // this time NOT an else if . . . we just want to know if the data
          // was long enough to be considered more than a fluke (so debouncing), for the sake of display
          if (IRReceiverDisplayStatus[nowSensorCounter] < 1){ // upgrade the display if no data has been seen, as we've now seen data
            IRReceiverDisplayStatus[nowSensorCounter] = 1;
          }
        }
        if(Debug>4){Serial.print("New Data packet: "); Serial.println(IRReceiverDataPackets[nowSensorCounter]);}
        if (IRReceiverDataPointer[nowSensorCounter] > 31){
          // THROW ERROR!!! we have exceeded our buffer . . . which also means we concatanated partial packets
          if (Debug>1){Serial.println("MAJOR ERROR, DATA OVERFLOW!!");}
          IRReceiverDataPointer[nowSensorCounter] = 0;
        }
        attemptDataPacketValidation(nowSensorCounter);
      }
      IRReceiverLastTransition[nowSensorCounter] = nowTime;
      IRReceiverStates[nowSensorCounter] = nowState;
    }

    // CHECK FOR TIMEOUTS?? ie, clear data if nothing has happened in a long time? or just trust that we'll get a header before the next packet?

  }
}

// this function will check the IRReceiverDataPacket for a single sensor, determine if it is a valid packet or not,
// and update the appropriate display and log registers if it IS a valid packet. The only time a packet could be valid (if it wasn't before)
// is if we just received a new bit, so no reason to check more often than that.
// it will also clear out the appropriate registers if the packet was valid, so we don't keep re-evaluating it as valid.
void attemptDataPacketValidation(int sensor){
  if(Debug>4){Serial.println("Searching for valid codes:");}
  // there will be many more ways to look at this, especially if I have variable packet length or 
  // partial packet decoding
  if (IRReceiverDataPointer[sensor] == 12){
    // at some point we would also want a checksum placed in here!!
    if(Debug>4){Serial.println("Valid code found!");}
    if(Debug>4){Serial.println(IRReceiverDataPackets[sensor]);}
    ValidCodes[ValidCodesPointer] = IRReceiverDataPackets[sensor];
    ValidCodesPointer ++;
    if (ValidCodesPointer > (ValidCodeBufferLength - 1)){
      if(Debug>1){Serial.println("ERROR!!! OVERFLOW!!");}
      ValidCodesPointer = 0;
    }
    // NOTE: WE DON'T clear the data buffer right now, because this may have been a partial packet.
    // only a header bit can clear the data packet.
  } else{ if(Debug>4){Serial.println("No valid codes found.");}}
}

void printDisplayStatus(){
  Serial.println();
  for (int counter1 = 0; counter1 < 8; counter1 ++){
    for (int counter2 = 7; counter2 >= 0; counter2 --){
      if (IRReceiverDisplayStatus[counter1*8+counter2] > 0){Serial.print("X");} else{Serial.print("O");}
      Serial.print(",");      
    }
    Serial.println();
  } 
  Serial.println();
  
}

void colorWipe(int color, int wait) {
  for (int i=0; i < leds.numPixels(); i++) {
    leds.setPixel(i, color);
    leds.show();
    delayMicroseconds(wait);
  }
}
