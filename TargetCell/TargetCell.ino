// TargetCell
// Laser tag high resolution grid of receivers project for blaster calibration and/or hit location information.
// code for a single sub unit of sensors.
// Kyle Mayer
// October 2019


// TODO STILL:
// &&packet validation code (something crude is implemented, but we need to identify what a real packet looks like)
// odds and ends, see all caps comments (timeout code, max bit duration code, etc.)

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

// PIN DECLARATIONS
int pinANALOGSELECT0 = 12;
int pinANALOGSELECT1 = 13;
int pinANALOGSELECT2 = 14;
int pinsSENSORS[] = {15,16,17,18,19,20,21,22};
int pinHeartbeat = 5;
//int pinHeartbeat = 13;

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


void setup() {
  pinMode(pinHeartbeat, OUTPUT);
  pinMode(pinANALOGSELECT0, OUTPUT);
  pinMode(pinANALOGSELECT1, OUTPUT);
  pinMode(pinANALOGSELECT2, OUTPUT);
 
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
