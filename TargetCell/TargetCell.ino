
// TODO STILL:
// packet validation code
// odds and ends, see all caps comments (timeout code, max bit duration code, etc.)
// accurate constants and pinouts
// comments
// debug print statements- make sure they don't interfere with the normal code!! Thanks Teensy for true USB comms!! Consider adding better debugging (see website).
// timing code to track how long the loop takes to execute

boolean Debug = true;

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
int pinANALOGSELECT0 = 1;
int pinANALOGSELECT1 = 1;
int pinANALOGSELECT2 = 1;
int pinsSENSORS[] = {1,1,1,1,1,1,1,1};

// PROGRAM CONSTANTS
unsigned int headerThreshold = 1200; // time in us required for a valid signal
unsigned int oneThreshold = 700;
unsigned int zeroThreshold = 400;

// GLOBAL CONSTANTS
int myByte = 0; 
int dataByte = 0;
int dataCounter = 0;
int nowSensorCounter = 0;
unsigned long nowTime = 0;
int nowState = 0;

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(1000000); // we'll see how fast we can go
  if (Debug){Serial.begin(9600);} // for debugging
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0; i < 8; i ++){
    setMux(i);
    sampleSensors(i);
  }
  

  while(Serial2.available()){
    myByte = Serial2.read();
    if (myByte == 0xF0){ // stop byte received
      printData();
      clearData();
      Serial2.print(myByte); // retransmit stop byte
    } else{
      Serial2.print(myByte); // simply pass through data
    }
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
  delayMicroseconds(5); // give settling/ capacitor charge shuffle equilization time.
}

// prints all interesting information in a condensed form up to the topside.
void printData(){
  Serial2.print(0xF1); // reserved header to say new cell start. Could someday also include a cell ID,
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
    Serial2.print(dataByte);
  }
  dataByte = 0x00;
  dataByte = dataByte | IRReceiverDisplayStatus[dataCounter];
  Serial2.print(dataByte);

  for (int i = 0; i < ValidCodesPointer; i ++){
    // maybe add another reserved byte here to make decoding slightly easier??
    Serial2.print(ValidCodes[i]); // we'll have to see if this formats correctly or not
  }
}

// clears any data registers that are sent topside, so after they are sent we won't be repeating ourselves.
// it is up to topside to maintain persistent data if desired. We only want to send the data again if our
// sensors pick it up again.
void clearData(){
  for (int i = 0; i < 64; i ++){
    IRReceiverDisplayStatus[i] = 0; // we've already sent any display info, if the sensors are still receiving data
    // we'll reset this and send it again, otherwise it is up to topside to do a faded or persistent animation
  }
  for (int i = 0; i < ValidCodeBufferLength; i ++){ // same deal here, if partial packets are formed, that info is preserved
    // in the IRReceiverDataPackets
    ValidCodes[i] = 0;
  }
  ValidCodesPointer = 0;
}

// this function grabs the current values for each pin on the micro for the current mux setting, then figures out
// if anything interesting is happening ie data recieved and what kind of data and valid packets, etc.
void sampleSensors(int channel){

  for (int counter = 0; counter < 8; counter ++){
    nowSensorCounter = channel + counter*8;
    nowTime = micros();
    nowState = digitalRead(pinsSENSORS[counter]);
  
    if (nowState != IRReceiverStates[nowSensorCounter]){ // data value has changed
      if (nowState == 0){ // data changed from 1 to 0, so end of a burst
        if (nowTime - IRReceiverLastTransition[nowSensorCounter] > headerThreshold){ // new packet forming
          // DO WE WANT TO CHECK FOR MAX TIMEOUT TOO??
          IRReceiverDataPackets[nowSensorCounter] = 0; // reset any existing packets. If they weren't complete, it's too late now!
          IRReceiverDataPointer[nowSensorCounter] = 0;
        }
        else if (nowTime - IRReceiverLastTransition[nowSensorCounter] > oneThreshold){
          IRReceiverDataPackets[nowSensorCounter] = IRReceiverDataPackets[nowSensorCounter] | (0x0001 << IRReceiverDataPointer[nowSensorCounter]);
          IRReceiverDataPointer[nowSensorCounter] ++;
        }
        else if (nowTime - IRReceiverLastTransition[nowSensorCounter] > zeroThreshold){
          // this is actually unnecessary because it is already 0 by default.
          IRReceiverDataPackets[nowSensorCounter] = IRReceiverDataPackets[nowSensorCounter] & (0xFFFE << IRReceiverDataPointer[nowSensorCounter]);
          IRReceiverDataPointer[nowSensorCounter] ++;
        }
        if (nowTime - IRReceiverLastTransition[nowSensorCounter] > zeroThreshold){ // this time NOT an else if . . . we just want to know if the data
          // was long enough to be considered more than a fluke (so debouncing), for the sake of display
          if (IRReceiverDisplayStatus[nowSensorCounter] < 1){ // upgrade the display if no data has been seen, as we've now seen data
            IRReceiverDisplayStatus[nowSensorCounter] = 1;
          }
        }
        if (IRReceiverDataPointer[nowSensorCounter] > 31){
          // THROW ERROR!!! we have exceeded our buffer . . . which also means we concatanated partial packets
          if (Debug){Serial.println("MAJOR ERROR, DATA OVERFLOW!!");}
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
  
}
