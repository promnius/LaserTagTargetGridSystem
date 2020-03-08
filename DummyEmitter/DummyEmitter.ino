
#include <WS2812Serial.h>

int pinTransmit = 4;
int pinLEDDATA = 1;
static const int NUMRECIEVERS = 256;
static const int numled = 256; // do I really need two different numbers here??
byte recieverStatus[NUMRECIEVERS];
int affectedLED0 = 0;
int affectedLED1 = 0;
int affectedLED2 = 0;
int affectedLED3 = 0;

unsigned long lngDataRequestStartTime = 0;
unsigned long lngDataRequestEndTime = 0;
boolean readbackActive = false;
int displayCounter = 0;
int dataByteCounter = 0;
byte myByte = 0;

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

void setup() {
  // put your setup code here, to run once:
  pinMode(pinTransmit, OUTPUT);

  analogWriteResolution(8);
  analogWriteFrequency(pinTransmit, 38000);
  analogWrite(pinTransmit, 127);

  Serial2.begin(9600); // we'll see how fast we can go.
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10000);
  requestVisualData();
  mapRecieverDataToLEDs(); // the data that comes back is in 8x8 chunks, but the leds are not mapped like this.
  // also, each reciever, being inbetween the leds, actually maps to 4 leds.
  leds.show();
  fadeRecieverData(); // cause reciever data to dim over time, a sort of 'fade' effect will happen on the LEDs.
  
}

// sends a daisy serial request for visual data, waits for data to come back, and processes the data.
// note THIS FUNCTION IS BLOCKING. it also has no error handling right now, so if it never gets back
// a stop byte, it will hang forever.
void requestVisualData(){
  lngDataRequestStartTime = micros();
  Serial2.write(0b11110000);
  readbackActive = true;
  displayCounter = 0;
  Serial.println("Sent request for data: waiting for return");
  while(readbackActive){ // keep looking for data until it all comes back
    // if we wanted timeout functionality, or ANY error handling (a single bad bit will totally break
    // everything right now), this is where it would go. Just add another end condition, and the proper 
    // way to handle it (probably just don't draw anything if data wasn't valid).
    if(Serial2.available()){ // just stall for time until another byte is available
      myByte = Serial2.read();
      if (myByte == 0xF0){ // stop byte, transmission over
        readbackActive = false;
        Serial.println("Got stop byte, transmission over");
      } else if (myByte == 0xF1){ // new message header
        Serial.println("Got header byte, new tile is sending data");
        Serial.print("Last tile sent: ");
        Serial.print(dataByteCounter);
        Serial.println(" Bytes of data. Should be 22.");
        dataByteCounter = 0;
      }else{ // assumed to be a data byte.
        // structure: each tile sends 22 data bytes, with the first 21 containing 3 elements with 2 bits each 
        // (2 reserved bits for special commands), and the 22 byte containing only the 64th element.
        if (dataByteCounter < 21){
          if ((B00000011 & myByte) > 0) {recieverStatus[displayCounter] = 255;}
          displayCounter ++;
          if ((B00001100 & myByte) > 0) {recieverStatus[displayCounter] = 255;}
          displayCounter ++;
          if ((B00110000 & myByte) > 0) {recieverStatus[displayCounter] = 255;}
          displayCounter ++;
        } else if (dataByteCounter == 21){
          if ((B00000011 & myByte) > 0) {recieverStatus[displayCounter] = 255;}
          displayCounter ++;
        } else {Serial.println("ERROR, DATA OVERFLOW");}
        dataByteCounter ++;
      }
    }  
  }
  lngDataRequestEndTime = micros();
  Serial.print("Exited loop. Number of sensors to respond: ");
  Serial.print(displayCounter); Serial.print(", should be: "); Serial.println(NUMRECIEVERS);
  Serial.print("Time to accumulate data: ");
  Serial.print(lngDataRequestEndTime - lngDataRequestStartTime); 
  Serial.println(" uS.");
  Serial.println();
}

// this enables the master unit to determine what if any valid laser tag codes have been
// picked up by the targets. it has not been implemented yet.
void requestValidCodes(){
  
}

// this function maps the 8x8 recievers (centered between 4 leds each) to the 16x16 leds.
void mapRecieverDataToLEDs(){
  for (int i = 0; i < numled*3; i++){
    drawingMemory[i] = 0; // wipe the old data so we can prepare the new
  }
}

// dimms each receiver value as a function of time . . . or crudely, as a function of how fast this gets called.
void fadeRecieverData(){
  for (int i = 0; i < NUMRECIEVERS; i ++){
    recieverStatus[i] = recieverStatus[i] - 1;
  }
}

// THIS IS DESTRUCTIVE!!
void gammaCorrect(){
  for (int i = 0; i < numled*3; i++){
    drawingMemory[i] = pgm_read_byte(&gamma8[drawingMemory[i]]);
  }
  for (int i = 0; i < NUMRECIEVERS; i++){
    panelNumber = i/64;
    recieverNumber  = i%64;
    // -------------------------------------------- THIS CODE IS WHAT CHANGES IF A DIFFERENT PANEL ARRANGEMENT IS USED -----------------------------
    affectedLED0 = i
    // -------------------------------------------- END CODE THAT CHANGES IF A DIFFERENT PANEL ARRANGEMENT IS USED ---------------------------------
    // < 0 means the LED doesn't exist, ie, the reciever is on the edge of the map and not all the LEDs around it are real.
    if (affectedLED0 > 0){drawingMemory[affectedLED0] = drawingMemory[affectedLED0]+(recieverStatus[i]/4);}
    if (affectedLED1 > 0){drawingMemory[affectedLED1] = drawingMemory[affectedLED1]+(recieverStatus[i]/4);}
    if (affectedLED2 > 0){drawingMemory[affectedLED2] = drawingMemory[affectedLED2]+(recieverStatus[i]/4);}
    if (affectedLED3 > 0){drawingMemory[affectedLED3] = drawingMemory[affectedLED3]+(recieverStatus[i]/4);}
  }
}
