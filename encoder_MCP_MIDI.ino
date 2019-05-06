#include <Wire.h>
#include "Adafruit_MCP23017.h"  // https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library
#include "MIDIUSB.h"


// #################### CONSTANTS ####################

// setup the port expander
Adafruit_MCP23017 mcp0;
Adafruit_MCP23017 mcp1;

#define addr0 0
#define addr1 1

boolean change=false;                // goes true when a change in the encoder state is detected
int butPress = 101;                  // stores which button has been pressed
int encSelect[3] = {101, 0, 101};    // stores the last encoder used and direction {encNo, 1=CW or 2=CCW, mcpX}
unsigned long currentTime;
unsigned long loopTime;

const int encCount0 = 8;  // number of rotary encoders
const int encCount1 = 1;  // number of rotary encoders

const int encPins0[encCount0][2] = {
  {0,1},   // enc:0 AA GPA0,GPA1 - pins 21/22 on mcp0
  {2,3},
  {4,5},
  {6,7},
  {8,9},    // GPB0,GPB1
  {10,11},
  {12,13},
  {14,15}
};  
const int encPins1[encCount1][2] = {
  {0,1}   // enc:0 AA GPA0,GPA1 - pins 21/22 on mcp1
};  


int encPos0[encCount0];
int encPos1[encCount1];

const int butCount0 = 0;  // number of buttons
const int butCount1 = 9;  // number of buttons
const int butPins0[butCount0] = {};
const int butPins1[butCount1] = { 2,3,4,5,6,7,8,9,10 };

// arrays to store the previous value of the encoders and buttons
unsigned char encoders0[encCount0];
unsigned char encoders1[encCount1];
unsigned char buttons0[butCount0];
unsigned char buttons1[butCount1];


// #################### FUNCTIONS ####################


// read the rotary encoder on pins X and Y, output saved in encSelect[encNo, direct]
unsigned char readEnc(Adafruit_MCP23017 mcpX, int X, const int *pin, unsigned char prev, int encNo) {

  unsigned char encA = mcpX.digitalRead(pin[0]);    // Read encoder pins
  unsigned char encB = mcpX.digitalRead(pin[1]);

  if((!encA) && (prev)) { 
    encSelect[0] = encNo;
    encSelect[2] = X;  // sets mcp number
    if(encB) {
      encSelect[1] = 1;  // clockwise
    }
    else {
      encSelect[1] = 2;  // counter-clockwise
    }
    change=true;
  }
  return encA;
}

// read the button on pin N. Change saved in butPress
unsigned char readBut(Adafruit_MCP23017 mcpX, const int pin, unsigned char prev, int encNo) {

  unsigned char butA = mcpX.digitalRead(pin);    // Read encoder pins
  //butA = digitalRead(
  if (butA != prev) {
    if (butA == HIGH) {
      butPress = encNo;
    }
  }
  return butA;  
}


void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}




// ######################## SETUP ########################

// setup the encoders as inputs. 
unsigned char encPinsSetup(Adafruit_MCP23017 mcpX, const int *pin) {
  mcpX.pinMode(pin[0], INPUT);  // A
  mcpX.pullUp(pin[0], HIGH);    // turn on a 100K pullup internally
  mcpX.pinMode(pin[1], INPUT);  // B
  mcpX.pullUp(pin[1], HIGH); 
}

// setup the push buttons
void butPinsSetup(Adafruit_MCP23017 mcpX, const int pin) {
  mcpX.pinMode(pin, INPUT);
  mcpX.pullUp(pin, HIGH); 
}

void setup() {  
  mcp0.begin(addr0);    // 0 = i2c address 0x20
  mcp1.begin(addr1);
  
  for (int n = 0; n < encCount0; n++) {           // setup the pins using loops, saves coding when you have a lot of encoders and buttons
    encPinsSetup(mcp0, encPins0[n]);
    encoders0[n] = 1;  // default state
  }
  for (int n = 0; n < butCount0; n++) {           // buttons and encoders are in separate arrays to allow for additional buttons
    butPinsSetup(mcp0, butPins0[n]);
    buttons0[n]  = 0;
  }

  for (int n = 0; n < encCount1; n++) {           // setup the pins using loops, saves coding when you have a lot of encoders and buttons
    encPinsSetup(mcp1, encPins1[n]);
    encoders1[n] = 1;  // default state
  }
  for (int n = 0; n < butCount1; n++) {           // buttons and encoders are in separate arrays to allow for additional buttons
    butPinsSetup(mcp1, butPins1[n]);
    buttons1[n]  = 0;
  }

  Serial.begin(9600); //for debugging
  //Serial.println("---------------------------------------");  
  //Serial.begin(115200);  // for midi

  currentTime = millis();
  loopTime = currentTime;
}







// ######################## MAIN ########################


void loop() {
  // check the encoders and buttons every 3 millis
  currentTime = millis();
  if(currentTime >= (loopTime + 3)){

    for (int n = 0; n < encCount1; n++) {
      encoders1[n] = readEnc(mcp1, 1, encPins1[n], encoders1[n],n);
    }
    for (int n = 0; n < butCount1; n++) {
      buttons1[n] = readBut(mcp1, butPins1[n], buttons1[n], n);
    }
    for (int n = 0; n < encCount0; n++) {
      encoders0[n] = readEnc(mcp0, 0, encPins0[n], encoders0[n],n);
    }
    for (int n = 0; n < butCount0 ; n++) {
      buttons0[n] = readBut(mcp0, butPins0[n], buttons0[n], n);
    }
    
    loopTime = currentTime;  // Updates loopTime
  } 
  
  
  if (change == true) {             // when an encoder has been rotated
    if (encSelect[0] < 100) {
      Serial.print("mcp: ");
      Serial.print(encSelect[2]); 
      Serial.print("  " );  
      Serial.print("Enc: ");
      Serial.print(encSelect[0]);  
      Serial.print(" " );  

      switch (encSelect[2]) {
        case (0): // mcp0
         switch (encSelect[1]) {
          case (1): // clockwise
            Serial.print("CW  ");
            encPos0[encSelect[0]] ++;
            Serial.println(encPos0[encSelect[0]]);
            break;
          case (2): // counter-clockwise
            Serial.print("CCW  ");
            encPos0[encSelect[0]] --;
            Serial.println(encPos0[encSelect[0]]);
            break;
         }
         break;
        case (1): // mcp1
         switch (encSelect[1]) {
          case (1): // clockwise
            Serial.print("CW  ");
            encPos1[encSelect[0]] ++;
            Serial.println(encPos1[encSelect[0]]);
            break;
          case (2): // counter-clockwise
            Serial.print("CCW  ");
            encPos1[encSelect[0]] --;
            Serial.println(encPos1[encSelect[0]]);
            break;
          }
          break;
       }




     
      if (encSelect[0] == 1) { // do something when a particular encoder has been rotated.
         //Serial.println("Encoder One has been used");
      }

      
      encSelect[0] = 101;     // set the selection to 101 now we have finished doing things. Not 0 as there is an encoder 0.
      encSelect[2] = 101;
    }
    change = false;       // ready for the next change
  }

  if (butPress < 100) {
    Serial.print("But: ");
    Serial.println(butPress); 

    butPress = 101; 
  }


}
