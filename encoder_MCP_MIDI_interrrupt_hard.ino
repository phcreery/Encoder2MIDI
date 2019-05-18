#include <Wire.h>
//#include "Adafruit_MCP23017.h"  // https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library
#include "MIDIUSB.h"


// #################### CONSTANTS ####################

// MCP23017 registers (everything except direction defaults to 0)

#define IODIRA   0x00   // IO direction  (0 = output, 1 = input (Default))
#define IODIRB   0x01
#define IOPOLA   0x02   // IO polarity   (0 = normal, 1 = inverse)
#define IOPOLB   0x03
#define GPINTENA 0x04   // Interrupt on change (0 = disable, 1 = enable)
#define GPINTENB 0x05
#define DEFVALA  0x06   // Default comparison for interrupt on change (interrupts on opposite)
#define DEFVALB  0x07
#define INTCONA  0x08   // Interrupt control (0 = interrupt on change from previous, 1 = interrupt on change from DEFVAL)
#define INTCONB  0x09
#define IOCON    0x0A   // IO Configuration: bank/mirror/seqop/disslw/haen/odr/intpol/notimp
//#define IOCON 0x0B  // same as 0x0A
#define GPPUA    0x0C   // Pull-up resistor (0 = disabled, 1 = enabled)
#define GPPUB    0x0D
#define INFTFA   0x0E   // Interrupt flag (read only) : (0 = no interrupt, 1 = pin caused interrupt)
#define INFTFB   0x0F
#define INTCAPA  0x10   // Interrupt capture (read only) : value of GPIO at time of last interrupt
#define INTCAPB  0x11
#define GPIOA    0x12   // Port value. Write to change, read to obtain value
#define GPIOB    0x13
#define OLLATA   0x14   // Output latch. Write to latch output.
#define OLLATB   0x15

int addr = 0x00;
#define addr0 0x20
#define addr1 0x21

//unsigned int keyValue = 0x00000000;
unsigned long keyValue0 = 0;
unsigned long keyValue1 = 0;

//boolean change=false;                // goes true when a change in the encoder state is detected
int butPress = 101;                    // stores which button has been pressed
int encSelect[3] = {101, 101, 0};      // stores the last encoder used and direction {mcpX, encNo, 1=CW or 127=CCW}
int encValArray[3] = {0,0,0};
int encCWArray[3] = {127,1,1};
bool encCWArrayComp = false;
int encCCWArray[3] = {1,1,127};
bool encCCWArrayComp = false;


int encseladd = 0;
int numbuttons = 0;

const int encCount = 9;  // number of rotary encoders

int encPos[encCount];

const int butCount0 = 0;            // number of buttons on mcp0
const int butCount1 = 9;            // number of buttons on mcp1
const int butPins0[butCount0] = {};
const int butPins1[butCount1] = { 2,3,4,5,6,7,8,9,10 };

// Interrupts from the MCP will be handled by this PIN on Arduino
byte arduinoIntPin0 = 0;
byte arduinoIntPin1 = 7;
// ... and this interrupt vector
byte arduinoInterrupt0 = digitalPinToInterrupt(arduinoIntPin0);
byte arduinoInterrupt1 = digitalPinToInterrupt(arduinoIntPin1);
//byte arduinoInterrupt = digitalPinToInterrupt(arduinoIntPin);

volatile boolean awakenByInterrupt0 = false;
volatile boolean awakenByInterrupt1 = false;

int led = 17;

int minval = 1;
int maxval = 127;

int debounce = 30;


// #################### FUNCTIONS ####################



void expanderWriteBoth(const byte reg, const byte data, unsigned char addrX) {
  Wire.beginTransmission (addrX);
  Wire.write (reg);   //
  Wire.write (data);  // port A
  Wire.write (data);  // port B
  Wire.endTransmission ();
} // end of expanderWrite

// read a byte from the expander
unsigned int expanderRead(const byte reg, unsigned char addrX) {
  Wire.beginTransmission (addrX);
  Wire.write (reg);
  Wire.endTransmission ();
  Wire.requestFrom (addrX, 1);
  return Wire.read();
} // end of expanderRead



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
  //midiEventPacket_t noteOff = {0x08 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}




// ######################## SETUP ########################

// setup the encoders as inputs. 
unsigned char encPinsSetup() {
  // expander configuration register
  expanderWriteBoth (IOCON, 0b01100000, addr0); // mirror interrupts, disable sequential mode
 
  // enable pull-up on switches
  expanderWriteBoth (GPPUA, 0xFF, addr0);   // pull-up resistor for switch - both ports

  // invert polarity
  expanderWriteBoth (IOPOLA, 0xFF, addr0);  // invert polarity of signal - both ports
  
  // enable all interrupts
  expanderWriteBoth (GPINTENA, 0xFF, addr0); // enable interrupts - both ports

  expanderRead(INTCAPA, addr0);
  expanderRead(INTCAPB, addr0);


  expanderWriteBoth (IOCON, 0b01100000, addr1); // mirror interrupts, disable sequential mode
 
  // enable pull-up on switches
  expanderWriteBoth (GPPUA, 0xFF, addr1);   // pull-up resistor for switch - both ports

  // invert polarity
  expanderWriteBoth (IOPOLA, 0xFF, addr1);  // invert polarity of signal - both ports
  
  // enable all interrupts
  expanderWriteBoth (GPINTENA, 0xFF, addr1); // enable interrupts - both ports

  expanderRead(INTCAPA, addr1);
  expanderRead(INTCAPB, addr1);
  
}


void setup() {  
  Wire.begin ();  
  pinMode(arduinoIntPin0, INPUT);
  pinMode(arduinoIntPin1, INPUT);

  encPinsSetup();

  Serial.begin (115200);  //for debugging
  //Serial.println("---------------------------------------");  
  //Serial.begin(115200);  // for midi

  //currentTime = millis();
  //loopTime = currentTime;
  

  // enable interrupts before going to sleep/wait
  // And we setup a callback for the arduino INT handler.
  attachInterrupt(arduinoInterrupt0, intCallBack0, FALLING);
  attachInterrupt(arduinoInterrupt1, intCallBack1, FALLING);
}







// ######################## MAIN ########################

void intCallBack0() {
  awakenByInterrupt0 = true;
  encSelect[0] = 0;
}
void intCallBack1() {
  awakenByInterrupt0 = true;
  encSelect[0] = 1;
}


void handleInterrupt() {
  // disable interrupts while handling them.
  detachInterrupt(arduinoInterrupt0);
  detachInterrupt(arduinoInterrupt1);

  //delay(debounce);  // de-bounce before we re-enable interrupts

  digitalWrite(led, !digitalRead(led));
  // check the encoders and buttons every 1 millis for debounce
  
  if (encSelect[0] == 0){
    addr = addr0;
    encseladd=0;
    numbuttons=32;
    //encSelect[0] = 0;
  }  
  else if (encSelect[0] == 1){
    addr = addr1;
    encseladd=5;
    numbuttons=16;
    //encSelect[0] = 1;
  }
  
  
  
  if (expanderRead(INFTFA, addr))            // Read port values, as required. Note that this re-arms the interrupts.
    {
    keyValue0 &= 0x00FF;
    keyValue0 |= expanderRead(INTCAPA, addr) << 8;    // read value at time of interrupt
    }
  if (expanderRead(INFTFB, addr))
    {
    keyValue0 &= 0xFF00;
    keyValue0 |= expanderRead(INTCAPB, addr);        // port B is in low-order byte
    }

  //Serial.println(keyValue0);
  if (keyValue0 != 0){
    //Serial.print("processing...: ");
    for (byte button = 0; button < 32; button++){
      // this key down?
      if (keyValue0 & (1 << button)) { //bitwyse keyvalue == binary value of button
        if ( button%2 == 0 ) {
          //Serial.print("CW ");
          //encSelect[1] = 1;
          encSelect[1] = round(button/2-0.01) + encseladd;
          //encPos[encSelect[2]] ++;                                                      //absolute
          //encPos[encSelect[2]] = constrain(encPos[encSelect[2]], minval, maxval);       //absolute constrain
          //encPos[encSelect[2]] = 1;                                                       //twos compiment
          encValArray[0] = encValArray[1];
          encValArray[1] = encValArray[2];
          encValArray[2] = 1;                                                              // shift array and append 1
          //for(int i = 0; i < 3; i++){
          //  Serial.print(encValArray[i]);
          //}
          
          break;
        } else {
          //Serial.print("CCW ");
          //encSelect[1] = 2;
          encSelect[1] = round(button/2-0.01) + encseladd;
          //encPos[encSelect[2]] --;                                                      //absolute
          //encPos[encSelect[2]] = constrain(encPos[encSelect[2]], minval, maxval);       //absolute boundaries
          //encPos[encSelect[2]] = 127;                                                   //twos compiment
          encValArray[0] = encValArray[1];
          encValArray[1] = encValArray[2];
          encValArray[2] = 127;                                                              //shift array and append 127
          //for(int i = 0; i < 3; i++){
          //  Serial.print(encValArray[i]);
          //}
            //print(encValArray);
          break;
        }
      } else {
      
      } // end of for each button
    }
  } else {
    decodeData();                                           // check data on every 4th interrupt. IDK why but the 4th interrupt has no value: keyValue = 0
    handleMidi();

    for(int i = 0; i < 3; i++){
      encValArray[i] = 0;
      //encValArray[i] = 0;
    }
    

    
  }
  
 
  cleanInterrupts();
  keyValue0 = 0;
  
  attachInterrupt(arduinoInterrupt0, intCallBack0, FALLING);
  attachInterrupt(arduinoInterrupt1, intCallBack1, FALLING);
}


void decodeData(){
  encCWArrayComp = true;
  encCCWArrayComp = true;

  for (int i = 0;  i < 3; i++) {
    if( encValArray[i] != encCWArray[i] ) {
      //Serial.println("not equal");
      encCWArrayComp = false;
      break;
    }
  }

  for (int i = 0;  i < 3; i++) {
    if( encValArray[i] != encCCWArray[i] ) {
      //Serial.println("not equal");
      encCCWArrayComp = false;
      break;
    }
  }

  if (encCWArrayComp == true) {
    //Serial.print("## CW ##################################");
    encSelect[2] = 1;
  } else if (encCCWArrayComp == true){
    //Serial.print("## CCW ---------------------------------");
    encSelect[2] = 127;
  } 
}



void cleanInterrupts() {
  EIFR = 0x01;
  awakenByInterrupt0 = false;
  awakenByInterrupt1 = false;
}


void handleMidi() {
  controlChange(1, encSelect[1], encSelect[2]);
  MidiUSB.flush();
}
  


void loop() {
  if (awakenByInterrupt0){
    //Serial.println("Interrupted...");
    handleInterrupt();
    
    //handleMidi();
    //Serial.println();
    //Serial.print(encSelect[2]);
    //Serial.print("  ");
    //Serial.println(encPos[encSelect[2]]);
    //Serial.println("Handled");
  }

  //delay(50);                                //Maximum delay acceptable for normal and reactive encoder response time without loose events
}
