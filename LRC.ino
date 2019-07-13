#include <Wire.h>
//#include "Adafruit_MCP23017.h"  // https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library
#include "MIDIUSB.h"

// #################### VARIABLES ####################

int maxBanks = 9;
bool debug = false;
bool debugAll = false;
//const int functionEnc = 9;


// #################### CONSTANTS ####################

// MCP23017 registers (everything except direction defaults to 0)

#define IODIRA   0x00   // IO direction  (0 = output, 1 = input (Default))
#define IODIRB   0x01
#define IOPOLA   0x02   // IO polarity   (0 = normal, 1 = inverse)
#define IOPOLB   0x03
#define GPINTENA 0x04   // Interrupt on change (0 = disable, 1 = enable)
#define GPINTENB 0x05
#define DEFVALA  0x06   // Default comparison for interrupt on change (interrupts on opposite)
#define DEFVALB  0x07`
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

#define addr0 0x20
#define addr1 0x21

unsigned long data[3][4];

int MCP = 0;
int Encoder = 0;
int Direction = 0;
int Bank = 1;

int encValArray[3] = {0,0,0};
int encCCWArray[3] = {2,1,1};
int encCWArray[3] = {1,1,2};

const int encCount = 9;                 // number of rotary encoders
const int encPins[encCount][2] = {
  {0,1},  //1
  {16,17},//2
  {18,19},//3
  {20,21},//4
  {22,23},//5
  {24,25},//6
  {26,27},//7
  {28,29},//8
  {30,31} //9
};
const int selectorEncoder = 7;

const int butCount = 9;            // number of buttons on mcp1d
const int butPins[butCount] = { 2,8,9,10,11,12,13,14,15 };

const int ledCount = 9;  
const int ledPins[ledCount] = { 9,10,16,14,15,18,19,20,21 };

byte arduinoIntPin0 = 0;                                                                // Interrupts from the MCP will be handled by this PIN on Arduino
byte arduinoIntPin1 = 7;

byte arduinoInterrupt0 = digitalPinToInterrupt(arduinoIntPin0);                         // ... and this interrupt vector
byte arduinoInterrupt1 = digitalPinToInterrupt(arduinoIntPin1);

volatile boolean awakenByInterrupt = false;

int led = 17;


//int debounce = 30;




// #################### TOOLS ####################

void expanderWriteBoth(const byte reg, const byte data, unsigned char addrX) {
  Wire.beginTransmission (addrX);
  Wire.write (reg);   //
  Wire.write (data);  // port A
  Wire.write (data);  // port B
  Wire.endTransmission ();
} 

unsigned int expanderRead(const byte reg, unsigned char addrX) {     // read a byte from the expander
  Wire.beginTransmission (addrX);
  Wire.write (reg);
  Wire.endTransmission ();
  Wire.requestFrom (addrX, 1);
  return Wire.read();
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
  //midiEventPacket_t noteOff = {0x08 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

void sendMIDI(int Bank, int Encoder, int Direction) {
  if (debug == true){
    Serial.print("MIDI Data: B");
    Serial.print(Bank);
    Serial.print(" E");
    Serial.print(Encoder);
    Serial.print(" D");
    Serial.print(Direction);
    Serial.println();
  } else {
    controlChange(Bank, Encoder, Direction);
    MidiUSB.flush();
  }
}

void sendMIDIButton(int Bank, int Button){
  if (debug == false){
    noteOn(Bank, Button, 127);
    MidiUSB.flush();
    delay(50);
    noteOff(Bank, Button, 127);
    MidiUSB.flush();
  } else {
    Serial.print("Button: ");
    Serial.println(Button);
  }
}

void printBits2(long var) {
  for (unsigned long test = 0x80000000; test; test >>= 1) {
    if (test == 0x800000){Serial.print(" ");}
    if (test == 0x8000){Serial.print("  ");}
    if (test == 0x80){Serial.print(" ");}
    Serial.write(var  & test ? '1' : '0');
  }
}

void displayData(){

  unsigned long keyValue = data[0][3];
  unsigned long prevkeyValue = data[0][2];
  unsigned long changedkeyValue = data[1][3];
  unsigned long comparedkeyValue = data[2][3];

  Serial.println("DATA         MCP2b    MCP2a     MCP1b    MCP1a  ");
  Serial.print("keyvalue:   ");
  printBits2(keyValue);
  Serial.println();
  Serial.print("change:     "); 
  printBits2(changedkeyValue);
  Serial.println();
  Serial.print("comparison: ");
  printBits2(comparedkeyValue);
  Serial.println();


  for (int i = 0; i < 3; i++){ 
    for (int j = 0; j < 4; j++){ 
      Serial.print(data[i][j]);
    }
    Serial.println();
  }
}

void setled(int ledNumber){
  ledNumber = ledNumber - 1;
  if (debugAll == true){
    Serial.print("LED:");
    Serial.println(ledNumber);
  }
  for (int j = 0; j < ledCount; j++){ 
    digitalWrite(ledPins[j], LOW);
  }
  digitalWrite(ledPins[ledNumber], HIGH);
}


// ######################## SETUP ########################

unsigned char encPinsSetup() { // setup the encoders as inputs. 
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

void ledPinsSetup() {
  for (int j = 0; j < ledCount; j++){ 
    pinMode(ledPins[j], OUTPUT);
    digitalWrite(ledPins[j], LOW);
  }
}

void setup() {  
  Wire.begin ();  
  pinMode(arduinoIntPin0, INPUT);
  pinMode(arduinoIntPin1, INPUT);

  encPinsSetup();
  ledPinsSetup();
  setled(Bank);

  Serial.begin (115200);  //for debugging & MIDI
  
  //attachInterrupts();

  attachInterrupt(arduinoInterrupt0, intCallBack0, FALLING);                          // enable interrupts before going to sleep/wait
  attachInterrupt(arduinoInterrupt1, intCallBack1, FALLING);                          // And we setup a callback for the arduino INT handler.
}


// ######################## FUNCTIONS ########################

void intCallBack0() {
  if (awakenByInterrupt == false){    //only proceed if previous interrupt has finished processing
    awakenByInterrupt = true;
    //encSelect[0] = 0;
    MCP = 0;
  }
}
void intCallBack1() {
  if (awakenByInterrupt == false){    //only proceed if previous interrupt has finished processing
    awakenByInterrupt = true;
    //encSelect[0] = 1;
    MCP = 1;
  }
}

void detachInterrupts() {
  // disable interrupts while handling them.
  detachInterrupt(arduinoInterrupt0);
  detachInterrupt(arduinoInterrupt1);

  //delay(debounce);  // de-bounce before we re-enable interrupts

  //digitalWrite(led, !digitalRead(led));
}

void cleanInterrupts() {
  EIFR = 0x01;
  awakenByInterrupt = false;
}

void attachInterrupts() {
  attachInterrupt(arduinoInterrupt0, intCallBack0, FALLING);
  attachInterrupt(arduinoInterrupt1, intCallBack1, FALLING);
}

unsigned long getInteruptValue() {                   //Get pin from MCP that was interrupted 
  int addr;
  int encseladd;

  static unsigned long keyValue;
  static unsigned long prevkeyValue;
  static unsigned long changedkeyValue;

  if (MCP == 0){
    addr = addr0;
    encseladd=0;
  }  
  else if (MCP == 1){
    addr = addr1;
    encseladd=8;
  }
  keyValue = 0x00000000;
  if (expanderRead(INFTFA, addr))     // Read port values, as required. Note that this re-arms the interrupts.
    {
    keyValue &= 0xFF00;
    keyValue |= expanderRead(INTCAPA, addr);// << 8;    // read value at time of interrupt
    }
  if (expanderRead(INFTFB, addr))
    {
    keyValue &= 0x00FF;
    keyValue |= expanderRead(INTCAPB, addr) << 8;        // port B is in low-order byte
    }
  keyValue = keyValue << 16 * MCP;

  changedkeyValue = keyValue^prevkeyValue;

 /*
  if (debug == true){
    Serial.println("DATA         MCP2b    MCP2a     MCP1b    MCP1a  ");
    Serial.print("keyvalue:   ");
    printBits2(keyValue);
    Serial.println();
    Serial.print("change:     "); 
    printBits2(changedkeyValue);
    Serial.println();
    Serial.print("comparison: ");
    printBits2(keyValue&changedkeyValue);
    Serial.println();
  }
  */
  prevkeyValue = keyValue;

  return keyValue;
}

void appendtoData(unsigned long keyValue){           // appends keyvalue to data and returns inteerrupt pin value

  //keyValue = data[0];
  //changedkeyValue = data[1];
  //comparedkeyValue = data[2];

  static unsigned long keyValueArray[4];
  static unsigned long changedkeyValueArray[4];
  static unsigned long comparedkeyValueArray[4];
  static unsigned long prevkeyValue;

  //static int data[3][4];
  //int intpin;

  static unsigned long changedkeyValue;
  static unsigned long comparedkeyValue;

  changedkeyValue = keyValue^prevkeyValue;
  comparedkeyValue = keyValue&changedkeyValue;

  /*
  if (debug == true){
    Serial.println("DATA         MCP2b    MCP2a     MCP1b    MCP1a  ");
    Serial.print("keyvalue:   ");
    printBits2(keyValue);
    Serial.println();
    Serial.print("change:     "); 
    printBits2(changedkeyValue);
    Serial.println();
    Serial.print("comparison: ");
    printBits2(comparedkeyValue);
    Serial.println();
  }
  */
  /*
  if ((keyValue&changedkeyValue)%2 == 0) {                     //if pin number is even
    Serial.println("CW");
  } else {
    Serial.println("CCW");
  }
  */

  keyValueArray[0] = keyValueArray[1];
  keyValueArray[1] = keyValueArray[2];
  keyValueArray[2] = keyValueArray[3];
  keyValueArray[3] = keyValue;                            // shift array and append

  changedkeyValueArray[0] = changedkeyValueArray[1];
  changedkeyValueArray[1] = changedkeyValueArray[2];
  changedkeyValueArray[2] = changedkeyValueArray[3];                         //shift array and append
  changedkeyValueArray[3] = changedkeyValue;

  comparedkeyValueArray[0] = comparedkeyValueArray[1];
  comparedkeyValueArray[1] = comparedkeyValueArray[2];
  comparedkeyValueArray[2] = comparedkeyValueArray[3];
  comparedkeyValueArray[3] = comparedkeyValue;


  for (int i = 0; i < 4; i++){ 
    data[0][i] = keyValueArray[i];
  }
  for (int i = 0; i < 4; i++){ 
    data[1][i] = changedkeyValueArray[i];
  } 
  for (int i = 0; i < 4; i++){ 
    data[2][i] = comparedkeyValueArray[i];
  } 

  prevkeyValue = keyValue;
}

int decodeInterruptValue(){   //converts interrupt value to to pin numbers. returns -1 if value=0
  long changedkeyValue;
  int intpin;
  changedkeyValue = data[1][3];
  long one = 0x00000001;
  byte localpinNumber = 0;
  if (changedkeyValue != 0){                           // if there is a value attached to the interrupt
    for (localpinNumber = 0; localpinNumber < 32; localpinNumber++){
      if (changedkeyValue & (one << localpinNumber)) {          //bitwyse keyvalue == binary value of button
        intpin = localpinNumber;
        //if (debug == true){Serial.println(intpin);}
      }
    }
  } else {
    return -1;
  }

  return intpin;
}

int encoderNumber(int interruptPin){
  for (int i = 0; i < encCount; i++){ 
    for (int j = 0; j < 2; j++){ 
      if (encPins[i][j] == interruptPin){
        return i + 1;
      }
    }
  }
  return 0;
}

int buttonNumber(int interruptPin){
  for (int i = 0; i < butCount; i++){ 
    if (butPins[i] == interruptPin){
      return i + 1;
    }
  }
  return 0;
}

bool isSelector(int Encoder){
  if (Encoder == selectorEncoder){
    return true;
  } else {
    return false;
  }
  return false;
}

bool isButton(int interruptPin){
  for (int j = 0; j < butCount; j++){ 
    if (butPins[j] == interruptPin){
      return true;
    }
  }
  return false;
}

int encoderDirection(){          // 0 = null   1 = CW     -1 = CCW

  if ((data[2][2] == 0) && (data[2][3] == 0)){
    if (data[2][0] < data[2][1]){
      return 1;
    } else {
      return -1;
    }
  } else {
    //Serial.println("BAD READING");
    return 0;
  }
}


// ################# MAIN ########################

void loop() {
  if (awakenByInterrupt){
    detachInterrupts();
    unsigned long interruptValue = getInteruptValue();
    appendtoData(interruptValue);
    int interruptPin = decodeInterruptValue();
    int Encoder = encoderNumber(interruptPin);
    
    if (debugAll == true){displayData();}
    //Serial.println(Encoder);
    //Serial.println(encDirection);
    
    if (isButton(interruptPin) == true){  //if interrupt pin is not recognized as button pin
      int Button = buttonNumber(interruptPin);
      sendMIDIButton(Bank, Button);
    } else {
      int encDirection = encoderDirection();
      if ((encDirection != 0) && (Encoder != 0)) {
        if (isSelector(Encoder) == true){
          Bank = Bank + encDirection;
          if (Bank > maxBanks){
            Bank = maxBanks;
          }
          if (Bank < 1){
            Bank = 1;
          }
          setled(Bank);
          if (debug == true){
            Serial.print("Bank: ");
            Serial.println(Bank);
          }
        } else {
          sendMIDI(Bank, Encoder, ((encDirection-1)*-63)+1);  // converts: -1/1 -> 1/127
        }
      } else if (((encDirection = 0) && (Encoder != 0)) || ((encDirection != 0) && (Encoder = 0))){
        Serial.println("BAD READING");
      } else {
        //Serial.println("Skipping Check");
      }
    }



    cleanInterrupts();
    attachInterrupts();

  }
  //delay(50);                                //Maximum delay acceptable for normal and reactive encoder response time without loose events
}
