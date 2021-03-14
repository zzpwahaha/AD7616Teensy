/****************************************************************************************
 * This is the firmware for the teensy 3.1/3.2 that takes command from serial interface 
 * and program the ADF4159 IC. 
 * The teensy micrcontroller also generates the necessary data for ramping the frequency.
 *  
 * The control command takes the following form. 
 * (ch, start freq, stop freq, ramp step number, ramp length)
 * 
 * range for input data:
 * Ch:0,1
 * Start freq: 1000-6000(MHz)  If start freq = stop freq then the ramp is ignored.
 * Stop freq:1000-6000(MHz)    If start freq = stop freq then the ramp is ignored.
 * Ramp step number: <2^32     Must be a positive integer
 * Ramp length: in ms.         Minimum step time is limited to 20us.
 *                             i.e. step length/step munber >20us
 *              
 * Anything out of range, the control will return an error.
 * Nothing will be updated. 
 * If the command is entered sucessfully, the control will give the echo of the command.
 * During power on, the controller programs the ADF4159 to the last defult frequency.
 ***************************************************************************************/

//Including necessary libraries 
#include <Arduino.h>
#include <SPI.h>
#include <EEPROMex.h>
#include <stdio.h>
#include <NativeEthernet.h>

//Function declaration
void pfdInit();
uint32_t calcFTW(uint32_t freq);
void updatePFD(uint32_t FTW, int LE);

bool dataCheck(uint32_t par[5]);
void setRamp0();
void setRamp1();

//Variable declaration


elapsedMicros rampDelay0;  //ramp delay timer0
elapsedMicros rampDelay1;  //ramp delay timer1



uint32_t stepCount0;  //ramp step counter for ch0
uint32_t stepCount1;  //ramp step counter for ch1

uint32_t rampflg0;  //ramp start/stop flag
uint32_t rampflg1;  //ramp start/stop flag

uint32_t rampCounter0;  //number of uploaded ramp in ch0
uint32_t rampCounter1;  //number of uploaded ramp in ch1





/*******************************************************************************************************************/
#define AD7616DEBUG
const char STARTM = '(';        // start marker for each set of data
const char SEPAR = ',';         // separator within one set of data
const char ENDM = ')';          // end marker for each set of data
const short MAXIN = 600;        // maximum number of chars in input stream
const short MAXSEQ = MAXIN / 6; // maximum number of sequences

unsigned char seqChannelA[MAXSEQ][8];    // measured channel index(0-7) in sequencer for channel A, the size is stored in seqChannelSize
unsigned char seqChannelB[MAXSEQ][8];    // measured channel index(0-7) in sequencer for channel B, the size is stored in seqChannelSize
unsigned char seqChannelSize[MAXSEQ][2];  // size of the measured channel in A and B respesctively
unsigned long seqRepNumber[MAXSEQ];       // number of repeat measurement in sequencer   

short seqTotal = 0;
short seqCounter = 0;
bool seqRunning = false; 
bool dummyRead = false;

EXTMEM short ADCDATA[4000000];
unsigned int DATASIZE = 0; 

const short RESET = 9;
const short CS = 10;      // out
const short SCLK = 13;    // out, only for initializing clock to be high in initDAC, since we are using mode2 of SPI. Maybe not nessary
const short CONVST = 15;  // out
const short BUSY = 14;    // in 
const short TRIG = 16;    // in


const SPISettings Wsetting(50000000, MSBFIRST, SPI_MODE3);
const SPISettings Rsetting(50000000, MSBFIRST, SPI_MODE3);
const short configRegAddr   = 0b0000100;
const short channRegAddr    = 0b0000110;
const short rangeRegAddr[]  = {0b0001000, 0b0001010, 0b0001100, 0b0001110}; /*A1,A2,B1,B2*/
const short rangeRegValue[] = {0x0000, 0x0000, 0x0000, 0x0000};//{0x00aa, 0x00bb, 0x00cc, 0x00dd}; // right now the values are for test
const short dummyReadAddr   = (1<<9);

elapsedMicros timer;
/******************ETHERNET CONNECTION****************************/
byte mac[] = {0x04,0xe9,0xe5,0x0e,0x0c,0xe0};
IPAddress ip(10,10,0,10);
EthernetServer server(80);

bool debugMode(const String& rc);

void interpretCmd();

void initADC();
void readADC();
void dataReady();
void dummyReadADC();

void sendADCDATA();

void initTCP();
void listenForEthernetClients();

void resetSeqTEENSY();
void writeSeq(unsigned short index);

void initADC()
{
  digitalWrite(SCLK, HIGH); //prepare it for the SPI, just to be safe
  digitalWrite(RESET, LOW);
  delay(1); //keep low for at least 1.2us to set a full reset
  digitalWrite(RESET, HIGH);
  delay(100); //settling time minimum 15ms

  /*initialize range register for A1(3-0),A2(7-4),B1(3-0),B2(7-4)*/
  SPI.beginTransaction(Rsetting);
  digitalWrite(CS,LOW);
  SPI.transfer16((1<<15)+(configRegAddr<<8) + 0b01100000); //enable burst and sequence, disable oversampling and CRC
  digitalWrite(CS, HIGH);  

  for (short i = 0; i < 4; i++)
  {
    digitalWrite(CS,LOW);
    SPI.transfer16((1<<15)+(rangeRegAddr[i]<<8) + rangeRegValue[i]); //set sampling range for all 16 channels
    digitalWrite(CS, HIGH);  
  }
  SPI.endTransaction();

}

void readADC()
{
  for (unsigned int repts = 0; repts < seqRepNumber[seqCounter]; repts++)
  {
    seqRunning = true;
    digitalWrite(CONVST,HIGH);
    delayNanoseconds(200);
    digitalWrite(CONVST,LOW);
    while (seqRunning) {}
  }
  seqCounter++;
  if (seqCounter<seqTotal){
    writeSeq(seqCounter);
  }
}

void dummyReadADC()
{
  dummyRead = true;
  digitalWrite(CONVST,HIGH);
  delayNanoseconds(200);
  digitalWrite(CONVST,LOW);
}

void dataReady()
{
  if (dummyRead) {
    SPI.beginTransaction(Rsetting);
    digitalWrite(CS,LOW);
    SPI.transfer16(dummyReadAddr); // D9=1 is the reserved addr which does not correspond to anything
    SPI.transfer16(dummyReadAddr); // D9=1 is the reserved addr which does not correspond to anything
    digitalWrite(CS,HIGH);
    SPI.beginTransaction(Rsetting);
    dummyRead = false;
    return;
  }

  // Serial.printf("Total time: %u \r\n", static_cast<unsigned long>(timer) );
  unsigned short dataA[seqChannelSize[seqCounter][0]];
  unsigned short dataB[seqChannelSize[seqCounter][1]];
  unsigned char minSize = min(seqChannelSize[seqCounter][0],seqChannelSize[seqCounter][1]);
  unsigned char maxSize = max(seqChannelSize[seqCounter][0],seqChannelSize[seqCounter][1]);
  SPI.beginTransaction(Rsetting);
  for (unsigned char i = 0; i < minSize; i++)
  {
    digitalWrite(CS,LOW);
    dataA[i] = SPI.transfer16(dummyReadAddr); // D9=1 is the reserved addr which does not correspond to anything
    dataB[i] = SPI.transfer16(dummyReadAddr); // D9=1 is the reserved addr which does not correspond to anything
    digitalWrite(CS,HIGH);
  }
  if (minSize==maxSize){}
  else if (minSize == seqChannelSize[seqCounter][0]){ /*read channel B*/
    for (unsigned char i = minSize; i < maxSize; i++)
    {
      digitalWrite(CS,LOW);
      SPI.transfer16(dummyReadAddr); 
      dataB[i] = SPI.transfer16(dummyReadAddr); 
      digitalWrite(CS,HIGH);
    }
  }
  else if (minSize == seqChannelSize[seqCounter][1]){ /*read channel A*/
    for (unsigned char i = minSize; i < maxSize; i++)
    {
      digitalWrite(CS,LOW);
      dataA[i] = SPI.transfer16(dummyReadAddr); 
      SPI.transfer16(dummyReadAddr); 
      digitalWrite(CS,HIGH);
    }
  }

  /*transfer data to external mem*/

  memcpy(ADCDATA + DATASIZE, dataA, sizeof(dataA));
  DATASIZE += sizeof(dataA);
  memcpy(ADCDATA + DATASIZE, dataB, sizeof(dataB));
  DATASIZE += sizeof(dataB);

  seqRunning = false;
}


void sendADCDATA()
{
  for (size_t i = 0; i < DATASIZE-1; i++)
  {
    Serial.printf("%d,",ADCDATA[i]);
  }
  Serial.println(ADCDATA[DATASIZE-1]);
  
}



void initTCP()
{
  Ethernet.begin(mac,ip);
  Serial.println("Try to initialize TCP \r\n");
  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    return;
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
    return;
  }
  Serial.println("Success start \r\n");
  // start listening for clients
  server.begin();

}

void listenForEthernetClients() 
{
  // listen for incoming clients
  EthernetClient client = server.available();
  Serial.println("Looking for client");
  if (client) {
    Serial.println("Got a client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available() > 0) {
        char c = client.read();
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println();
          // print the current readings, in HTML format:
          client.print("Temperature: ");
          client.print("dummy temperature");
          client.print(" degrees C");
          client.println("<br />");
          client.print("Pressure: " + String("dummy pressure"));
          client.print(" Pa");
          client.println("<br />");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("Connection closed");
  }
  Serial.println("No client");
}


bool debugMode(const String& rc)
{
  bool gotcha = true;
  Serial.println("I am in debug mode");
  if (rc.length() != 0){
    Serial.println(rc.c_str());
    if (rc.compareTo("write") == 0){
      Serial.println("re-write range register");
      SPI.beginTransaction(Wsetting);
      digitalWrite(CS,LOW);
      SPI.transfer16((1<<15)+(configRegAddr<<8) + 0b01100000); //enable burst and sequence, disable oversampling and CRC
      digitalWrite(CS, HIGH);  
      for (short i = 0; i < 4; i++)
      {
        digitalWrite(CS, LOW);
        SPI.transfer16((1<<15)+(rangeRegAddr[i]<<8) + rangeRegValue[i]);
        digitalWrite(CS, HIGH);  
      }
      SPI.endTransaction();
    }
    else if (rc.compareTo("reset") == 0){
      Serial.println("full-reset register");
      digitalWrite(RESET, LOW);
      delay(1);
      digitalWrite(RESET, HIGH);
      delay(100); 
    }
    else if (rc.compareTo("readseq") == 0){
      unsigned short seqtmp[32];
      SPI.beginTransaction(Rsetting);
      digitalWrite(CS,LOW);
      SPI.transfer16((0b01<<14)+(0<<8) + 0x00); 
      digitalWrite(CS, HIGH); 
      for (short i = 0; i < 31; i++)
      {
        digitalWrite(CS,LOW);
        seqtmp[i] = SPI.transfer16((0b01<<14) + ((i+1)<<9) + 0x00); 
        digitalWrite(CS, HIGH); 
      }
      digitalWrite(CS,LOW);
      seqtmp[31] = SPI.transfer16((0b01<<14) + ((31)<<9) + 0x00); 
      digitalWrite(CS, HIGH); 
      for (short i = 0; i < 32; i++)
      {

        Serial.printf("seq %2u:", i);
        Serial.print(seqtmp[i]>>8,HEX);
        Serial.println(seqtmp[i] & 0xff,HEX);
      }
    }
    else if (rc.compareTo("trig") == 0){
      // timer = 0;
      // digitalWrite(CONVST,HIGH);
      // delayMicroseconds(10);
      // digitalWrite(CONVST,LOW);
      readADC();
      sendADCDATA();
    }
    else if (rc.compareTo("mac") == 0){
      unsigned char mac[6];
      for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
      for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
      Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      initTCP();
      listenForEthernetClients();
    }
    else if (rc.compareTo("readreg") == 0){
      unsigned short tmp[5];
      SPI.beginTransaction(Rsetting);
      digitalWrite(CS,LOW);
      SPI.transfer16((0<<15)+(configRegAddr<<8) + 0x00); 
      digitalWrite(CS, HIGH); 
      digitalWrite(CS,LOW);
      tmp[0] = SPI.transfer16((0<<15)+(rangeRegAddr[0]<<8) + 0x00); 
      digitalWrite(CS, HIGH); 
      digitalWrite(CS,LOW);
      tmp[1] = SPI.transfer16((0<<15)+(rangeRegAddr[1]<<8) + 0x00); 
      digitalWrite(CS, HIGH); 
      digitalWrite(CS,LOW);
      tmp[2] = SPI.transfer16((0<<15)+(rangeRegAddr[2]<<8) + 0x00); 
      digitalWrite(CS, HIGH);
      digitalWrite(CS,LOW);
      tmp[3] = SPI.transfer16((0<<15)+(rangeRegAddr[3]<<8) + 0x00); 
      digitalWrite(CS, HIGH);
      digitalWrite(CS,LOW);
      tmp[4] = SPI.transfer16((0<<15)+(rangeRegAddr[3]<<8) + 0x00); 
      digitalWrite(CS, HIGH);
      SPI.endTransaction(); 

      for (short i = 0; i < 5; i++)
      {
        Serial.print(tmp[i]>>8,HEX);
        Serial.println(tmp[i] & 0xff,HEX);
      }
      
    }
    else{
      gotcha = false;
    }

  }
  // delay(100);
  return gotcha;
}

//initialization of hardware
void setup() 
{  
  // Initializing USB serial to 12Mbit/sec. Teensy ignores the 9600 baud rate. see https://www.pjrc.com/teensy/td_serial.html
  Serial.begin(9600);

  pinMode(RESET, OUTPUT);
  pinMode(CS, OUTPUT);
  pinMode(CONVST, OUTPUT);
  pinMode(BUSY, INPUT);
  pinMode(TRIG, INPUT);
  digitalWrite(CS,HIGH);
  attachInterrupt(digitalPinToInterrupt(TRIG),readADC,RISING);
  attachInterrupt(digitalPinToInterrupt(BUSY),dataReady, FALLING);

  SPI.begin();
  delay(500);
  initADC();

  initTCP();
}

void resetSeqTEENSY(){
  DATASIZE = 0;
  seqTotal = 0;
  seqCounter = 0;
  seqRunning = false; 
  memset(seqChannelA, 0, sizeof(seqChannelA));
  memset(seqChannelB, 0, sizeof(seqChannelB));
  memset(seqChannelSize, 0,sizeof(seqChannelSize));
  memset(seqRepNumber, 0, sizeof(seqRepNumber));
}

void writeSeq(unsigned short index){
  SPI.beginTransaction(Wsetting);

  /*disable burst and sequence, disable oversampling and CRC*/
  digitalWrite(CS,LOW);
  SPI.transfer16((1<<15)+(configRegAddr<<8) + 0b00000000); 
  digitalWrite(CS, HIGH); 

  /*program sequencer stack for channel A and B*/
  unsigned char minSize = min(seqChannelSize[index][0],seqChannelSize[index][1]);
  unsigned char maxSize = max(seqChannelSize[index][0],seqChannelSize[index][1]);
  for (unsigned char i = 0; i < minSize; i++) /*fill A and B*/
  {
    digitalWrite(CS,LOW);
    SPI.transfer16((1<<15)+ (1<<14) + (i<<9) + (0<<8) + (seqChannelB[index][i]<<4) + (seqChannelA[index][i])); // fill sequencer as ABAB...
    digitalWrite(CS, HIGH); 
  }
  if (minSize == maxSize) { /*rewrite the last seq to indicate termination*/
    digitalWrite(CS,LOW);
    SPI.transfer16((1<<15)+ (1<<14) + ((maxSize-1)<<9) + (1<<8) + (seqChannelB[index][maxSize-1]<<4) + (seqChannelA[index][maxSize-1]));
    digitalWrite(CS, HIGH); 
  }
  else if (minSize == seqChannelSize[index][0]){ /*sizeB>sizeA, fill B, default A to A0*/
    for (unsigned char i = minSize; i < maxSize - 1; i++)
    {
      digitalWrite(CS,LOW);
      SPI.transfer16((1<<15)+ (1<<14) + (i<<9) + (0<<8) + (seqChannelB[index][i]<<4) + (0)); 
      digitalWrite(CS, HIGH); 
    }
    digitalWrite(CS,LOW);
    SPI.transfer16((1<<15)+ (1<<14) + ((maxSize-1)<<9) + (1<<8) + (seqChannelB[index][maxSize-1]<<4) + (0)); 
    digitalWrite(CS, HIGH);
  }
  else if (minSize == seqChannelSize[index][1]){ /*sizeB<sizeA, fill A, default B to B0*/
    for (unsigned char i = minSize; i < maxSize - 1; i++)
    {
      digitalWrite(CS,LOW);
      SPI.transfer16((1<<15)+ (1<<14) + (i<<9) + (0<<8) + (0<<4) + (seqChannelA[index][i])); // fill sequencer as ABAB...
      digitalWrite(CS, HIGH); 
    }
    digitalWrite(CS,LOW);
    SPI.transfer16((1<<15)+ (1<<14) + ((maxSize-1)<<9) + (1<<8) + (0<<4) + (seqChannelA[index][maxSize-1])); 
    digitalWrite(CS, HIGH);
  }

  /*enable burst and sequence (note these two has to be enabled at the same time), disable oversampling and CRC*/
  digitalWrite(CS,LOW);
  SPI.transfer16((1<<15)+(configRegAddr<<8) + 0b01100000); 
  digitalWrite(CS, HIGH); 

  SPI.endTransaction();

  /*per manual, better provide a dummy reading before reading sequencer*/
  dummyReadADC();

}

void interpretCmd()
{
  String rc = Serial.readString(MAXIN);
  if (rc.length()==0){
    Serial.println("Error: nothing received");
  }
  #ifdef AD7616DEBUG
  if (debugMode(rc)) {
    return;
  }
  #endif

  resetSeqTEENSY();
  unsigned short charcnts = 0;
  unsigned short numSEQ = 0;
  byte chnl[3];

  while (charcnts < rc.length())
  {
    if (rc[charcnts++]!=STARTM) {
      Serial.println("Error: Start terminator \"" + String(STARTM) + "\" is missing");
      resetSeqTEENSY();
      return;
    }
    rc.getBytes(chnl,3,charcnts); // the last character is always '\0' by String::getBytes
    if (chnl[0]==0 || chnl[1]==0){
      Serial.println("Error: Encounter zero in channel selection");
      resetSeqTEENSY();
      return;
    }
    Serial.println((char*)chnl);
    charcnts+=2;
    if (rc[charcnts++]!=SEPAR) {
      Serial.println("Error: Separator \"" + String(SEPAR) + "\" is missing");
      resetSeqTEENSY();
      return;
    }
    short endpos = rc.indexOf(ENDM, charcnts); // endposition points to END MARKER
    long repts = rc.substring(charcnts,endpos).toInt(); // not including endposition, the endposition is filled with '\0' in the substring
    if (repts<=0) {
      Serial.println("Error: Repeatation number is less or equal to zero");
      resetSeqTEENSY();
      return;
    }
    Serial.println(repts);
    charcnts = endpos;
    Serial.println(rc[charcnts]);
    if (rc[charcnts++]!=ENDM) {
      Serial.println("Error: Start terminator \"" + String(ENDM) + "\" is missing");
      resetSeqTEENSY();
      return;
    }

    // data in chnl is A7-A0, B7-B0
    for (unsigned char j = 0; j < 8/*bits*/; j++)
    {
      if ((chnl[0]>>j) & 0b1){ /*channel A*/
        seqChannelA[numSEQ][seqChannelSize[numSEQ][0]++] = j;
      }
      if ((chnl[1]>>j) & 0b1){ /*channel B*/
        seqChannelB[numSEQ][seqChannelSize[numSEQ][1]++] = j;
      }
    }
    seqRepNumber[numSEQ] = repts;
    // Serial.println(seqChannel[numSEQ]);
    numSEQ++;
  }
  seqTotal = numSEQ;

  /*write the first sequence and wait for trigger, the following sequence is written after each data out*/
  writeSeq(seqCounter);
  Serial.println("Success in decoding incoming sequence");

}

//main function
void loop() 
{
  // put your main code here, to run repeatedly:
  if (Serial.available()){
    interpretCmd();
  }

}
