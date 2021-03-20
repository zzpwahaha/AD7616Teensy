/****************************************************************************************

 ***************************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <EEPROMex.h>
#include <stdio.h>
// #include <NativeEthernet.h>
#include <../lib/NativeEthernet/src/NativeEthernet.h>

/******************************************************VARIABLES*************************************************************/
#undef AD7616DEBUG
bool debug = false;

/*interpreting input command*/
const char STARTM = '(';        // start marker for each set of command data
const char SEPAR = ',';         // separator within one set of command data
const char ENDM = ')';          // end marker for each set of command data
const char TERMINATOR = '#';    // terminator for tcp socket connection, the command data in should end with TERMINATOR + '\0'
const char PLACEHOLDER = '*';   // placeholder for sending command, if send in (*xx,xxxx), it will go as channel conversion, otherwise (xxx,xxxx) will go as command
const short CMDHEADSIZE = 3;    // size of the command head, ie size of 'xxx' before the separator from (xxx,xxxx)
const short MAXIN = 600;        // maximum number of chars in input stream
const short MAXSEQ = MAXIN / 6; // maximum number of sequences
const short MAXBUF = 2048;      // maximum output buff for TCP socket, break single send if byte exceeds this number

/*global vairbale for sequencer command*/
unsigned char seqChannelA[MAXSEQ][8];    // measured channel index(0-7) in sequencer for channel A, the size is stored in seqChannelSize
unsigned char seqChannelB[MAXSEQ][8];    // measured channel index(0-7) in sequencer for channel B, the size is stored in seqChannelSize
unsigned char seqChannelSize[MAXSEQ][2]; // size of the measured channel in A and B respesctively
unsigned long seqRepNumber[MAXSEQ];      // number of repeat measurement in sequencer   

short seqTotal = 0;
short seqCounter = 0;
bool seqRunning = false; 
bool dummyRead = false;

/*external 8M memory storage*/
EXTMEM short ADCDATA[4000000];
unsigned int DATASIZE = 0; 

/*Teensy pin configuration*/
const short RESET = 9;    // out
const short CS = 10;      // out
const short SCLK = 13;    // out, only for initializing clock to be high in initDAC, since we are using mode2 of SPI. Maybe not nessary
const short CONVST = 15;  // out, triggers AD7616 to start single sequence
const short BUSY = 14;    // in, falling edge trigers dataReady 
const short TRIG = 16;    // in, external trigger for starting one full ADC read, repeatition * sequence

/*SPI setting*/
const SPISettings Wsetting(45000000, MSBFIRST, SPI_MODE2);
const SPISettings Rsetting(45000000, MSBFIRST, SPI_MODE2);

/*internal register address of AD7616, refer to datasheet*/
const short configRegAddr   = 0b0000100;
const short channRegAddr    = 0b0000110;
const short rangeRegAddr[]  = {0b0001000, 0b0001010, 0b0001100, 0b0001110}; /*A1,A2,B1,B2*/
const short rangeRegValue[] = {0x0000, 0x0000, 0x0000, 0x0000};//{0x00aa, 0x00bb, 0x00cc, 0x00dd}; // right now the values are for test
const short dummyReadAddr   = 0/*(1<<9)*/; // used for reading conversion result out of ADC, turns out can just use 0, if use 1<<9 which is a reserved addr, only read out the first result and the following result are all zeros

elapsedMicros timer;
/****************************************************ETHERNET CONNECTION****************************************************/
byte mac[] = {0x04,0xe9,0xe5,0x0e,0x0c,0xe0};
IPAddress ip(10,10,0,10);
EthernetServer server(80);


/******************************************************FUNCTION HEAD********************************************************/
bool debugMode(const String& rc);

void interpretCmd(const String& rc);
void handleDirectCmd(String cmd, String content);

void initADC();
void startReadADC();
void dataReady();
void dummyReadADC();

void sendADCDATA();

void initTCP();
bool listenForEthernetClients(String& rc);

void resetSeqTEENSY();
void writeSeq(unsigned short index);

/********************************************************************************************************************************/
//initialization of hardware
void setup() 
{  
  // Initializing USB serial to 12Mbit/sec. Teensy ignores the 9600 baud rate. see https://www.pjrc.com/teensy/td_serial.html
  Serial.begin(9600);

  pinMode(RESET, OUTPUT);
  pinMode(CS, OUTPUT);
  pinMode(CONVST, OUTPUT);
  pinMode(BUSY, INPUT);
  pinMode(TRIG, INPUT_PULLDOWN);
  digitalWrite(CS,HIGH);
  attachInterrupt(digitalPinToInterrupt(TRIG),startReadADC,RISING);
  attachInterrupt(digitalPinToInterrupt(BUSY),dataReady, FALLING);

  SPI.begin();
  delay(500);
  initADC();
  initTCP();
}

//main function
void loop() 
{
  // put your main code here, to run repeatedly:
  if (Serial.available()){
    String rc = Serial.readString(MAXIN);
    if (rc.length()==0){
      Serial.println("Error: nothing received");
    }
    interpretCmd(rc);
  }
  else{
    timer = 0;
    String rc;
    bool success = listenForEthernetClients(rc);
    if (success){
      Serial.printf("After listenForEthernetClients, used %u us\r\n",(unsigned long)timer);
      interpretCmd(rc);
    }
  }

  // listenForEthernetClients();

}

void initADC()
{
  // digitalWrite(SCLK, HIGH); //prepare it for the SPI, just to be safe
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

void startReadADC()
{
  Serial.println("Start to read ADC");
  if (seqTotal==0){
    Serial.println("Error: sequence number is zero, no sequence is armed for trigger");
    server.println("Error: sequence number is zero, no sequence is armed for trigger");
    return;
  }
  timer = 0;
  for (unsigned int repts = 0; repts < seqRepNumber[seqCounter]; repts++)
  {
    seqRunning = true;
    digitalWrite(CONVST,HIGH);
    delayNanoseconds(200);
    digitalWrite(CONVST,LOW);
    while (seqRunning){
      // delayMicroseconds(1);
      // Serial.println(int(seqRunning));
      Serial.flush(); // have to add a serial related operation here otherwise it won't jump out of while loop even seqRunning is false
      // Serial.println("asd");
    }
    // Serial.println("Finish read ADC" + String(repts));
  }
  Serial.printf("Finish read all ADC %d repts in %d us, %.2f us per repetition and %.2f per repetition per ports\r\n", 
    seqRepNumber[seqCounter], (unsigned long)timer, (double)timer / seqRepNumber[seqCounter], 
    (double)timer / seqRepNumber[seqCounter] / (seqChannelSize[seqCounter][0]+seqChannelSize[seqCounter][1])  );
  seqCounter++;
  if (seqCounter<seqTotal){
    writeSeq(seqCounter);
  }
  else{
    sendADCDATA();
    Serial.println("Finised all stored sequencer command");
  }
}

void dummyReadADC()
{
  dummyRead = true;
  digitalWrite(CONVST,HIGH);
  delayNanoseconds(200);
  digitalWrite(CONVST,LOW);
  // Serial.println("Start dummy read");
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
    Serial.println("Finished dummy read");
    return;
  }
  else if (debug) {
    Serial.println("debug sequence output 16 ports");
    SPI.beginTransaction(Rsetting);
    for (short i = 0; i < 16; i++)
    {
      digitalWrite(CS,LOW);
      signed short data = SPI.transfer16(0);
      Serial.println(data);
    }
    debug = false;
    return;
  }
  
  // Serial.println("received a falling edge of BUSY");
  // Serial.printf("Total time: %u \r\n", static_cast<unsigned long>(timer) );
  short dataA[seqChannelSize[seqCounter][0]];
  short dataB[seqChannelSize[seqCounter][1]];
  unsigned char minSize = min(seqChannelSize[seqCounter][0],seqChannelSize[seqCounter][1]);
  unsigned char maxSize = max(seqChannelSize[seqCounter][0],seqChannelSize[seqCounter][1]);
  SPI.beginTransaction(Rsetting);
  for (unsigned char i = 0; i < minSize; i++)
  {
    digitalWrite(CS,LOW);
    dataA[i] = SPI.transfer16(dummyReadAddr); // D9=1 is the reserved addr which does not correspond to anything
    dataB[i] = SPI.transfer16(dummyReadAddr); // D9=1 is the reserved addr which does not correspond to anything
    digitalWrite(CS,HIGH);
    // Serial.printf("dataA: %d, dataB: %d \r\n", dataA[i],dataB[i]);
  }
  if (minSize==maxSize){
    // Serial.println("size of channel A and B are equal");
  }
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
  // Serial.println("finished a falling edge of BUSY");
  // Serial.printf("Size of dataA is %d, channel size is %d \r\n",sizeof(dataA), seqChannelSize[seqCounter][0]);
  // Serial.printf("Size of dataB is %d, channel size is %d \r\n",sizeof(dataB), seqChannelSize[seqCounter][1]);
  memcpy(ADCDATA + DATASIZE, dataA, sizeof(dataA)); // sizeof returns the number of byte in total to be copied
  DATASIZE += seqChannelSize[seqCounter][0]; //sizeof(dataA) / 2/*byte for short*/;
  memcpy(ADCDATA + DATASIZE, dataB, sizeof(dataB));
  DATASIZE += seqChannelSize[seqCounter][1]; //sizeof(dataB) / 2/*byte for short*/;

  // Serial.println("Finished copying data to ext mem");
  seqRunning = false;
  // Serial.println("finished setting seqRunning");
}

//the TCP socket buffer content will exist if not read by client
void sendADCDATA()
{
  timer = 0;
  String sd;
  sd.reserve(DATASIZE*2);
  for (size_t i = 0; i < DATASIZE-1; i++)
  {
    sd.append((char)(ADCDATA[i]>>8));
    sd.append((char)(ADCDATA[i]&0xff));
    // Serial.printf("%d,",ADCDATA[i]);
    // server.printf("%c%c",(char)(ADCDATA[i]>>8),(char)(ADCDATA[i]));
    // server.printf("%d,",ADCDATA[i]);
    // server.write(uint8_t(ADCDATA[i]>>8));
    // server.write(uint8_t(ADCDATA[i]));
  }
  sd.append((char)(ADCDATA[DATASIZE-1]>>8));
  sd.append((char)(ADCDATA[DATASIZE-1]&0xff));
  // sd.append((char)(ADCDATA[DATASIZE-1]));
  // Serial.println(ADCDATA[DATASIZE-1]);
  // server.printf("%c%c",(char)(ADCDATA[DATASIZE-1]>>8),(char)(ADCDATA[DATASIZE-1]));
  // server.printf("%d\0",ADCDATA[DATASIZE-1]);
  // server.write(uint8_t(ADCDATA[DATASIZE-1]>>8));
  // server.write(uint8_t(ADCDATA[DATASIZE-1]));
  
  
  if (DATASIZE*2>MAXBUF){
    for (unsigned i = 0; i < (DATASIZE*2)/MAXBUF; i++)
    {
      server.write(sd.c_str() + i*MAXBUF, MAXBUF);
    }
    server.write(sd.c_str() + ((DATASIZE*2)/MAXBUF)*MAXBUF, (DATASIZE*2)%MAXBUF);
  }
  else{
    server.write(sd.c_str(),DATASIZE*2);
  }
  
  Serial.println(sd);
  Serial.printf("Total send time is %d us and %.2f us per 2 byte data \r\n", (unsigned long)timer, double(timer)/DATASIZE);

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

bool listenForEthernetClients(String& rc) 
{
  // listen for incoming clients
  EthernetClient client = server.available();
  // Serial.println("Looking for client");
  if (client) {
    Serial.println("Got a client");
    char buff[MAXIN];
    if (client.available() > 0) {
      size_t size = client.readBytesUntil('\0',buff,MAXIN);
      if (buff[size-1] != TERMINATOR){
        Serial.println(buff);
        Serial.printf("Error in reading TCP command: size=%d, end character is %c, but should be %c\r\n", size, buff[size-1],TERMINATOR);
        server.printf("Error in reading TCP command: size=%d, end character is %c, but should be %c\r\n", size, buff[size-1],TERMINATOR);
        return false;
      }
      if (server.available()){
        Serial.println(buff);
        Serial.printf("Error in reading TCP command: Command is exceeding max size: %d, or command is sent in too fast\r\n", MAXIN);
        server.printf("Error in reading TCP command: Command is exceeding max size: %d, or command is sent in too fast\r\n", MAXIN);
        return false;
      }
      buff[size-1] = 0; // remove TERMINATOR by changing the TERMINATOR to null 
      rc = String(buff);
      // Serial.println("Command from TCP socket " + rc + " of size: " + String(size));
      return true;
      // note null(0) in String will make string be chopped at that point, can only put it in char[] 
    }
  }
  return false;
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
    else if (rc.compareTo("writeseq") == 0){
      SPI.beginTransaction(Rsetting);
      unsigned short channel[] = {4,5,6};
      for (short i = 0; i < 3; i++)
      {
        digitalWrite(CS,LOW);
        SPI.transfer16((0b11<<14) + (i<<9) + (0<<8) + (channel[i]<<4) + (channel[i])); 
        digitalWrite(CS, HIGH); 
      }
      digitalWrite(CS,LOW);
      SPI.transfer16((0b11<<14) + (2<<9) + (1<<8) + (channel[2]<<4) + (channel[2])); 
      digitalWrite(CS, HIGH); 
    }
    else if (rc.compareTo("convst") == 0){
      debug = true;
      digitalWrite(CONVST,HIGH);
      delayNanoseconds(200);
      digitalWrite(CONVST,LOW);
    }
    else if (rc.compareTo("trig") == 0){
      // timer = 0;
      // digitalWrite(CONVST,HIGH);
      // delayMicroseconds(10);
      // digitalWrite(CONVST,LOW);
      startReadADC();
      // sendADCDATA();//this send all valid storage of all previous acquisition
    }
    else if (rc.compareTo("mac") == 0){
      unsigned char mac[6];
      for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
      for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
      Serial.printf("MAC:%c %02x:%02x:%02x:%02x:%02x:%02x\r\n",0, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      server.printf("MAC:%c %02x:%02x:%02x:%02x:%02x:%02x\r\n",0, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      // initTCP();
      // listenForEthernetClients();
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
      Serial.printf("Command %s is not recongized in debug mode\r\n", rc.c_str());
    }

  }
  // delay(100);
  return gotcha;
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
  // memset(ADCDATA, 0, sizeof(ADCDATA));
}

void writeSeq(unsigned short index){
  SPI.beginTransaction(Wsetting);
  Serial.printf("After start transaction, used %u us\r\n",(unsigned long)timer);

  /*disable burst and sequence, disable oversampling and CRC*/
  digitalWrite(CS,LOW);
  SPI.transfer16((1<<15)+(configRegAddr<<8) + 0b00000000); 
  digitalWrite(CS, HIGH); 

  /*program sequencer stack for channel A and B*/
  unsigned short minSize = min(seqChannelSize[index][0],seqChannelSize[index][1]);
  unsigned short maxSize = max(seqChannelSize[index][0],seqChannelSize[index][1]);
  for (unsigned short i = 0; i < minSize; i++) /*fill A and B*/
  {
    digitalWrite(CS,LOW);
    SPI.transfer16((1<<15)+ (1<<14) + (i<<9) + (0<<8) + (seqChannelB[index][i]<<4) + (seqChannelA[index][i])); // fill sequencer as ABAB...
    digitalWrite(CS, HIGH); 

    // Serial.printf("write seq %d: ", i);
    // Serial.println((1<<15)+ (1<<14) + (i<<9) + (0<<8) + (seqChannelB[index][i]<<4) + (seqChannelA[index][i]), 2);
  }
  Serial.printf("After write common SPI sequence, used %u us\r\n",(unsigned long)timer);
  if (minSize == maxSize) { /*rewrite the last seq to indicate termination*/
    digitalWrite(CS,LOW);
    SPI.transfer16((1<<15)+ (1<<14) + ((maxSize-1)<<9) + (1<<8) + (seqChannelB[index][maxSize-1]<<4) + (seqChannelA[index][maxSize-1]));
    digitalWrite(CS, HIGH); 
    // Serial.println("Size A and Size B are equal");
    // Serial.printf("write seq %d: ", maxSize-1);
    // Serial.println((1<<15)+ (1<<14) + ((maxSize-1)<<9) + (1<<8) + (seqChannelB[index][maxSize-1]<<4) + (seqChannelA[index][maxSize-1]), 2);
  }
  else if (minSize == seqChannelSize[index][0]){ /*sizeB>sizeA, fill B, default A to A0*/
    Serial.println("Size A < Size B");
    for (unsigned short i = minSize; i < maxSize - 1; i++)
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
    Serial.println("Size A > Size B");
    for (unsigned short i = minSize; i < maxSize - 1; i++)
    {
      digitalWrite(CS,LOW);
      SPI.transfer16((1<<15)+ (1<<14) + (i<<9) + (0<<8) + (0<<4) + (seqChannelA[index][i])); // fill sequencer as ABAB...
      digitalWrite(CS, HIGH); 
    }
    digitalWrite(CS,LOW);
    SPI.transfer16((1<<15)+ (1<<14) + ((maxSize-1)<<9) + (1<<8) + (0<<4) + (seqChannelA[index][maxSize-1])); 
    digitalWrite(CS, HIGH);
  }
  Serial.printf("After write SPI difference sequence, used %u us\r\n",(unsigned long)timer);
  /*enable burst and sequence (note these two has to be enabled at the same time), disable oversampling and CRC*/
  digitalWrite(CS,LOW);
  SPI.transfer16((1<<15)+(configRegAddr<<8) + 0b01100000); 
  digitalWrite(CS, HIGH); 

  SPI.endTransaction();

  /*per manual, better provide a dummy reading before reading sequencer*/
  dummyReadADC();

}

void interpretCmd(const String &rc)
{
  #ifdef AD7616DEBUG
  if (debugMode(rc)) {
    return;
  }
  #endif


  bool firstSeq = true;
  unsigned short charcnts = 0;
  unsigned short numSEQ = 0;
  byte chnl[CMDHEADSIZE + 1];

  while (charcnts < rc.length())
  {
    if (rc[charcnts++]!=STARTM) {
      Serial.println("Error: Start terminator \"" + String(STARTM) + "\" is missing");
      server.println("Error: Start terminator \"" + String(STARTM) + "\" is missing");
      resetSeqTEENSY();
      return;
    }
    rc.getBytes(chnl, CMDHEADSIZE+1, charcnts); // the last character is always '\0' by String::getBytes
    if (chnl[0]==0 || chnl[1]==0){
      Serial.println("Error: Encounter zero in channel selection");
      server.println("Error: Encounter zero in channel selection");
      resetSeqTEENSY();
      return;
    }
    // Serial.println((char*)chnl);
    charcnts+=CMDHEADSIZE;
    if (rc[charcnts++]!=SEPAR) {
      Serial.println("Error: Separator \"" + String(SEPAR) + "\" is missing");
      server.println("Error: Separator \"" + String(SEPAR) + "\" is missing");
      resetSeqTEENSY();
      return;
    }
    short endpos = rc.indexOf(ENDM, charcnts); // endposition points to END MARKER
    String contents = rc.substring(charcnts,endpos); // not including endposition, the endposition is filled with '\0' in the substring
    charcnts = endpos;
    // Serial.println(rc[charcnts]);
    if (rc[charcnts++]!=ENDM) {
      Serial.println("Error: Start terminator \"" + String(ENDM) + "\" is missing");
      server.println("Error: Start terminator \"" + String(ENDM) + "\" is missing");
      resetSeqTEENSY();
      return;
    }
    
    if (chnl[0]==PLACEHOLDER){ /*interpret as (*xx,xxxx), ie the channel to be sampled and the repeatition number */
      Serial.printf("After check PLACEHOLDER, used %u us\r\n",(unsigned long)timer);
      if(firstSeq){
        resetSeqTEENSY();
        firstSeq = false;
      }
      Serial.printf("Before read repetition, used %u us\r\n",(unsigned long)timer);
      long repts = contents.toInt(); 
      Serial.printf("After read repetition, used %u us\r\n",(unsigned long)timer);
      if (repts<=0) {
        Serial.println("Error: Repeatation number is less or equal to zero");
        resetSeqTEENSY();
        return;
      }
      // Serial.println(repts);
      Serial.printf("After check repetition, used %u us\r\n",(unsigned long)timer);
      // data in chnl is A7-A0, B7-B0
      for (unsigned char j = 0; j < 8/*bits*/; j++)
      {
        if ((chnl[1]>>j) & 0b1){ /*channel A*/
          seqChannelA[numSEQ][seqChannelSize[numSEQ][0]++] = j;
        }
        if ((chnl[2]>>j) & 0b1){ /*channel B*/
          seqChannelB[numSEQ][seqChannelSize[numSEQ][1]++] = j;
        }
      }
      Serial.printf("After write seqChannel mem sequence, used %u us\r\n",(unsigned long)timer);
      // Serial.printf("seqChannelA(numSEQ = %d): ", numSEQ);
      // for (unsigned char j = 0; j < seqChannelSize[numSEQ][0]; j++)
      // {
      //   Serial.printf("%d, ",seqChannelA[numSEQ][j]);
      // }
      // Serial.print("\r\n");
      // Serial.printf("seqChannelB(numSEQ = %d): ", numSEQ);
      // for (unsigned char j = 0; j < seqChannelSize[numSEQ][1]; j++)
      // {
      //   Serial.printf("%d, ",seqChannelB[numSEQ][j]);
      // }
      // Serial.print("\r\n");
      
      seqRepNumber[numSEQ] = repts;
      // Serial.println(seqChannel[numSEQ]);
      numSEQ++;
    }
    else{/*interpret as (xxx,xxxx), ie sending in command*/
      handleDirectCmd(String((char*)chnl),contents);
    }
  }

  if (numSEQ>0){/*the incoming command containes sequencer*/
    seqTotal = numSEQ;
    /*write the first sequence and wait for trigger, the following sequence is written after each data out*/
    Serial.printf("After analyzing all input command, start SPI seq, used %u us\r\n",(unsigned long)timer);
    writeSeq(seqCounter);
    Serial.printf("After write SPI sequence, used %u us\r\n",(unsigned long)timer);
    Serial.printf("Success in decoding incoming sequence with %d us\r\n", (unsigned long)timer);
  }

}

void handleDirectCmd(String cmd, String content)
{
  //note, be extremely careful with 0x00 in content since that may cause termination of the string
  if(cmd.compareTo("rng") == 0){/*content is the input range of A3-A0,A7-A4,B3-B0,B7-B4 encoded as raw binary form 0b11 -> 10V, 0b01 -> 2.5V, 0b10 -> 5V*/
    Serial.printf("Direct command, write range register with 0x%02X 0x%02X 0x%02X 0x%02X to A1A2,B1B2\r\n", content[0],content[1],content[2],content[3]);
    if (content.length() != 4){
      Serial.printf("Error: write range register with 0x%02X 0x%02X 0x%02X 0x%02X is invalid\r\n", content[0],content[1],content[2],content[3]);
      server.printf("Error: write range register with 0x%02X 0x%02X 0x%02X 0x%02X is invalid\r\n", content[0],content[1],content[2],content[3]);
    }
    byte buff[4+1];
    content.getBytes(buff,4+1);
    SPI.beginTransaction(Wsetting);
    for (short i = 0; i < 4; i++)
    {
      digitalWrite(CS,LOW);
      SPI.transfer16((1<<15)+(rangeRegAddr[i]<<8) + buff[i/*2*(i/2)+1-(i%2)*/]); 
      digitalWrite(CS, HIGH); 
    }
    server.println("Success in writing range register");
  }
  else if (cmd.compareTo("trg") == 0){
    startReadADC();
    // server.println("Success in writing software trig");
    // sendADCDATA();//this send all valid storage of all previous acquisition
  }
  else{
    Serial.printf("Error: direct command: %s, not recongized\r\n", cmd.c_str());
  }
  
}

