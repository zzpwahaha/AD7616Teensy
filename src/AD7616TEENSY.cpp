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
void processData();
bool dataCheck(uint32_t par[5]);
void setRamp0();
void setRamp1();

//Variable declaration
const int cs0 = 14;  //slave select pin for ch0
const int cs1 = 15;  //slave select pin for ch1

const int Trig0 = 7;  //ch0 ramp trigger pin
const int Trig1 = 8;  //ch1 ramp trigger pin

const uint32_t fPFD = 50000;  //PFD frequency in kHz

int32_t ramp0[40][6];  //ramp parameters for ch0 (Fs,Fe,R#,Rleng,dF,dt)
int32_t ramp1[40][6];  //ramp parameters for ch1 (Fs,Fe,R#,Rleng,dF,dt)

uint32_t address = 0;  //eeprom address for ch0,1 default frequencies

elapsedMicros rampDelay0;  //ramp delay timer0
elapsedMicros rampDelay1;  //ramp delay timer1

uint32_t dt0;  //ch0 ramp step delay interval
uint32_t dt1;  //ch1 ramp step delay interval

uint32_t stepCount0;  //ramp step counter for ch0
uint32_t stepCount1;  //ramp step counter for ch1

uint32_t rampflg0;  //ramp start/stop flag
uint32_t rampflg1;  //ramp start/stop flag

uint32_t rampCounter0;  //number of uploaded ramp in ch0
uint32_t rampCounter1;  //number of uploaded ramp in ch1

const char startMarker = '(';  //start marker for each set of data
const char endMarker = ')';  //end marker for each set of data
const char endendMarker = 'e';  //end marker for the whole frame of data


/*******************************************************************************************************************/
#define AD7616DEBUG
const char TERMINATOR = '#';

const short RESET = 9;
const short CS = 10;      // out
const short SCLK = 13;    // out, only for initializing clock to be high in initDAC, since we are using mode2 of SPI. Maybe not nessary
const short CONVST = 15;  // out
const short BUSY = 14;    // in 
const short TRIG = 16;    // in


const SPISettings Wsetting(45000000, MSBFIRST, SPI_MODE3);
const SPISettings Rsetting(45000000, MSBFIRST, SPI_MODE3);
const short configRegAddr   = 0b0000100;
const short channRegAddr    = 0b0000110;
const short rangeRegAddr[]  = {0b0001000, 0b0001010, 0b0001100, 0b0001110}; /*A1,A2,B1,B2*/
const short rangeRegValue[] = {0x0000, 0x0000, 0x0000, 0x0000};//{0x00aa, 0x00bb, 0x00cc, 0x00dd}; // right now the values are for test

elapsedMicros timer;
/******************ETHERNET CONNECTION****************************/
byte mac[] = {0x04,0xe9,0xe5,0x0e,0x0c,0xe0};
IPAddress ip(10,10,0,10);
EthernetServer server(80);

void debugMode();

void initADC();
void readADC();
void dataReady();

void initTCP();
void listenForEthernetClients();

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

}

void dataReady()
{
  Serial.printf("Total time: %u \r\n", static_cast<unsigned long>(timer) );
  SPI.beginTransaction(Rsetting);
  for (short i = 0; i < 16; i++)
  {
    digitalWrite(CS,LOW);
    signed short data = SPI.transfer16(0);
    Serial.println(data);
  }
  

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


void debugMode()
{
  String rc;
  if (Serial.available()){
    rc = Serial.readStringUntil(TERMINATOR);
  }
  else{
    return;
  }
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
      SPI.beginTransaction(Wsetting);
      digitalWrite(RESET, LOW);
      delay(1);
      digitalWrite(RESET, HIGH);
      delay(100);
      SPI.endTransaction();  
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
      timer = 0;
      digitalWrite(CONVST,HIGH);
      delayMicroseconds(10);
      digitalWrite(CONVST,LOW);
    }
    else if (rc.compareTo("mac") == 0){
      unsigned char mac[6];
      for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
      for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
      Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      initTCP();
      listenForEthernetClients();
    }
    else{
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
  }
  delay(100);
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

  // Initializing USB serial to 12Mbit/sec. Teensy ignores the 9600 baud rate. see https://www.pjrc.com/teensy/td_serial.html
  // Serial.begin(9600);
  // //Initializing SPI to 1Mbit/sec
  // SPI.begin();
  // SPI.setClockDivider(SPI_CLOCK_DIV2);
  // pinMode(cs0,OUTPUT);   //set cs pin as output
  // pinMode(cs1,OUTPUT);
  // digitalWrite(cs0,LOW);
  // digitalWrite(cs1,LOW);
  // //Initialize trigger pins and attach interrupts
  // pinMode(Trig0,INPUT);
  // pinMode(Trig1,INPUT);
  // attachInterrupt(digitalPinToInterrupt(Trig0),setRamp0,RISING);
  // attachInterrupt(digitalPinToInterrupt(Trig1),setRamp1,RISING);
  // //Initialize ADF4159 to default frequency from EEPROM
  // delay(1000);
  // pfdInit();
  
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for native USB port only
  // }
  initTCP();
}

//main function
void loop() 
{
  // put your main code here, to run repeatedly:
  #ifdef AD7616DEBUG
  debugMode();
  #endif

  //Check for serial data
  // if (Serial.available()){
  //   String rc = Serial.readStringUntil(TERMINATOR);

  //   Serial.print(rc);
  //   Serial.println(", data is still avai");
  //   // processData();
  // }
  // String rc;
  // if (Serial.available()){
  //   rc = Serial.readStringUntil(TERMINATOR);
  // }
  // strtol
  // if (rampflg0%2 && rampflg0/2 < rampCounter0){
  //   if (stepCount0 == 0){
  //     dt0 = ramp0[rampflg0/2][5];
  //     updatePFD(calcFTW(ramp0[rampflg0/2][0]),cs0);
  //     stepCount0++;
  //   }
  //   if (rampDelay0 >= dt0 && stepCount0 <= ramp0[rampflg0/2][2]){
  //     rampDelay0 = rampDelay0 - dt0;
  //     //update ch0 pfd function goes here
  //     updatePFD(calcFTW(ramp0[rampflg0/2][0]+(ramp0[rampflg0/2][4]*stepCount0)),cs0);
  //     stepCount0++;
  //   }
  //   else if (stepCount0 > ramp0[rampflg0/2][2]){
  //     stepCount0 = 0;
  //     rampflg0++;
  //   }
  // }
  // if (rampflg1%2 && rampflg1/2 < rampCounter1){
  //   if (stepCount1 == 0){
  //     dt1 = ramp1[rampflg1/2][5];
  //     updatePFD(calcFTW(ramp1[rampflg1/2][1]),cs1);
  //     stepCount1++;
  //   }
  //   if (rampDelay1 >= dt1 && stepCount1 <= ramp1[rampflg1/2][2]){
  //     rampDelay1 = rampDelay1 - dt1;
  //     //update ch1 pfd function goes here
  //     updatePFD(calcFTW(ramp1[rampflg1/2][0]+(ramp1[rampflg1/2][4]*stepCount1)),cs1);
  //     stepCount1++;
  //   }
  //   else if (stepCount1 > ramp1[rampflg1/2][2]){
  //     stepCount1 = 0;
  //     rampflg1++;
  //   }
  // }

}

//Initialize ADF4159 board and load default frequencies from EEPROM
//The ADF4159 is initialized to the following 
//ch0: R=2,cp=0.3125mA
//ch1: R=2, cp=0.625mA
void pfdInit(){
  char INI[28] = {0,0,0,7,
                  0,0x00,0,6,
                  0,0x80,0,6,
                  0,0x00,0,5,
                  0,0x80,0,5,
                  0,0x18,1,0x04,
                  0,0x18,1,0x44};
  char ch0R23[8] = {0x01,0x03,0x00,0x83,0x00,0x01,0x00,0x0A};
  char ch1R23[8] = {0x00,0x83,0x00,0xC3,0x01,0x01,0x00,0x0A};
  //ch0 R4,R5,R6,R7 Initialization
  for(int i=0;i<7;i++){
    for(int j=0;j<4;j++){
      SPI.transfer(INI[4*i+j]);
    }
    digitalWrite(cs0,HIGH);
    digitalWrite(cs0,LOW);
  }
  //write ch0 R3,R2
  for(int i=0;i<2;i++){
    for(int j=0; j<4;j++){
      SPI.transfer(ch0R23[4*i+j]);
    }
    digitalWrite(cs0,HIGH);
    digitalWrite(cs0,LOW);
  }
  //write ch0 R1,R0
  updatePFD(calcFTW(EEPROM.readLong(address)),cs0);
  //ch1 R7,R6,R5,R4 Initialization
  for(int i=0;i<7;i++){
    for(int j=0;j<4;j++){
      SPI.transfer(INI[4*i+j]);
    }
    digitalWrite(cs1,HIGH);
    digitalWrite(cs1,LOW);
  }
  //write ch1 R3,R2
  for(int i=0;i<2;i++){
    for(int j=0; j<4;j++){
      SPI.transfer(ch1R23[4*i+j]);
    }
    digitalWrite(cs1,HIGH);
    digitalWrite(cs1,LOW);
  }
  //write ch1 R1,R0
  updatePFD(calcFTW(EEPROM.readLong(address+5)),cs1);
}

//Calculate the FTW according to the input frequency in kHz
uint32_t calcFTW(uint32_t freq){
  uint32_t N;
  float Frac;
  uint32_t FTW;
  N = freq/fPFD;
  Frac = (float)(freq%fPFD)/fPFD*0x2000000; //2**25//33554432;
  FTW = (N << 25)+ (uint32_t)Frac;
  return FTW;
}

//send data to ADF4159, FTW is in the format of (N7..N1,F25..F1)
void updatePFD(uint32_t FTW, int LE){
  char R0[4];
  char R1[4];
  
  R0[0] = 0x00;
  R0[1] = (uint32_t)((FTW >> 26) & 0x3F);
  R0[2] = (uint32_t)((FTW >> 18) & 0xFF);
  R0[3] = (uint32_t)((FTW >> 10) & 0xF8);

  R1[0] = (uint32_t)((FTW >> 9) & 0x0F);
  R1[1] = (uint32_t)((FTW >> 1) & 0xFF);
  R1[2] = (uint32_t)((FTW << 7) & 0x80);
  R1[3] = 0x01;

  //write to R1 first
  for (int n=0; n<4; n++){
    SPI.transfer(R1[n]);
  }
  digitalWrite(LE,HIGH);
  digitalWrite(LE,LOW);
  //write to R0
  for (int n=0; n<4; n++){
    SPI.transfer(R0[n]);
  }
  digitalWrite(LE,HIGH);
  digitalWrite(LE,LOW);
}

//process incoming serial data and check if data are valid and write default into EEPROM
//test(0,1300,1300,1,1)(1,5100,5100,1,1)(1,5100,5150,10,1000)(1,5150,5100,10,1000)(0,1300,1350,10,1000)(0,1350,1300,10,1000)e
//test2 (0,1340,1340,1,1)(1,5160,5160,1,1)e
//(0,1150,1150,1,1)(1,1175,1175,1,1)e
//test3 (0,1340,1340,1,1)(0,1340,1300,1000,2000)(0,1300,1340,1000,2000)e
void processData(){
  float para[5];
  uint32_t paraint0[40][5];
  uint32_t paraint1[40][5];
  uint32_t rampCount0 = 0;
  uint32_t rampCount1 = 0;

  char rc = 0;
  bool error = true;   //error flag for input parameter
  elapsedMillis serialTimeout;  //timeout for serial receive in case of faliur
  
  //process all serial data until the end marker is received
  serialTimeout = 0;
  while (rc != endendMarker && serialTimeout != 10000){
    if (Serial.available()){
      rc = Serial.read();
    }
    if (rc == startMarker){
      //Take the first 5 float from serial command
      for (int i=0; i<5; i++){
        para[i] = Serial.parseFloat();
      }
      //convert float into int and transfer to parameter buffer also check the data validity
      if ((int)para[0] == 0){
        paraint0[rampCount0][0] = para[0];    //ch
        paraint0[rampCount0][1] = para[1]*1000;    //freq start in kHz
        paraint0[rampCount0][2] = para[2]*1000;    //freq stop in kHz
        paraint0[rampCount0][3] = para[3];    //step number
        paraint0[rampCount0][4] = para[4]*1000;    //step length time in us
        error = dataCheck(paraint0[rampCount0]);
        rampCount0++;
      }
      else if ((int)para[0] == 1){
        paraint1[rampCount1][0] = para[0];    //ch
        paraint1[rampCount1][1] = para[1]*1000;    //freq start in kHz
        paraint1[rampCount1][2] = para[2]*1000;    //freq stop in kHz
        paraint1[rampCount1][3] = para[3];    //step number
        paraint1[rampCount1][4] = para[4]*1000;    //step length time in us
        error = dataCheck(paraint1[rampCount1]);
        rampCount1++;
      }
    }
  }
  //check if there are errors in the data or the serial communication timeout is reached
  if (error == false && serialTimeout < 10000){
    //transfer local buffered parameter to global parameter in the main program
    for (int i=0; i<rampCount0; i++){
      for(int j=0; j<4; j++){
        ramp0[i][j] = paraint0[i][j+1];
      }
      ramp0[i][4] = (ramp0[i][1]-ramp0[i][0])/ramp0[i][2];
      ramp0[i][5] = ramp0[i][3]/ramp0[i][2];
    }
    for (int i=0; i<rampCount1; i++){
      for(int j=0; j<4; j++){
        ramp1[i][j] = paraint1[i][j+1];
      }
      ramp1[i][4] = (ramp1[i][1]-ramp1[i][0])/ramp1[i][2];
      ramp1[i][5] = ramp1[i][3]/ramp1[i][2];
    }
    rampCounter0 = rampCount0;
    rampCounter1 = rampCount1;
    rampflg0 = 0;
    rampflg1 = 0;
    // EEPROM.updateLong(address,ramp0[0][0]);
    // EEPROM.updateLong(address+5,ramp1[0][0]);
    //write ch0 R1,R0
    updatePFD(calcFTW(EEPROM.readLong(address)),cs0);
    //write ch1 R1,R0
    updatePFD(calcFTW(EEPROM.readLong(address+5)),cs1);
    char buff[64];
    sprintf(buff, "Parameter programmed in :%.2f ms.", (double)serialTimeout );
    Serial.println(buff);
    // Serial.print("Parameter programmed in :");
    // Serial.print(serialTimeout);
    // Serial.println(" ms");
    // Serial.print("f");
  }
  else{
    Serial.println("Error in paramter!! Nothing changed.");
    // Serial.print("f");
  }
}

//check if input data are in range and valid
bool dataCheck(uint32_t par[5]){
  bool errorflg = true;
  if (par[0] == 0 || par[0] == 1 ){
    if (par[1] >= 1000000 && par[1] <= 6000000 && par[2] >= 1000000 && par[2] <= 6000000){
      if (par[3] > 0 && par[4]/par[3] >= 20){
        errorflg = false;
      }
      else{
        errorflg = true;
      }
    }
    else{
      errorflg = true;
    }
  }
  else{
    errorflg = true;
  }
  return errorflg;
}

//ISR for ch0 ramp when the trigger is detected
void setRamp0(){
  rampDelay0 = 0;
  rampflg0++;
  stepCount0 = 0;
}

//ISR for ch1 ramp when the trigger is detected
void setRamp1(){
  rampDelay1 = 0;
  rampflg1++;
  stepCount1 = 0;
}
