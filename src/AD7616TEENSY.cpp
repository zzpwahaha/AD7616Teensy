/****************************************************************************************

 ***************************************************************************************/
// C++ includes
#include <algorithm>
#include <cstdio>
#include <utility>
#include <vector>
// Teensy includes
#include <Arduino.h>
#include <SPI.h>
#include <EEPROMex.h>
#include <stdio.h>
// #include <NativeEthernet.h>
// #include <QNEthernet.h>
// #include <../lib/NativeEthernet/src/NativeEthernet.h>

#include <QNEthernet.h>
using namespace qindesign::network;

/******************************************************VARIABLES*************************************************************/
#undef AD7616DEBUG
bool debug = false;
// note for debuging:
// For serial port debugging with out AD7616DEBUG, the command does not need to be appended with terminator, the terminator is only needed in TCP mode
// The command need to be send in hex, which is easier use h-term and convert the ascii part of the command to hex
// e.g. (rng,\xFD\xFF\xFF\xFF) -> 28 72 6E 67 2C FD FF FF FF 29 
//      (*\x01\x01,100) -> 28 2A 01 01 2C 31 30 30 29
//      (trg, ) -> 28 74 72 67 2C 20 29
// With the above three commands, should be able to read the A0 channel with 2.5V range and 100 repetition


/*interpreting input command*/
const char STARTM = '(';        // start marker for each set of command data
const char SEPAR = ',';         // separator within one set of command data
const char ENDM = ')';          // end marker for each set of command data
const char TERMINATOR = '#';    // terminator for tcp socket connection, the command data in should end with TERMINATOR + '\0'
const char PLACEHOLDER = '*';   // placeholder for sending command, if send in (*xx,xxxx), it will go as channel conversion, otherwise (xxx,xxxx) will go as command
const short CMDHEADSIZE = 3;    // size of the command head, ie size of 'xxx' before the separator from (xxx,xxxx)
const short MAXIN = 128*6;        // maximum number of chars in input stream
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
EXTMEM short ADCDATA[1<<22 /*2^22*/];
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
// byte mac[] = {0x04,0xe9,0xe5,0x0e,0x0c,0xe0};
// The DHCP timeout, in milliseconds. Set to zero to not wait and instead rely on the listener to inform us of an address assignment.
// using static ip, so can just ignore DHCP part. Keeping everything for future convenience
constexpr uint32_t kDHCPTimeout = 10000;  // 10 seconds
// The link timeout, in milliseconds. Set to zero to not wait and instead rely on the listener to inform us of a link.
constexpr uint32_t kLinkTimeout = 5000;  // 5 seconds
// Timeout for waiting for input from the client.
constexpr uint32_t kClientTimeout = 5000;  // 5 seconds
// Timeout for waiting for a close from the client after a half close.
constexpr uint32_t kShutdownTimeout = 30000;  // 30 seconds

// Set the static IP to something other than INADDR_NONE (zero)
// to not use DHCP. The values here are just examples.
IPAddress staticIP{10, 10, 0, 10};
IPAddress subnetMask{255, 255, 255, 0};
IPAddress gateway{0, 0, 0, 0};
constexpr uint16_t kServerPort = 80;
// The server.
EthernetServer server{kServerPort};

// Keeps track of state for a single client.
struct ClientState {
  ClientState(EthernetClient client)
      : remoteIP{client.remoteIP()},
        remotePort(client.remotePort()),
        client(std::move(client)) {}

  // Put these before the moved client
  IPAddress remoteIP;
  uint16_t remotePort;

  EthernetClient client;
  bool closed = false;
  // For timeouts.
  uint32_t lastRead = millis();  // Mark creation time
  // For half closed connections, after "Connection: close" was sent and closeOutput() was called
  uint32_t closedTime = 0;    // When the output was shut down
  bool outputClosed = false;  // Whether the output was shut down
};
std::vector<ClientState> clients; // more than one client can be connected at one time
std::vector<String> clientInputs; // incoming commands from all clients, will report error is this vector is ever larger than one
// ClientState nullClient(EthernetClient());
ClientState* clientWriter = nullptr; // used to store the connected client and write data to it


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
void serverUpdate(bool hasIP) ; // Tell the server there's been an IP address change. IMPORTANT: the server is started in this function
bool listenForEthernetClients(String& rc);

void resetSeqTEENSY();
void writeSeq(unsigned short index);

/********************************************************************************************************************************/
//initialization of hardware
void setup() 
{  
  // Initializing USB serial to 12Mbit/sec. Teensy ignores the 9600 baud rate. see https://www.pjrc.com/teensy/td_serial.html
  Serial.begin(9600);
  while (!Serial && millis() < 4000) {
    // Wait for Serial to initialize
  }

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
    Serial.println("Reading from serial: " + rc);
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
  if (clientWriter->client) {
    Serial.printf("startReadADC: The client is %s and is %s\r\n", 
    clientWriter->client.connected() ? "Connected" : "Disconnected",
    clientWriter->client.availableForWrite() ? "Available for write" : "Not available for write"); 
    // return;
  }
  Serial.println("Start to read ADC");
  if (seqTotal==0){
    Serial.println("Error: sequence number is zero, no sequence is armed for trigger");
    clientWriter->client.println("Error: sequence number is zero, no sequence is armed for trigger");
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
  }
  sd.append((char)(ADCDATA[DATASIZE-1]>>8));
  sd.append((char)(ADCDATA[DATASIZE-1]&0xff));
  
  clientWriter->client.writeFully(sd.c_str(),DATASIZE*2);

  // Serial.println(sd);
  Serial.printf("Total send time is %d us and %.2f us per 2 byte data \r\n", (unsigned long)timer, double(timer)/DATASIZE);

}



void initTCP()
{
  stdPrint = &Serial;  // Make printf work (a QNEthernet feature)
  printf("Starting...\r\n");

  /*Unlike the Arduino API (which you can still use), QNEthernet uses the Teensy's internal MAC address by default, so we can retrieve it here*/
  uint8_t mac[6];
  Ethernet.macAddress(mac);  // This is informative; it retrieves, not sets
  printf("MAC = %02x:%02x:%02x:%02x:%02x:%02x\r\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /*Add listeners. It's important to add these before doing anything with Ethernet, so no events are missed.*/
  /*Listen for link changes*/
  Ethernet.onLinkState([](bool state) {
    printf("[Ethernet] Link %s\r\n", state ? "ON" : "OFF");
  });
  /*Listen for address changes*/
  Ethernet.onAddressChanged([]() {
    IPAddress ip = Ethernet.localIP();
    bool hasIP = (ip != INADDR_NONE);
    if (hasIP) {
      printf("[Ethernet] Address changed:\r\n");
      IPAddress ip = Ethernet.localIP();
      printf("    Local IP = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.subnetMask();
      printf("    Subnet   = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.gatewayIP();
      printf("    Gateway  = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.dnsServerIP();
      if (ip != INADDR_NONE) {  // May happen with static IP
        printf("    DNS      = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
      }
    } else {
      printf("[Ethernet] Address changed: No IP address\r\n");
    }
    /*Tell interested parties the state of the IP address, for example, servers, SNTP clients, and other sub-programs that need to know whether 
    to stop/start/restart/etc. Note: When setting a static IP, the address will be set but a link might not yet exist*/
    serverUpdate(hasIP);
  });

  // using static ip, so can just ignore DHCP part. Keeping everything for future convenience
  if (staticIP == INADDR_NONE) {
    printf("Starting Ethernet with DHCP...\r\n");
    if (!Ethernet.begin()) {
      printf("Failed to start Ethernet\r\n");
      return;
    }
    // We can choose not to wait and rely on the listener to tell us when an address has been assigned
    if (kDHCPTimeout > 0) {
      if (!Ethernet.waitForLocalIP(kDHCPTimeout)) {
        printf("Failed to get IP address from DHCP\r\n");
        // We may still get an address later, after the timeout,
        // so continue instead of returning
      }
    }
  } else {
    printf("Starting Ethernet with static IP...\r\n");
    Ethernet.begin(staticIP, subnetMask, gateway);
    // When setting a static IP, the address is changed immediately, but the link may not be up; optionally wait for the link here
    if (kLinkTimeout > 0) {
      if (!Ethernet.waitForLink(kLinkTimeout)) {
        printf("Failed to get link\r\n");
        // We may still see a link later, after the timeout, so continue instead of returning
      }
    }
  }

}

void serverUpdate(bool hasIP) {
  // If there's no IP address, could optionally stop the server, depending on your needs
  if (hasIP) {
    if (server) {
      // Optional
      printf("Address changed: Server already started\r\n");
    } else {
      printf("Starting server on port %u...", kServerPort);
      fflush(stdout); // Print what we have so far if line buffered
      server.begin(); // SERVER STARTED HERE, IMPORTANT!!!
      printf("%s\r\n", server ? "done." : "FAILED!");
    }
  } else {
    // Stop the server if there's no IP address
    if (!server) {
      // Optional
      printf("Address changed: Server already stopped\r\n");
    } else {
      printf("Stopping server...");
      fflush(stdout);  // Print what we have so far if line buffered
      printf("%s\r\n", server.end() ? "done." : "FAILED!");
    }
  }
}

bool listenForEthernetClients(String& rc) 
{
  bool success = false;
  clientInputs.clear();
  EthernetClient client = server.accept();
  if (client) {
    // We got a connection!
    IPAddress ip = client.remoteIP();
    uint16_t port = client.remotePort();
    printf("Client connected: IP %u.%u.%u.%u, port %d\r\n", ip[0], ip[1], ip[2], ip[3], port);
    clients.emplace_back(client); // maybe do not need to use std::move(client) here since it is moved in the constructor
    printf("Client count: %u\r\n", clients.size());
  }

  // Process data from each client
  for (ClientState &state : clients) {  // Use a reference so we don't copy
    if (!state.client.connected()) {
      state.closed = true;
      continue;
    }
    // Check if we need to force close the client
    if (state.outputClosed) {
      if (millis() - state.closedTime >= kShutdownTimeout) {
        IPAddress ip = state.client.remoteIP();
        printf("Client shutdown timeout: %u.%u.%u.%u\n",
                ip[0], ip[1], ip[2], ip[3]);
        state.client.stop();
        state.closed = true;
        continue;
      }
    } else {
      if (millis() - state.lastRead >= kClientTimeout) {
        IPAddress ip = state.client.remoteIP();
        printf("Client timeout: %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
        state.client.stop();
        state.closed = true;
        continue;
      }
    }

    // try to read data from client
    uint8_t buff[MAXIN];
    int actualSize = state.client.read(buff, MAXIN);
    // try to format the data if there is any
    if (actualSize>0) { 
      if (buff[actualSize-1] != TERMINATOR){
        printf("Received from TCP: %s\r\n", reinterpret_cast<char*>(buff));
        printf("Error in reading TCP command: size=%d, end character is %c, but should be %c\r\n", actualSize, buff[actualSize-1],TERMINATOR);
        state.client.printf("Error in reading TCP command: size=%d, end character is %c, but should be %c\r\n", actualSize, buff[actualSize-1],TERMINATOR);
        success = false;
      }
      else if (server.available()){
        printf("Received from TCP: %s\r\n", reinterpret_cast<char*>(buff));
        printf("Error in reading TCP command: Command is exceeding max size: %d, or command is sent in too fast\r\n", MAXIN);
        state.client.printf("Error in reading TCP command: Command is exceeding max size: %d, or command is sent in too fast\r\n", MAXIN);
        success = false;
      }
      else{
        buff[actualSize-1] = 0; // remove TERMINATOR by changing the TERMINATOR to null 
        clientInputs.push_back(String(reinterpret_cast<char*>(buff)));
        rc = clientInputs.back();
        clientWriter = &state;
        Serial.println("************************************************************************");
        Serial.println("Command from TCP socket " + clientInputs.back() + " of size: " + String(actualSize));
        success = true;

        // char s[] = "Server write test";  // server.write does not work
        // server.write(reinterpret_cast<uint8_t*>(s), strlen(s));
        // should use state.client.printf, state.client.write, state.client.writeFully, state.client.println, etc
      }
    }
  }
  // check if there is only one input command, report error otherwise
  if (clientInputs.size()>1) {
    printf("Error in connected client: the client size %d is larger than 1, which is fine. But the program can only accept one client command at a time for the data income, instead of %d. Maybe try to give sometime in sending two commands. \r\n", 
      clients.size(), clientInputs.size());
    printf("The commands are: \r\n");
    for (auto &s : clientInputs) {
      printf("%s\r\n", s.c_str());
    }
    for (ClientState &state : clients) {  // Use a reference so we don't copy
      if (!state.client.connected()) {
        state.closed = true;
        continue;
      }
      state.client.printf("Error in connected client: the client size %d is larger than 1, which is fine. But the program can only accept one client command at a time for the data income, instead of %d. Maybe try to give sometime in sending two commands \r\n", 
      clients.size(), clientInputs.size());
      state.client.printf("The commands are: \r\n");
      for (auto &s : clientInputs) {
        state.client.printf("%s\r\n", s.c_str());
      }
    }
    success = false;
  }

  // Clean up all the closed clients
  size_t size = clients.size();
  clients.erase(std::remove_if(clients.begin(), clients.end(),
                               [](const auto &state) { return state.closed; }),
                clients.end());
  if (clients.size() != size) {
    printf("New client count: %u\r\n", clients.size());
  }

  return success;
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
    else if (rc.compareTo("serverPrint") == 0){
      // The server print would not show anything, since there is not client connect to it and it therefore does not know who to send to 
      // even if there is client connected to it, shoul use client.write to transfer data to client
      server.println("Serve try to print. Should be able to see in WireShark");
      server.flush();
      server.printf("Serve try to print. Should be able to see in WireShark");
      server.flush();
      server.write("Success start with write");
      server.flush();
      uint8_t buf[] = "Success start with NativeEthernetServer::write";
      server.write(buf, sizeof(buf)/sizeof(buf[0]));
      server.flush();
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
  if (clientWriter->client) {
    Serial.printf("interpretCmd: The client is %s and is %s\r\n", 
      clientWriter->client.connected() ? "Connected" : "Disconnected",
      clientWriter->client.availableForWrite() ? "Available for write" : "Not available for write"); 
    // return;
  }

  bool firstSeq = true;
  unsigned short charcnts = 0;
  unsigned short numSEQ = 0;
  byte chnl[CMDHEADSIZE + 1];

  while (charcnts < rc.length())
  {
    if (rc[charcnts++]!=STARTM) {
      Serial.println("Error: Start terminator \"" + String(STARTM) + "\" is missing");
      clientWriter->client.println("Error: Start terminator \"" + String(STARTM) + "\" is missing");
      resetSeqTEENSY();
      return;
    }
    rc.getBytes(chnl, CMDHEADSIZE+1, charcnts); // the last character is always '\0' by String::getBytes
    if (chnl[0]==0 || chnl[1]==0){
      Serial.println("Error: Encounter zero in channel selection");
      clientWriter->client.println("Error: Encounter zero in channel selection");
      resetSeqTEENSY();
      return;
    }
    // Serial.println((char*)chnl);
    charcnts+=CMDHEADSIZE;
    if (rc[charcnts++]!=SEPAR) {
      Serial.println("Error: Separator \"" + String(SEPAR) + "\" is missing");
      clientWriter->client.println("Error: Separator \"" + String(SEPAR) + "\" is missing");
      resetSeqTEENSY();
      return;
    }
    short endpos = rc.indexOf(ENDM, charcnts); // endposition points to END MARKER
    String contents = rc.substring(charcnts,endpos); // not including endposition, the endposition is filled with '\0' in the substring
    charcnts = endpos;
    // Serial.println(rc[charcnts]);
    if (rc[charcnts++]!=ENDM) {
      Serial.println("Error: End terminator \"" + String(ENDM) + "\" is missing");
      clientWriter->client.println("Error: End terminator \"" + String(ENDM) + "\" is missing");
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
      clientWriter->client.printf("Error: write range register with 0x%02X 0x%02X 0x%02X 0x%02X is invalid\r\n", content[0],content[1],content[2],content[3]);
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
    clientWriter->client.println("Success in writing range register");
    Serial.println("Success in writing range register");
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

