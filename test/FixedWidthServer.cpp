// SPDX-FileCopyrightText: (c) 2021-2022 Shawn Silverman <shawn@pobox.com>
// SPDX-License-Identifier: MIT

// FixedWidthServer demonstrates how to serve a protocol having a
// continuous stream of fixed-size messages from multiple clients.
//
// This file is part of the QNEthernet library.

// C++ includes
#include <algorithm>
#include <cstdio>
#include <utility>
#include <vector>

#include <QNEthernet.h>

using namespace qindesign::network;

// --------------------------------------------------------------------------
//  Configuration
// --------------------------------------------------------------------------

constexpr uint32_t kDHCPTimeout = 10000;  // 10 seconds
constexpr uint16_t kServerPort = 80;
constexpr int kMessageSize = 10;  // Pretend the protocol specifies 10 bytes

// The link timeout, in milliseconds. Set to zero to not wait and
// instead rely on the listener to inform us of a link.
constexpr uint32_t kLinkTimeout = 5000;  // 5 seconds

IPAddress staticIP{10, 10, 0, 10};//{192, 168, 1, 101};
IPAddress subnetMask{255, 255, 255, 0};
IPAddress gateway{0, 0, 0, 0};
IPAddress dnsServer = gateway;

// --------------------------------------------------------------------------
//  Types
// --------------------------------------------------------------------------

// Keeps track of state for a single client.
struct ClientState {
  ClientState(EthernetClient client)
      : client(std::move(client)) {}

  EthernetClient client;
  int bufSize = 0;  // Keeps track of how many bytes have been read
  uint8_t buf[kMessageSize];
  bool closed = false;
};

// --------------------------------------------------------------------------
//  Program state
// --------------------------------------------------------------------------

// Keeps track of what and where belong to whom.
std::vector<ClientState> clients;

// The server.
EthernetServer server{kServerPort};

// --------------------------------------------------------------------------
//  Main program
// --------------------------------------------------------------------------

// Forward declarations
void systemReady(bool hasIP, bool hasLink);


// Program setup.
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 10000) {
    // Wait for Serial to initialize
  }
  stdPrint = &Serial;  // Make printf work (a QNEthernet feature)
  printf("Starting...\n");


  // Add listeners
  // It's important to add these before doing anything with Ethernet
  // so no events are missed.

  // Listen for link changes
  Ethernet.onLinkState([](bool state) {
    printf("[Ethernet] Link %s\n", state ? "ON" : "OFF");

    // When setting a static IP, the address will be set but a link
    // might not yet exist
    bool hasIP = (Ethernet.localIP() != INADDR_NONE);
    systemReady(hasIP, state);
  });

  // Listen for address changes
  Ethernet.onAddressChanged([]() {
    IPAddress ip = Ethernet.localIP();
    bool hasIP = (ip != INADDR_NONE);
    if (hasIP) {
      IPAddress ip = Ethernet.localIP();
      IPAddress subnet = Ethernet.subnetMask();
      IPAddress gw = Ethernet.gatewayIP();
      IPAddress dns = Ethernet.dnsServerIP();
      // Note: In order for the DNS address to not be INADDR_NONE
      //       (zero) when setting a static IP, it must be set first

      printf(
          "[Ethernet] Address changed:\n"
          "    Local IP = %u.%u.%u.%u\n"
          "    Subnet   = %u.%u.%u.%u\n"
          "    Gateway  = %u.%u.%u.%u\n"
          "    DNS      = %u.%u.%u.%u\n",
          ip[0], ip[1], ip[2], ip[3],
          subnet[0], subnet[1], subnet[2], subnet[3],
          gw[0], gw[1], gw[2], gw[3],
          dns[0], dns[1], dns[2], dns[3]);
    } else {
      printf("[Ethernet] Address changed: No IP address\n");
    }

    // Tell interested parties the state of the IP address and system
    // readiness, for example, servers, SNTP clients, and other
    // sub-programs that need to know whether to stop/start/restart/etc
    // Note: When setting a static IP, the address will be set but a
    //       link might not yet exist
    systemReady(hasIP, Ethernet.linkState());
  });



  // Unlike the Arduino API (which you can still use), QNEthernet uses
  // the Teensy's internal MAC address by default, so we can retrieve
  // it here
  uint8_t mac[6];
  Ethernet.macAddress(mac);  // This is informative; it retrieves, not sets
  printf("MAC = %02x:%02x:%02x:%02x:%02x:%02x\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if (staticIP == INADDR_NONE){
    printf("Starting Ethernet with DHCP...\n");
    if (!Ethernet.begin()) {
      printf("Failed to start Ethernet\n");
      return;
    }
    if (!Ethernet.waitForLocalIP(kDHCPTimeout)) {
      printf("Failed to get IP address from DHCP\n");
    }
    else {
      IPAddress ip = Ethernet.localIP();
      printf("    Local IP    = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.subnetMask();
      printf("    Subnet mask = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.gatewayIP();
      printf("    Gateway     = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.dnsServerIP();
      printf("    DNS         = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);

      // Start the server
      printf("Listening for clients on port %u...\n", kServerPort);
      server.begin();
    }
  }
  else{
      printf("Starting Ethernet with static IP...\n");
      // Ethernet.setDNSServerIP(dnsServer);  // Set first so that the
                                          // listener sees it
      Ethernet.begin(staticIP, subnetMask, gateway);
      
      IPAddress ip = Ethernet.localIP();
      printf("    Local IP    = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.subnetMask();
      printf("    Subnet mask = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.gatewayIP();
      printf("    Gateway     = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.dnsServerIP();
      printf("    DNS         = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);

      // Start the server
      printf("Listening for clients on port %u...\n", kServerPort);
      server.begin();
      // When setting a static IP, the address is changed immediately,
      // but the link may not be up; optionally wait for the link here
      if (kLinkTimeout > 0) {
        if (!Ethernet.waitForLink(kLinkTimeout)) {
          printf("Warning: No link detected\n");
          // We may still see a link later, after the timeout, so
          // continue instead of returning
        }
      }
  }

}

// Process one message. This implementation simply dumps to Serial.
//
// We could pass just the buffer, but we're passing the whole state
// here so we know which client it's from.
void processMessage(const ClientState &state) {
  printf("Message: ");
  fwrite(state.buf, sizeof(state.buf[0]), kMessageSize, stdout);
  printf("\n");
}

// This is called when the system readiness has changed. The system is
// considered ready if there's an IP address and the link is up.
void systemReady(bool hasIP, bool hasLink) {
  printf("System is%s ready\n", (hasIP && hasLink) ? "" : " not");

  // *** Notification or start/stop/restart code goes here

  // For servers, it is suggested to follow the address state because
  // they can be brought up and active even when there's no link,
  // unlike clients and connections, which require both an address and
  // a link.
}


// Main program loop.
void loop() {
  EthernetClient client = server.accept();
  if (client) {
    IPAddress ip = client.remoteIP();
    printf("Client connected: %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
    clients.emplace_back(std::move(client));
    printf("Client count: %u\n", clients.size());
  }

  // Process data from each client
  for (ClientState &state : clients) {  // Use a reference
    if (!state.client.connected()) {
      state.closed = true;
      continue;
    }

    int avail = state.client.available();
    if (avail > 0) {
      int toRead = std::min(kMessageSize - state.bufSize, avail);
      state.bufSize += state.client.read(&state.buf[state.bufSize], toRead);
      if (state.bufSize >= kMessageSize) {
        processMessage(state);
        state.bufSize = 0;
      }
    }

  }

  // IPAddress ip = state.client.remoteIP();
  // printf("Sending to client: %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
  // state.client.writeFully("HTTP/1.1 200 OK\r\n"
  //                         "Connection: close\r\n"
  //                         "Content-Type: text/plain\r\n"
  //                         "\r\n"
  //                         "Hello, Client!\r\n");
  // state.client.flush();

  // // Half close the connection, per
  // // [Tear-down](https://datatracker.ietf.org/doc/html/rfc7230#section-6.6)
  // state.client.closeOutput();
  // // state.closedTime = millis();
  // // state.outputClosed = true;

  // Clean up all the closed clients
  size_t size = clients.size();
  clients.erase(std::remove_if(clients.begin(), clients.end(),
                               [](const auto &state) { return state.closed; }),
                clients.end());
  if (clients.size() != size) {
    printf("Client count: %u\n", clients.size());
  }
}
