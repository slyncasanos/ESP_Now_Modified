/*********
  Modified from the examples of the Arduino LoRa library
  More resources: https://randomnerdtutorials.com
*********/
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>

//define the pins used by the transceiver module
#define ss 18
#define rst 14
#define dio0 26

int counter = 0;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int id;
  float x;
  float y;
  float z;
}struct_message;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
struct_message board3;

// Create an array with all the structures
struct_message boardsStruct[3] = {board1, board2, board3};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[myData.id-1].x = myData.x;
  boardsStruct[myData.id-1].y = myData.y;
  boardsStruct[myData.id-1].z = myData.z;
  boardsStruct[myData.id-2].x = myData.x;
  boardsStruct[myData.id-2].y = myData.y;

    // Update the structure with the new incoming data
  if (myData.id == 1) {
    boardsStruct[0].x = myData.x;
    boardsStruct[0].y = myData.y;
    boardsStruct[0].z = myData.z;
    Serial.printf("Board 1 - Temperature: %.2f °C\n", boardsStruct[0].x);
    Serial.printf("Board 1 - Humidity: %.2f %%\n", boardsStruct[0].y);
    Serial.printf("Board 1 - Rain in last minute: %.2f mm\n", boardsStruct[0].z); 
  } else if (myData.id == 2) {
    boardsStruct[1].x = myData.x;
    boardsStruct[1].y = myData.y;
    Serial.printf("Board 2 - Flow Rate: %.2f L/min\n", boardsStruct[1].x);
    Serial.printf("Board 2 - Water Level: %.2f cm\n", boardsStruct[1].y);
  } else {
    // Handle other boards if needed
    Serial.println("Unknown board ID");
  }
}

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(866E6)) {
    Serial.println(".");
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.println("hello ");
   if (myData.id == 1) {
  LoRa.printf("Board 1 - Temperature: %.2f °C\n", boardsStruct[0].x);
  float board1X = boardsStruct[0].x;
  LoRa.printf("Board 1 - Humidity: %.2f %%\n", boardsStruct[0].y);
  float board1Y = boardsStruct[0].y;
  LoRa.printf("Board 1 - Rain in last minute: %.2f mm\n", boardsStruct[0].z);
  float board1Z = boardsStruct[0].z;
  } else if (myData.id == 2) {
  LoRa.printf("Board 2 - Flow Rate: %.2f L/min\n", boardsStruct[1].x);
  float board2X = boardsStruct[1].x;
  LoRa.printf("Board 2 - Water Level: %.2f cm\n", boardsStruct[1].y);
  float board2Y = boardsStruct[1].y;
  } else {
    // Handle other boards if needed
    Serial.println("Unknown board ID");
  }
  LoRa.print(counter);
    LoRa.endPacket();

  counter++;

  delay(10000);
}
