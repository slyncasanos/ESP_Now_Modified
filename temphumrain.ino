/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-many-to-one-esp32/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <esp_now.h>
#include <WiFi.h>
#include "DHT.h"
#include "RTClib.h"
#include <Wire.h>

#define DHTPIN 4 
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
#define RainPin 2  

bool bucketPositionA = false;             // one of the two positions of tipping-bucket               
const double bucketAmount = 0.4090909;   // inches equivalent of ml to trip tipping-bucket
double dailyRain = 0.0;                   // rain accumulated for the day
double minuteRain = 0.0;                  // rain accumulated for one hour
double dailyRain_till_LastMinute = 0.0;     // rain accumulated for the day till the last hour          
bool first;                               // as we want readings of the (MHz) loops only at the 0th moment 

RTC_Millis rtc;

// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0x10, 0x52, 0x1C, 0x64, 0xEF, 0x6C};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int id; // must be unique for each sender board
    float x;
    float y;
    float z;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  dht.begin();
  rtc.begin(DateTime(__DATE__, __TIME__));       // start the RTC
  pinMode(RainPin, INPUT);  

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {

DateTime now = rtc.now();

// ++++++++++++++++++++++++ Count the bucket tips ++++++++++++++++++++++++++++++++
  if ((bucketPositionA==false)&&(digitalRead(RainPin)==HIGH)){
    bucketPositionA=true;
    dailyRain+=bucketAmount;                               // update the daily rain
  }
  
  if ((bucketPositionA==true)&&(digitalRead(RainPin)==LOW)){
    bucketPositionA=false;  
  } 

  if(now.second() != 0) first = true; 

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.println(F("°C "));

if(now.second() == 0 && first == true) {
  minuteRain = dailyRain - dailyRain_till_LastMinute;      // calculate the last hour's rain
  dailyRain_till_LastMinute = dailyRain;                   // update the rain till last hour for next calculation

  // facny display for humans to comprehend
  Serial.print("Rain in last minute = ");
  Serial.print(minuteRain,2);
  Serial.println(" mm");
    
  first = false;                                        // execute calculations only once per hour
    
  if(now.minute()== 0) {
  dailyRain = 0.0;                                      // clear daily-rain at midnight
  dailyRain_till_LastMinute = 0.0;                        // we do not want negative rain at 01:00
  }
}

  // Set values to send
  myData.id = 1;
  myData.x = t;
  myData.y = h;
  myData.z = minuteRain;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(10000);

  // Deep sleep for 10 seconds using timer
  esp_sleep_enable_timer_wakeup(10 * 1000000); // 10 seconds
  esp_deep_sleep_start();
}
