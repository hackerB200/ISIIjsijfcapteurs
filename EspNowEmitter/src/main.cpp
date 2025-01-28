#include <SPI.h>
#include <MFRC522.h>
#include <esp_now.h>
#include <WiFi.h>

// PIN Numbers : RESET + SDA
#define RST_PIN 21
#define SS_1_PIN 5

// Wifi
const char *ssid = "iot";
const char *password = "iotisis;";

// set pin numbers
const int redLedPin = 26;   // RED LED
const int greenLedPin = 25; // GREEN LED

// Create an MFRC522 instance
MFRC522 mfrc522;

// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0x24, 0xdc, 0xc3, 0x14, 0x3d, 0x94};

// Sending data
String rfid_message;

// RECEIVING data
boolean openTheDoor = false;

esp_now_peer_info_t peerInfo;

/**
   Helper routine to dump a byte array as hex values to Serial.
*/
String dump_byte_array(byte *buffer, byte bufferSize)
{
  String result = "";
  for (byte i = 0; i < bufferSize; i++)
  {
    if (buffer[i] < 0x10)
    {
      result += "0";
    }
    result += String(buffer[i], HEX);
  }
  Serial.print(F(": Card UID:"));
  Serial.println(result);
  return result;
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.print("To: ");
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print("\tDelivery Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  memcpy(&openTheDoor, incomingData, sizeof(openTheDoor));
  Serial.print("Received data from: ");
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);

  Serial.print("Door Status: ");
  if (openTheDoor)
  {
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(redLedPin, LOW);
    Serial.println("Autorized to open the door");
  }
  else
  {
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
    Serial.println("WARNING : Not autorized to open the door");
  }
  delay(500);
  digitalWrite(greenLedPin, LOW);
  digitalWrite(redLedPin, LOW);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);

  SPI.begin();

  mfrc522.PCD_Init(SS_1_PIN, RST_PIN);
  mfrc522.PCD_DumpVersionToSerial();

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Setting as a Wi-Fi Station..");
  }
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
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
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop()
{
  // Looking for new cards
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial())
  {
    String uid_str = dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
    Serial.println(uid_str);
    // Send RFID data via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&uid_str, sizeof(uid_str));

    if (result == ESP_OK)
    {
      Serial.println("Sent with success");
    }
    else
    {
      Serial.println("Error sending the data");
    }

    // Halt PICC
    mfrc522.PICC_HaltA();
    // Stop encryption on PCD
    mfrc522.PCD_StopCrypto1();
  }
}

// #include <Arduino.h>
// /*********
//   Rui Santos & Sara Santos - Random Nerd Tutorials
//   Complete project details at https://RandomNerdTutorials.com/esp-now-many-to-one-esp32/
//   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
//   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// *********/
// #include <esp_now.h>
// #include <WiFi.h>

// // REPLACE WITH THE MAC Address of your receiver
// uint8_t broadcastAddress[] = {0x24, 0xdc, 0xc3, 0x14, 0x3d, 0x94};

// // Define variables to store BME280 readings to be sent
// float temperature;
// float humidity;
// float pressure;

// // Define variables to store incoming readings
// float incomingTemp;
// float incomingHum;
// float incomingPres;

// // Variable to store if sending data was successful
// String success;

// // Structure example to receive data
// // Must match the sender structure
// typedef struct struct_message
// {
//   float temp;
//   float hum;
//   float pres;
//   unsigned int readingId;
// } struct_message;

// // Create a struct_message called BME280Readings to hold sensor readings
// struct_message BME280Readings;

// // Create a struct_message to hold incoming sensor readings
// struct_message incomingReadings;

// esp_now_peer_info_t peerInfo;

// // Callback when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
// {
//   Serial.print("\r\nLast Packet Send Status:\t");
//   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//   if (status == 0)
//   {
//     success = "Delivery Success :)";
//   }
//   else
//   {
//     success = "Delivery Fail :(";
//   }
// }

// // Callback when data is received
// void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
// {
//   memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
//   Serial.println("Received data from: ");
//   char macStr[18];
//   snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
//            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//   Serial.print(macStr);
//   Serial.print("Bytes received: ");
//   Serial.println(len);
//   Serial.print("Temperature: ");
//   Serial.println(incomingReadings.temp);
//   Serial.print("Humidity: ");
//   Serial.println(incomingReadings.hum);
//   Serial.print("Pressure: ");
//   Serial.println(incomingReadings.pres);
//   incomingTemp = incomingReadings.temp;
//   incomingHum = incomingReadings.hum;
//   incomingPres = incomingReadings.pres;
// }

// void setup()
// {
//   // Init Serial Monitor
//   Serial.begin(115200);

//   // Set device as a Wi-Fi Station
//   WiFi.mode(WIFI_STA);

//   // Init ESP-NOW
//   if (esp_now_init() != ESP_OK)
//   {
//     Serial.println("Error initializing ESP-NOW");
//     return;
//   }

//   // Once ESPNow is successfully Init, we will register for Send CB to
//   // get the status of Trasnmitted packet
//   esp_now_register_send_cb(OnDataSent);

//   // Register peer
//   memcpy(peerInfo.peer_addr, broadcastAddress, 6);
//   peerInfo.channel = 0;
//   peerInfo.encrypt = false;

//   // Add peer
//   if (esp_now_add_peer(&peerInfo) != ESP_OK)
//   {
//     Serial.println("Failed to add peer");
//     return;
//   }
//   // Register for a callback function that will be called when data is received
//   esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
// }

// void loop()
// {
//   temperature = 100;
//   humidity = 500;
//   pressure = 10000;

//   // Set values to send
//   BME280Readings.temp = temperature;
//   BME280Readings.hum = humidity;
//   BME280Readings.pres = pressure;

//   // Send message via ESP-NOW
//   esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&BME280Readings, sizeof(BME280Readings));

//   if (result == ESP_OK)
//   {
//     Serial.println("Sent with success");
//   }
//   else
//   {
//     Serial.println("Error sending the data");
//   }
//   delay(10000);
// }