#include <Arduino.h>
#include <MFRC522.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <iostream>
#include <string>
#include <ArduinoJson.h>

// MQTT
const char *mqtt_server = "192.168.3.98";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// Wifi
const char *ssid = "iot";
const char *password = "iotisis;";

// ESP-NOW
const int nodes_nb = 1;
uint8_t broadcastAddresses[nodes_nb][6] = {
    {0x24, 0xdc, 0xc3, 0x14, 0x40, 0xac}};
esp_now_peer_info_t peerInfo[nodes_nb];

// Variable to sent about the door
boolean openTheDoor = false;

// Variable rfid_message to store the message
String rfid_message;

void espnowSendDoorStatus()
{
  // Send message via ESP-NOW
  for (int i = 0; i < nodes_nb; i++)
  {
    esp_err_t result = esp_now_send(broadcastAddresses[i], (uint8_t *)&openTheDoor, sizeof(openTheDoor));
    if (result == ESP_OK)
    {
      Serial.println("Sent with success");
    }
    else
    {
      Serial.println("Error sending the data");
    }
  }
}

// Callback function to receive the message from the MQTT broker
void mqttCallback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");

  StaticJsonDocument<256> jsonMessage;
  DeserializationError error = deserializeJson(jsonMessage, message);
  if (error)
  {
    Serial.print(F("Erreur de parsing JSON: "));
    Serial.println(error.c_str());
    return;
  }
  Serial.println(jsonMessage.as<String>());

  if (String(topic) == "/authorized")
  {
    bool isAuthorized = jsonMessage["isAuthorized"];
    if (isAuthorized == 1)
    {
      Serial.println("Authorized tag detected");
      openTheDoor = true;
    }
    else
    {
      Serial.println("Unauthorized tag detected");
      openTheDoor = false;
    }
    espnowSendDoorStatus(); // Send the door status to the nodes
  }
}

void mqttReconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client"))
    {
      Serial.println("connected");
      // Subscribe to topics
      client.subscribe("/authorized");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Callback when data is sent
void espnowOnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void espnowOnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  Serial.print("Received data from: ");
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&rfid_message, incomingData, sizeof(rfid_message));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("RFID Message: ");
  Serial.println(rfid_message);

  // publier le message sur le topic
  client.publish("/security", rfid_message.c_str());
}

void setup()
{
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
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
  esp_now_register_send_cb(espnowOnDataSent);

  // register peers (motes)
  for (int i = 0; i < nodes_nb; i++)
  {
    // register peer
    peerInfo[i].channel = 0;
    peerInfo[i].encrypt = false;
    memcpy(peerInfo[i].peer_addr, broadcastAddresses[i], 6);
    if (esp_now_add_peer(&peerInfo[i]) != ESP_OK)
    {
      Serial.println("Failed to add peer");
      return;
    }
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(espnowOnDataRecv));

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
}

void loop()
{
  if (!client.connected())
  {
    mqttReconnect();
  }
  client.loop();
}