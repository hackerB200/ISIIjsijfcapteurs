#include <Arduino.h>
#include <MFRC522.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <iostream>
#include <string>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

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
const int nodes_nb = 2;
uint8_t broadcastAddresses[nodes_nb][6] = {
    {0x24, 0xdc, 0xc3, 0x14, 0x40, 0xac},
    {0xc4, 0xde, 0xe2, 0xae, 0xd6, 0xe4}};
esp_now_peer_info_t peerInfo[nodes_nb];

// SENDING data
typedef struct sinkToNode
{
  boolean openTheDoor = false;
  String staffName = "";
} struct_sinkToNode;
sinkToNode sinkToNodeData;

// RECEIVING data
typedef struct nodeToSink
{
  int nodeId;
  boolean emergencyRequested;
  boolean doorOpen;
  boolean closeRequested;
  String rfidMessage;
} struct_nodeToSink;
nodeToSink nodeToSinkData;

void writeToDisplay(String message)
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(message);
  display.display();
}

void espnowSendDoorStatus()
{
  // Send message via ESP-NOW
  for (int i = 0; i < nodes_nb; i++)
  {
    esp_err_t result = esp_now_send(broadcastAddresses[i], (uint8_t *)&sinkToNodeData, sizeof(sinkToNode));
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

  if (String(topic) == "/security/authorized")
  {
    bool isAuthorized = jsonMessage["isAuthorized"];
    if (isAuthorized == 1)
    {
      Serial.println("Authorized tag detected");
      sinkToNodeData.openTheDoor = true;
      writeToDisplay((jsonMessage["user"].as<String>()) + " authorized");
    }
    else
    {
      Serial.println("Unauthorized tag detected");
      sinkToNodeData.openTheDoor = false;
      writeToDisplay("Unauthorized staff member");
    }
    espnowSendDoorStatus(); // Send the door status to the nodes
  }
  else if (String(topic) == "/emergency/action")
  {
    bool emergencyOpen = jsonMessage["emergencyOpen"];
    if (emergencyOpen == 1)
    {
      Serial.println("Emergency opening requested");
      sinkToNodeData.openTheDoor = true;
    }
    else
    {
      Serial.println("Emergency opening canceled");
      sinkToNodeData.openTheDoor = false;
    }
    espnowSendDoorStatus(); // Send the door status to the nodes
  }
}

void mqttReconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client"))
    {
      Serial.println("connected");
      // Subscribe to topics
      client.subscribe("/security/authorized");
      client.subscribe("/emergency/action");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
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
  memcpy(&nodeToSinkData, incomingData, sizeof(nodeToSink));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Node ID: ");
  Serial.println(nodeToSinkData.nodeId);

  if (nodeToSinkData.nodeId == 1) // RFID node
  {
    Serial.println("RFID message: ");
    Serial.println(nodeToSinkData.rfidMessage);
    client.publish("/security/staff", nodeToSinkData.rfidMessage.c_str());
  }
  else if (nodeToSinkData.nodeId == 2) // Electromagnet node
  {
    if (nodeToSinkData.emergencyRequested) // Emergency opening request
    {
      Serial.println("Emergency request: ");
      Serial.println(nodeToSinkData.emergencyRequested);
      client.publish("/emergency/alert", "Emergency opening requested");
    }
    else if (nodeToSinkData.closeRequested) // Door closing request
    {
      Serial.println("Door closing request: ");
      Serial.println(nodeToSinkData.closeRequested);
      client.publish("/security/close", "Closing requested");
    }
    else // Door status changed
    {
      Serial.println("New door status: ");
      Serial.println(nodeToSinkData.doorOpen);
      client.publish("/security/alert", (nodeToSinkData.doorOpen) ? "Door open" : "Door closed");
    }
  }
}

void setup()
{
  Serial.begin(115200);

  // Wi-Fi
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Setting as a Wi-Fi Station..");
  }
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());

  // ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(espnowOnDataSent);
  for (int i = 0; i < nodes_nb; i++)
  {
    peerInfo[i].channel = 0;
    peerInfo[i].encrypt = false;
    memcpy(peerInfo[i].peer_addr, broadcastAddresses[i], 6);
    if (esp_now_add_peer(&peerInfo[i]) != ESP_OK)
    {
      Serial.println("Failed to add peer");
      return;
    }
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(espnowOnDataRecv));

  // MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);

  // OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  display.clearDisplay();
  display.display();
}

void loop()
{
  if (!client.connected())
  {
    mqttReconnect();
  }
  client.loop();
}