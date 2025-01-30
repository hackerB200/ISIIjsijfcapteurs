#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Wifi
const char *ssid = "iot";
const char *password = "iotisis;";

// set pin numbers
const int magnetPin = 25;           // Electro Magnet PIN
const int redLedPin = 2;            // RED LED
const int greenLedPin = 5;          // GREEN LED
const int emergencyButtonPin = 26;  // Bouton d'urgence pour ouvrir
const int doorSensorButtonPin = 27; // Bouton qui simule la porte
int emergencyButtonState = 0;
int doorSensorButtonState = 0;

uint8_t broadcastAddress[] = {0x24, 0xdc, 0xc3, 0x14, 0x3d, 0x94};

// SENDING data
typedef struct nodeToSink
{
  const int nodeId = 2;
  boolean emergencyRequested = false;
  boolean doorOpen = false;
  boolean closeRequested = false;
  const String rfidMessage = "";
} struct_nodeToSink;
nodeToSink nodeToSinkData;

// RECEIVING data
typedef struct sinkToNode
{
  boolean openTheDoor = false;
  String staffName = "";
} struct_sinkToNode;
sinkToNode sinkToNodeData;

esp_now_peer_info_t peerInfo;

void sendDataEspNow(const uint8_t *sinkAddress, nodeToSink data)
{
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(sinkAddress, (uint8_t *)&data, sizeof(nodeToSink));
  if (result == ESP_OK)
  {
    Serial.println("Sent with success");
  }
  else
  {
    Serial.println("Error sending the data");
  }
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
  memcpy(&sinkToNodeData, incomingData, sizeof(sinkToNode));
  Serial.print("Received data from: ");
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);

  Serial.print("Door Status: ");
  if (sinkToNodeData.openTheDoor)
  {
    digitalWrite(magnetPin, LOW);
    Serial.println("Open the door");
    nodeToSinkData.doorOpen = true;
    nodeToSinkData.emergencyRequested = false;
    nodeToSinkData.closeRequested = false;
    sendDataEspNow(broadcastAddress, nodeToSinkData);
  }
  else
  {
    if (nodeToSinkData.doorOpen) // not already closed
    {
      digitalWrite(magnetPin, HIGH);
      Serial.println("Close the door");
      nodeToSinkData.doorOpen = false;
      nodeToSinkData.emergencyRequested = false;
      nodeToSinkData.closeRequested = false;
      sendDataEspNow(broadcastAddress, nodeToSinkData);
    }
    else
    {
      Serial.println("The door is already closed");
    }
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(emergencyButtonPin, INPUT);
  pinMode(doorSensorButtonPin, INPUT);
  pinMode(magnetPin, OUTPUT);
  digitalWrite(magnetPin, HIGH); // Always close the door
  nodeToSinkData.doorOpen = false;
  nodeToSinkData.emergencyRequested = false;
  nodeToSinkData.closeRequested = false;

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
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  sendDataEspNow(broadcastAddress, nodeToSinkData); // Send the initial door status
}

void loop()
{
  emergencyButtonState = digitalRead(emergencyButtonPin);
  doorSensorButtonState = digitalRead(doorSensorButtonPin);

  if (nodeToSinkData.doorOpen) {
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(redLedPin, LOW);
  } else {
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
  }

  if (emergencyButtonState == HIGH)
  {
    nodeToSinkData.emergencyRequested = true;
    nodeToSinkData.doorOpen = true;
    nodeToSinkData.closeRequested = false;
    sendDataEspNow(broadcastAddress, nodeToSinkData);

    while (emergencyButtonState == HIGH)
    {
      emergencyButtonState = digitalRead(emergencyButtonPin);
    }
  }

  if (doorSensorButtonState == HIGH)
  {
    if (nodeToSinkData.doorOpen) // not already closed
    {
      nodeToSinkData.emergencyRequested = false;
      nodeToSinkData.closeRequested = true;
      sendDataEspNow(broadcastAddress, nodeToSinkData);
    }

    while (doorSensorButtonState == HIGH)
    {
      doorSensorButtonState = digitalRead(doorSensorButtonPin);
    }
  }
}