#include <SPI.h>
#include <MFRC522.h>
#include <esp_now.h>
#include <WiFi.h>

#define RST_PIN 21
#define SS_1_PIN 5

// Wifi
const char *ssid = "iot";
const char *password = "iotisis;";

// set pin numbers
const int redLedPin = 26;   // RED LED
const int greenLedPin = 25; // GREEN LED

MFRC522 mfrc522;

uint8_t broadcastAddress[] = {0x24, 0xdc, 0xc3, 0x14, 0x3d, 0x94};

// SENDING data
typedef struct nodeToSink
{
  const int nodeId = 1;
  const boolean emergencyRequested = false;
  const boolean doorOpen = false;
  const boolean closeRequested = false;
  String rfidMessage = "";
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
  memcpy(&sinkToNodeData, incomingData, sizeof(sinkToNode));
  Serial.print("Received data from: ");
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);

  Serial.print("Door Status: ");
  if (sinkToNodeData.openTheDoor)
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
}

void loop()
{
  // Looking for new cards
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial())
  {
    nodeToSinkData.rfidMessage = dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
    Serial.println(nodeToSinkData.rfidMessage);
    // Send RFID data via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&nodeToSinkData, sizeof(nodeToSink));

    if (result == ESP_OK)
    {
      Serial.println("Sent with success");
    }
    else
    {
      Serial.println("Error sending the data");
    }

    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
  }
}