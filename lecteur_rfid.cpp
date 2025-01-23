// TEST RFID
/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-mfrc522-rfid-reader-arduino/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <SPI.h>
#include <MFRC522.h>

// PIN Numbers : RESET + SDA
#define RST_PIN 21
#define SS_1_PIN 5

// List of Tags UIDs that are allowed to open the puzzle
#define MAX_TAGS 3
#define UID_SIZE 4
byte tagarray[MAX_TAGS][UID_SIZE] = {
    {0xB4, 0x94, 0x88, 0x5B},
    {0x04, 0x3F, 0x0C, 0x5E},
    {0x0C, 0xC7, 0xB7, 0x6E},
};

// Inlocking status
// bool access = false;

// Create an MFRC522 instance
MFRC522 mfrc522;

/**
   Helper routine to dump a byte array as hex values to Serial.
*/
void dump_byte_array(byte *buffer, byte bufferSize)
{
  for (byte i = 0; i < bufferSize; i++)
  {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

/**
   Initialize.
*/
void setup()
{

  Serial.begin(115200); // Initialize serial communications with the PC
  while (!Serial)
    ; // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)

  SPI.begin(); // Init SPI bus

  mfrc522.PCD_Init(SS_1_PIN, RST_PIN);
  mfrc522.PCD_DumpVersionToSerial();
}

bool checkAccess(MFRC522 &mfrc522)
{

  for (int x = 0; x < MAX_TAGS; x++)
  {
    bool match = true; // Supposons que le tag correspond jusqu'à preuve du contraire

    // Comparer chaque octet de l'UID
    for (int i = 0; i < UID_SIZE; i++)
    {
      if (i >= mfrc522.uid.size || mfrc522.uid.uidByte[i] != tagarray[x][i])
      {
        match = false; // Pas de correspondance
        break;         // Arrêter la vérification de ce tag
      }
    }

    if (match)
    {
      return true; // Le tag est autorisé
    }
  }
  return false; // Aucun tag correspondant trouvé
}

/*
   Main loop.
*/

void loop()
{

  // Looking for new cards
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial())
  {
    // Show some details of the PICC (that is: the tag/card)
    Serial.print(F(": Card UID:"));
    dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
    Serial.println();

    if (checkAccess(mfrc522))
    {
      Serial.println("Welcome! The door is now open.");
      // Ajoutez ici le code pour ouvrir la porte
    }
    else
    {
      Serial.println("This Tag isn't allowed!");
    }
    // Halt PICC
    mfrc522.PICC_HaltA();
    // Stop encryption on PCD
    mfrc522.PCD_StopCrypto1();
  }
}