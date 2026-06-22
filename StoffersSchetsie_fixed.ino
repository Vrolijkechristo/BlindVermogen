#include <Wire.h>
#include <LD2450.h>
#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h>

/* PINOUT
GPIO34  →  PIR sensor 
GPIO4   →  INT rechts TOF
GPIO15  →  LED pin
GPIO16  →  LPn recht TOF
GPIO17  →  INT links TOF
GPIO5   →  LPn links TOF
GPIO21  →  SDA
GPIO1   →  RADAR RX
GPIO3   →  RADAR TX
GPIO22  →  SCL
*/

int pinRadarRX = 1;
int pinRadarTX = 3;
int LPN_PIN_SENSOR1 = 4;
int LPN_PIN_SENSOR2 = 5;  
int pinLED = 15;
int LD2450_RX = 16;
int LD2450_TX = 17;
int pinSDA = 21;
int pinSCL = 22;
int pinPIR = 34;

bool ledState = LOW;
int intervalLED = 1000; // LED knippert elke seconde
int intervalPIR = 1000;
unsigned long previousMillisLED = 0;
unsigned long previousMillisPIR = 0;
unsigned long timetime;

LD2450 ld2450;
#define SENSOR1_ADDRESS 0x29
#define SENSOR2_NEW_ADDRESS 0x30
#define GRID_START_REGISTER 0x0858 // Startregister voor 8x8 grid-data

const char* heatmapChars = " .-:=+*#%@";
int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output
float closeDistance = 1000.0; // Afstand waarbinnen een target als "dichtbij" wordt beschouwd

// Globale variabelen voor TOF en HLK data
uint16_t dataTOF[8][16];
float dataHLK[3];

// Forward declarations
void initTOF();
void initHLK();
void readTOF();
void readHLK();
void calculateClosestPerson(uint16_t dataTOF[8][16], float dataHLK[3]);
void initSensor(uint8_t deviceAddress);
void changeI2CAddress(uint8_t currentAddress, uint8_t newAddress);
void readSensorGrid(uint8_t deviceAddress, uint16_t grid[8][16], int startCol);
void printHeatmap(uint16_t grid[8][16]);
void writeI2CRegister(uint8_t deviceAddress, uint16_t registerAddress, uint8_t value);
uint8_t readI2CRegister(uint8_t deviceAddress, uint16_t registerAddress);
void readI2CBytes(uint8_t deviceAddress, uint16_t startRegister, uint8_t *buffer, uint8_t length);

void setup() {
  Serial.begin(115200);      // Debug monitor
  delay(1000);
  Serial.println("Nou beginnen maar");

  pinMode(pinLED, OUTPUT);

  // initTOF(); // Uitgecommentarieerd zoals in je originele code
  initHLK();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisLED >= intervalLED) {
    previousMillisLED = currentMillis;
    ledState = !ledState;
    if(ledState) { analogWrite(pinLED, 122); }
    else { analogWrite(pinLED, 22); }
  }

  if (currentMillis - previousMillisPIR >= intervalPIR) {
    previousMillisPIR = currentMillis;
    int sensorValue = analogRead(pinPIR);
    if(sensorValue > 575) {
      Serial.println("Mens gesignaleerd");
      readTOF();
      readHLK();
      // calculateClosestPerson(dataTOF, dataHLK); // Deze functie werkt nog niet
    } else {
      Serial.println("Er is niemand");
    }
  }
}

// Initialiseer TOF-sensoren
void initTOF() {
  Wire.begin();
  Wire.setClock(400000); // Zet I2C-clock op 400 kHz

  // Stel LPN-pinnen in als uitgang
  pinMode(LPN_PIN_SENSOR1, OUTPUT);
  pinMode(LPN_PIN_SENSOR2, OUTPUT);

  // Zet beide sensoren uit
  digitalWrite(LPN_PIN_SENSOR1, LOW);
  digitalWrite(LPN_PIN_SENSOR2, LOW);
  delay(10);

  // Initialiseer Sensor 1 (adres 0x29)
  digitalWrite(LPN_PIN_SENSOR1, HIGH);
  delay(50);
  initSensor(SENSOR1_ADDRESS);
  digitalWrite(LPN_PIN_SENSOR1, LOW);

  // Wijzig adres van Sensor 2 naar 0x30
  digitalWrite(LPN_PIN_SENSOR2, HIGH);
  delay(50);
  changeI2CAddress(SENSOR1_ADDRESS, SENSOR2_NEW_ADDRESS);
  initSensor(SENSOR2_NEW_ADDRESS); // Initialiseer Sensor 2 met nieuw adres
  digitalWrite(LPN_PIN_SENSOR2, LOW);

  // Activeer beide sensoren
  digitalWrite(LPN_PIN_SENSOR1, HIGH);
  digitalWrite(LPN_PIN_SENSOR2, HIGH);
  delay(100); // Wacht tot beide sensoren klaar zijn
}

// Initialiseer HLK-radar
void initHLK() {
  Serial2.begin(256000, SERIAL_8N1, LD2450_RX, LD2450_TX);
  ld2450.begin(Serial2);
  delay(1000);
  Serial.println("Sensors klaar!");
}

// Lees TOF-sensoren en vul het grid
void readTOF() {
  // Lees Sensor 1 (adres 0x29, linker 8x8 grid)
  readSensorGrid(SENSOR1_ADDRESS, dataTOF, 0);

  // Lees Sensor 2 (adres 0x30, rechter 8x8 grid)
  readSensorGrid(SENSOR2_NEW_ADDRESS, dataTOF, 8);

  // Print de heatmap
  printHeatmap(dataTOF);
}

// Lees HLK-radar
void readHLK() {
  // Initialiseer dataHLK
  dataHLK[0] = 0;
  dataHLK[1] = 0;
  dataHLK[2] = 0;

  ld2450.read();
  // HLK levert een lijst met targets (x, y, snelheid, etc.)
  for (int i = 0; i < ld2450.getSensorSupportedTargetCount(); i++) {
    const LD2450::RadarTarget result_target = ld2450.getTarget(i);

    if (dataHLK[0] == 0) {
      dataHLK[0] = result_target.distance;
      dataHLK[1] = result_target.x;
      dataHLK[2] = result_target.y;
    } else {
      if (result_target.distance < dataHLK[0]) {
        dataHLK[0] = result_target.distance;
        dataHLK[1] = result_target.x;
        dataHLK[2] = result_target.y;
      }
    }
  }
}

// Bereken de dichtstbijzijnde persoon (nog niet geïmplementeerd)
void calculateClosestPerson(uint16_t dataTOF[8][16], float dataHLK[3]) {
  // Hier kan logica komen om TOF en HLK data te combineren
}

// Initialiseer de sensor in 8x8 modus
void initSensor(uint8_t deviceAddress) {
  // Soft reset
  writeI2CRegister(deviceAddress, 0x002D, 0x01);
  delay(100);

  // Wacht tot de sensor klaar is na reset
  while (readI2CRegister(deviceAddress, 0x0013) != 0x01) {
    delay(10);
  }

  // Stel de sensor in 8x8 modus
  writeI2CRegister(deviceAddress, 0x001E, 0x01); // 8x8 modus
  writeI2CRegister(deviceAddress, 0x0040, 0x01); // Start ranging
  delay(100);
}

// Wijzig het I2C-adres van een sensor
void changeI2CAddress(uint8_t currentAddress, uint8_t newAddress) {
  // Schrijf het nieuwe adres naar register 0x0001
  writeI2CRegister(currentAddress, 0x0001, newAddress << 1); // Shift left voor 7-bit adres
  delay(50); // Wacht tot het adres is gewijzigd
}

// Lees de 8x8 grid van een sensor
void readSensorGrid(uint8_t deviceAddress, uint16_t grid[8][16], int startCol) {
  uint8_t gridData[128];
  readI2CBytes(deviceAddress, GRID_START_REGISTER, gridData, 128);

  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      int index = (y * 8 + x) * 2;
      uint16_t distance = (gridData[index + 1] << 8) | gridData[index];

      // Filter onrealistische waarden (timeout of fout)
      if (distance >= 4000 || distance == 0) {
        distance = 0; // Vervang door 0 voor betere visualisatie
      }
      grid[y][startCol + x] = distance;
    }
  }
}

// Print de heatmap
void printHeatmap(uint16_t grid[8][16]) {
  Serial.println("\n=== 16x8 Grid Heatmap ===");

  // Bepaal de maximale afstand voor normalisatie
  uint16_t maxDistance = 0;
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 16; x++) {
      if (grid[y][x] > maxDistance) {
        maxDistance = grid[y][x];
      }
    }
  }

  // Print de heatmap
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 16; x++) {
      uint16_t distance = grid[y][x];
      if (distance == 0) {
        Serial.print("  "); // Lege ruimte voor onrealistische waarden
      } else {
        int intensity = map(distance, 0, maxDistance, 0, 8);
        intensity = constrain(intensity, 0, 8);
        Serial.print(heatmapChars[intensity]);
      }
      Serial.print(" ");
    }
    Serial.println();
  }

  // Print de numerieke waarden
  Serial.println("\n=== Numerieke Waarden (mm) ===");
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 16; x++) {
      Serial.print(grid[y][x]);
      if (grid[y][x] < 1000) Serial.print("   ");
      else if (grid[y][x] < 10000) Serial.print("  ");
      else Serial.print(" ");
    }
    Serial.println();
  }
}

// Schrijf 1 byte naar een register
void writeI2CRegister(uint8_t deviceAddress, uint16_t registerAddress, uint8_t value) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(highByte(registerAddress));
  Wire.write(lowByte(registerAddress));
  Wire.write(value);
  Wire.endTransmission();
  delay(10); // Kleine vertraging voor betrouwbaarheid
}

// Lees 1 byte uit een register
uint8_t readI2CRegister(uint8_t deviceAddress, uint16_t registerAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(highByte(registerAddress));
  Wire.write(lowByte(registerAddress));
  Wire.endTransmission(false);
  Wire.requestFrom(deviceAddress, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

// Lees meerdere bytes via I2C
void readI2CBytes(uint8_t deviceAddress, uint16_t startRegister, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(highByte(startRegister));
  Wire.write(lowByte(startRegister));
  Wire.endTransmission(false);

  Wire.requestFrom(deviceAddress, length);
  uint8_t i = 0;
  while (Wire.available() && i < length) {
    buffer[i++] = Wire.read();
  }
}
