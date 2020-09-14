#include <TinyLoRa.h>
#include <Adafruit_GPS.h>
#include <SPI.h>

#define GPSSerial Serial1

// Network Session Key (MSB)
uint8_t NwkSkey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Application Session Key (MSB)
uint8_t AppSkey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Device Address (MSB)
uint8_t DevAddr[4] = { 0x00, 0x00, 0x00, 0x00 };

// How many times data transfer should occur, in seconds
const unsigned int sendInterval = 30;

// Pinout for Adafruit Feather 32u4 LoRa
TinyLoRa lora = TinyLoRa(7, 8, 4);
Adafruit_GPS GPS(&GPSSerial);

unsigned char loraData[10];

void setupLora() {
  Serial.print("Starting LoRa...");
  lora.setChannel(MULTI);
  lora.setDatarate(SF7BW125);
  if(!lora.begin())
  {
    Serial.println("LoRa connection Failed");
    Serial.println("Check your radio");
    while(true) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(2000);
    }
  }

  Serial.println("LoRa Setup Complete");
}

void setupGPS() {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ); 
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_100_MILLIHERTZ); 

  delay(1000);
  Serial.println("GPS Setup Complete");  
}

void setCoords() {
  snprintf((char*)loraData, sizeof(loraData), "%.4f%c%.4f%c", 
   GPS.latitude, GPS.lat, GPS.longitude, GPS.lon);
}

void sendCoords() {
  lora.sendData(loraData, sizeof(loraData), lora.frameCounter);
  lora.frameCounter++;
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
}

void setup()
{
  delay(2000);
  Serial.begin(115200);
  
  // Initialize pin LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);
  
  setupLora();
  setupGPS();
}

void loop()
{
  setCoords();
  Serial.println("Sending LoRa Data..."); Serial.print(loraData);
  sendCoords();
  
  delay(sendInterval * 1000);
}
