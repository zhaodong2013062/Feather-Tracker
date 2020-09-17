#include <TinyLoRa.h>
#include <Adafruit_GPS.h>
#include <SPI.h>

#define DEBUG
#define GPSSerial Serial1

#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.println (x)
 #define DEBUG_LED(x) digitalWrite(LED_BUILTIN, x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_LED(x)
#endif

// Network Session Key (MSB)
uint8_t NwkSkey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Application Session Key (MSB)
uint8_t AppSkey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Device Address (MSB)
uint8_t DevAddr[4] = { 0x00, 0x00, 0x00, 0x00 };

// How many times data transfer should occur, in seconds
const unsigned int sendInterval = 30;

TinyLoRa lora = TinyLoRa(7, 8, 4);
Adafruit_GPS GPS(&GPSSerial);

unsigned char loraData[15];
float latitude;
float longitude;

void setupLora() {
  DEBUG_PRINT("Starting LoRa...");
  lora.setChannel(MULTI);
  lora.setDatarate(SF7BW125);
  if(!lora.begin())
  {
    DEBUG_PRINT("LoRa connection Failed");
    DEBUG_PRINT("Check your radio");
    while(true) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(2000);
    }
  }

  DEBUG_PRINT("LoRa Setup Complete");
}

void setupGPS() {
  DEBUG_PRINT("Setting up GPS...");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ); 
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_100_MILLIHERTZ); 

  delay(1000);
  DEBUG_PRINT("GPS Setup Complete");  
}

void setCoords() {
  if (GPS.fix) {
    latitude = GPS.latitude;
    longitude = GPS.longitude;
  } else {
    latitude = 0.0;
    longitude = 0.0;
  }
  
  DEBUG_PRINT(printf("Latitude: %.4f, Longitude: %.4f", latitude, longitude));
  snprintf((char*)loraData, sizeof(loraData), "%.4f%c%.4f%c", 
   GPS.latitude, GPS.lat, GPS.longitude, GPS.lon);
}

void sendCoords() {
  DEBUG_PRINT("Sending LoRa Data..."); 
  DEBUG_PRINT((char*)loraData);
  lora.sendData(loraData, sizeof(loraData), lora.frameCounter);
  DEBUG_PRINT("Data sent successfully");
  lora.frameCounter++;
  DEBUG_LED(HIGH);
  delay(250);
  DEBUG_LED(LOW);
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
  sendCoords();
  
  delay(sendInterval * 1000);
}
