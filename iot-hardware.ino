#include <ESP8266WiFi.h>

#include<dht.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>

//Variables for Wi-Fi connection
#ifndef STASSID
  #define STASSID "Malibongwe"
  #define STAPSK  "Hephzibah"
#endif

WiFiClient mClient;

const char* ssid     = STASSID;
const char* password = STAPSK;

const char* server = "remotelivestockmonitor.com";

SoftwareSerial gpsSerial(D3, D4); //(RxPin, TxPin)

//Pin for DHT11 (Humidity cum temperature sensor)
uint8_t DHT_pin = D2;

//Pin for pulse rate sensor
uint8_t BPM_pin = A0;

TinyGPSPlus retGPS;

dht retDHT;

float latitude, longitude;

uint8_t retBPM;

uint8_t intervalCounter = 0;

#define PN532_SCK  (D5)
#define PN532_MOSI (D6)
#define PN532_SS   (D7)
#define PN532_MISO (D8)

Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

dht readHumidityTemperature(){
  dht DHT;
  int chk = DHT.read11(DHT_pin);
  return DHT;
}

int readBMP(){
  unsigned int BPM = analogRead(BPM_pin);
  BPM = (BPM / 10) * 1.2;
  return BPM;
}

TinyGPSPlus readGPS(){
  TinyGPSPlus GPS;
  while(gpsSerial.available() > 0){
    if (GPS.encode(gpsSerial.read())){
      return GPS;
    }
  }
  return GPS;
}

float getLatitude(){
  return 5.4801;
}

float getLongitude(){
  return 7.5437;
}

String readNFC(){
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
  uint8_t uidLength; 
  
  Serial.println(F("Before reading..."));
    
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  Serial.println(F("After reading..."));
  
  if(success){

    Serial.println(F("Successful reading..."));

    //Prepare the UID as String
    uint32_t cardId = uid[0];
    cardId <<= 8;
    cardId |= uid[1];
    cardId <<= 8;
    cardId |= uid[2];  
    cardId <<= 8;
    cardId |= uid[3];

    String s(cardId);

    return s;
  }

  return "";
}

void sendLivestockData(String NFC_UUID, String temperature, String humidity, String pulseRate, String locLatitude, String locLongitude){

  String endPoint = "/api/livestock_data/" + NFC_UUID + "/" + temperature + "/" + humidity + "/" + pulseRate + "/" + locLatitude + "/" + locLongitude;
  
  if (mClient.connect(server, 80))
  { 
    Serial.println(F("Connected to remote server on port 80"));

    Serial.println(F("Attempting to send captured data to remote server..."));

    const String request =  "GET " + endPoint + " HTTP/1.1\r\n" +
                            "Host: remotelivestockmonitor.com\r\n" +
                            "Connection: close\r\n" +
                            "\r\n";
                            
    mClient.print(request);
    mClient.flush();

    delay(5000);

    Serial.println("Response: ");
    
    while(mClient.available()){
      char c = mClient.read();
      Serial.print(c);
    }
    Serial.println("");
  }else{
    Serial.println(F("Could not connect to the server. Check your internet connection..."));
  }
  mClient.stop();
}

void setup() {
  Serial.begin(9600);

  //Setting up the Wi-Fi module
  Serial.println();
  Serial.print(F("Trying to connect to ")); Serial.println(String(ssid));

//  WiFi.mode(WIFI_OFF);
//  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }

  Serial.println();

  delay(1000);
  
  Serial.print(F("WiFi connected - IP address: ")); Serial.println(WiFi.localIP());

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println(F("Didn't find PN53x board"));
//    while (1);
  }
 
  Serial.print(F("Firmware version: ")); Serial.println(versiondata); 

  nfc.SAMConfig();

  //Setting up the GPS module
  gpsSerial.begin(9600);
  
}

void loop() {
  //Counter to determine when data will be sent to server
  intervalCounter++;

//  String UUID = "123456";
  String UUID = readNFC();
  dht retDHT = readHumidityTemperature();
  retBPM = readBMP();
  retGPS = readGPS();

  if(retGPS.location.isValid()){
    latitude = retGPS.location.lat();
    longitude = retGPS.location.lng();
  }else{
    latitude = getLatitude();
    longitude = getLongitude();
  }

  Serial.println("Cattle UUID: " + String(UUID));
  Serial.println("Humidity: " + String(retDHT.humidity));
  Serial.println("Temperature: " + String(retDHT.temperature));
  Serial.println("BMP: " + String(retBPM));
  Serial.println("Latitude: " + String(latitude));
  Serial.println("Longitude: " + String(longitude));

  Serial.println();
  Serial.println();

  if(intervalCounter%5 == 0){
    sendLivestockData(String(UUID), String(retDHT.temperature), String(retDHT.humidity), String(retBPM), String(latitude), String(longitude));
    Serial.println(F("Sent to remote server..."));
    Serial.println();
  }
  
  delay(5000);
}
