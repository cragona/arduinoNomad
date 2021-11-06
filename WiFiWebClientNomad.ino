/*
  Nomad - Uses Adafruit m0 feather board and wings for wifi and gps modules. DHT sensor with analogue.
          Sps30 sensor uses the i2c protocol. 
 */


#include <SPI.h>
#include <WiFi101.h>
#include "arduino_secrets.h" 
#include <Adafruit_GPS.h>
#include "timestamp32bits.h"
#include <sps30.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define GPSSerial Serial1

// OLED FeatherWing buttons map to different pins depending on board:
#if defined(ESP8266)
  #define BUTTON_A  0
  #define BUTTON_B 16
  #define BUTTON_C  2
#elif defined(ESP32)
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
#elif defined(ARDUINO_STM32_FEATHER)
  #define BUTTON_A PA15
  #define BUTTON_B PC7
  #define BUTTON_C PC5
#elif defined(TEENSYDUINO)
  #define BUTTON_A  4
  #define BUTTON_B  3
  #define BUTTON_C  8
#elif defined(ARDUINO_FEATHER52832)
  #define BUTTON_A 31
  #define BUTTON_B 30
  #define BUTTON_C 27
#else // 32u4, M0, M4, nrf52840 and 328p
  #define BUTTON_A  9
  #define BUTTON_B  6
  #define BUTTON_C  5
#endif

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;
// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
char server[] = "us-central1-little-deuce-coupe.cloudfunctions.net";    // name address for Google (using DNS)
// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
WiFiClient client;
Adafruit_GPS GPS(&GPSSerial);
uint32_t timer = millis();
uint32_t delay_timer = 10000;
timestamp32bits stamp = timestamp32bits();
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

float g_lat = 0;
float g_long = 0;
unsigned long g_epoch_time = stamp.timestamp(21,11,5,5,0,0); //timestamp paramaters order is (year, month, day, hour, minute, second)
float g_pm_sensor_reading = 0.0;
float g_temp = 0.0;
int g_hum = 0;

#define DATA_PIN A5
DHT dht(DATA_PIN, DHT22);

void setup() {
  //Initialize serial and wait for port to open:
  WiFi.setPins(8,7,4,2);
  Serial.begin(115200);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(BUTTON_A, INPUT_PULLUP);
  dht.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  display.display();
  display.startscrollleft(0x00, 0x0F);
  while (digitalRead(BUTTON_A)); //stay till A is pressed
  display.stopscroll();
  // Clear the buffer.
  display.clearDisplay();
  display.display();

  // set display text
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    display.print("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  display.println("Wifi: Connecting");
  display.display(); // actually display all of the above
  while (status != WL_CONNECTED) {
    Serial.print("Connecting to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(delay_timer);  
  }
    // Clear the buffer.
  display.clearDisplay();
  display.setCursor(0,0);
  Serial.println("Connected to wifi!");
  display.println("Wifi: Connected"); drawWifiBars();
  display.display(); // actually display all of the above
  printWiFiStatus();

  gpsSetup();
  spsSetup();
}

void gpsSetup()
{
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  delay(1000);
}

void spsSetup()
{
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Wifi: Connected"); drawWifiBars();
  display.println("SPS sensor setup...");
  display.display();
  Serial.println("SPS sensor setup...\n");
  int16_t ret;
  uint8_t auto_clean_days = 4;
  uint32_t auto_clean;

  delay(2000);

  sensirion_i2c_init();

  while (sps30_probe() != 0) {
    Serial.print("SPS sensor probing failed\n");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Wifi: Connected");
    display.println("SPS sensor probing failed");
    drawWifiBars();
    display.display();
    delay(500);
  }

  ret = sps30_set_fan_auto_cleaning_interval_days(auto_clean_days);

  if (ret) {
    Serial.print("error setting the auto-clean interval: ");
    Serial.println(ret);
  }

  ret = sps30_start_measurement();
  if (ret < 0) {
    Serial.print("error starting measurement\n");
  }
  else
  {
    Serial.println("...SPS sensor started\n");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Wifi: Connected"); drawWifiBars();
    display.println("...SPS sensor started");
    display.display();
  }

  delay(1000);
}

bool printGpsWaitingMsg = false;
int dotWaitingCounter = 0;

void gpsLoop()
{
  GPS.read();

  if (GPS.newNMEAreceived()) 
  {
    if (!printGpsWaitingMsg)
    {
      Serial.print("Gps Searching");
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Gps Searching"); drawWifiBars();
      display.display();
      printGpsWaitingMsg = true;
    }

    GPS.lastNMEA(); // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    {
      return; // we can fail to parse a sentence in which case we should just wait for another
    }

    if (millis() - timer > 2000) 
    {
      if (GPS.fix) 
      {
        Serial.print("\nLatitude: "); Serial.print(GPS.latitudeDegrees); 
        Serial.print(" Longitude: "); Serial.println(GPS.longitudeDegrees); 
        g_lat = GPS.latitudeDegrees;
        g_long = GPS.longitudeDegrees;
        
        g_epoch_time = stamp.timestamp(GPS.year,GPS.month,GPS.day,GPS.hour, GPS.minute, GPS.seconds); //CONVERT TO EPOCHTIME FOR DB
        spsLoop();
        dhtLoop();
        
        //update screen after sensor readings
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0,0); drawWifiBars();
        display.print("Lat: "); display.println(GPS.latitudeDegrees); //line 2
        display.print("Long: "); display.println(GPS.longitudeDegrees); 
        display.print("Temp: "); display.print(g_temp); display.print("C"); 
        display.print(" RH: "); display.print(g_hum); display.println("%"); 
        display.print("PM: "); display.print(g_pm_sensor_reading); display.print("ug/m3");
        display.display();
        postLoop();
        printGpsWaitingMsg = false;
      }
      else
      {
        Serial.print("."); 
        dotWaitingCounter++;
        if (dotWaitingCounter > 50)
        {
          Serial.print("\n"); 
          dotWaitingCounter = 0;
        }
      }
    }
  }
}

void dhtLoop()
{
  g_temp = dht.readTemperature();
  g_hum = dht.readHumidity();

  if (isnan(g_temp)) g_temp = 0;
  if (isnan(g_hum)) g_hum = 0;

  Serial.print("Temp: "); Serial.print(g_temp); Serial.println("C");Serial.print(" Humidity: "); Serial.print(g_hum); Serial.println("%");
}

// do not modify this. This is the protocal set up to pass the data to firestore
String deviceIdTitle = "deviceId=nomad-9999-00000005";
String humidtyTitle = "&humidity=";
String latitudeTitle = "&latitude=";
String longitudeTitle = "&longitude=";
String pmTitle = "&pm2x5Mass=";
String tempTitle = "&temperature=";
String timestampTitle = "&timestamp=";

void postLoop()
{
  String postData = deviceIdTitle+humidtyTitle+String(g_hum)+latitudeTitle+String(g_lat)+longitudeTitle+String(g_long)+pmTitle+String(g_pm_sensor_reading)+tempTitle+String(g_temp)+timestampTitle+String(g_epoch_time);
  Serial.println("\nStarting connection to server...");
  Serial.print("query: "); Serial.println(postData);
  // if you get a connection, report back via serial:
  if (client.connectSSL(server, 443)) {
    Serial.println("connected to server");
    //Make a HTTP request:
    client.println("POST /telemetry HTTP/1.1"); 
    client.print("Host: "); client.println(server);
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println("Connection: close");
    client.print("Content-Length: "); client.println(postData.length());
    client.println();
    client.println(postData);
  }

  // // if there are incoming bytes available, this is for get
  // // from the server, read them and print them:
  // while (client.available()) {
  //   char c = client.read();
  //   Serial.write(c);
  // }

  // if the server's disconnected, stop the client:
  if (!client.connected()) 
  {
    Serial.println();
    client.stop();
  }

  delay(1000);
}

void loop() 
{
  gpsLoop();
  drawWifiBars();
}

void drawWifiBars()
{
  long rssi = WiFi.RSSI();
  if (rssi >= -55) 
  { 
    display.fillRect(102,7,4,1, SSD1306_WHITE);
    display.fillRect(107,6,4,2, SSD1306_WHITE);
    display.fillRect(112,4,4,4, SSD1306_WHITE);
    display.fillRect(117,2,4,6, SSD1306_WHITE);
    display.fillRect(122,0,4,8, SSD1306_WHITE);
  } 
  else if (rssi < -55 & rssi > -65) 
  {
    display.fillRect(102,7,4,1, SSD1306_WHITE);
    display.fillRect(107,6,4,2, SSD1306_WHITE);
    display.fillRect(112,4,4,4, SSD1306_WHITE);
    display.fillRect(117,2,4,6, SSD1306_WHITE);
    display.drawRect(122,0,4,8, SSD1306_WHITE);
  } 
  else if (rssi < -65 & rssi > -75) 
  {
    display.fillRect(102,8,4,1, SSD1306_WHITE);
    display.fillRect(107,6,4,2, SSD1306_WHITE);
    display.fillRect(112,4,4,4, SSD1306_WHITE);
    display.drawRect(117,2,2,6, SSD1306_WHITE);
    display.drawRect(122,0,4,8, SSD1306_WHITE);
  }
  else if (rssi < -75 & rssi > -85) 
  {
    display.fillRect(102,8,4,1, SSD1306_WHITE);
    display.fillRect(107,6,4,2, SSD1306_WHITE);
    display.drawRect(112,4,4,4, SSD1306_WHITE);
    display.drawRect(117,2,4,6, SSD1306_WHITE);
    display.drawRect(122,0,4,8, SSD1306_WHITE);
  } 
  else if (rssi < -85 & rssi > -96) 
  {
    display.fillRect(102,8,4,1, SSD1306_WHITE);
    display.drawRect(107,6,4,2, SSD1306_WHITE);
    display.drawRect(112,4,4,4, SSD1306_WHITE);
    display.drawRect(117,2,4,6, SSD1306_WHITE);
    display.drawRect(122,0,4,8, SSD1306_WHITE);
  } 
  else 
  {
    display.drawRect(102,8,4,1, SSD1306_WHITE);
    display.drawRect(107,6,4,2, SSD1306_WHITE);
    display.drawRect(112,4,4,4, SSD1306_WHITE);
    display.drawRect(117,2,4,6, SSD1306_WHITE);
    display.drawRect(122,0,4,8, SSD1306_WHITE);  
  }
}

void printWiFiStatus() 
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void spsLoop() 
{
  struct sps30_measurement m;
  char serial[SPS30_MAX_SERIAL_LEN];
  uint16_t data_ready;
  int16_t ret;

  do 
  {
    ret = sps30_read_data_ready(&data_ready);
    if (ret < 0) {
      Serial.print("error reading data-ready flag: ");
      Serial.println(ret);
    } else if (!data_ready)
      Serial.print("data not ready, no new measurement available\n");
    else
      break;
    delay(100); /* retry in 100ms */
  } while (1);

  ret = sps30_read_measurement(&m);

  if (ret < 0) 
  {
    Serial.print("error reading measurement\n");
  } 
  else 
  {
    Serial.print("PM  2.5: ");
    Serial.println(m.mc_2p5);
    g_pm_sensor_reading = m.mc_2p5;
  }
}



