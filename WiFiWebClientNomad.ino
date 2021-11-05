/*
  Nomad
 */


#include <SPI.h>
#include <WiFi101.h>
#include "arduino_secrets.h" 
#include <ArduinoJson.h>
#include <Adafruit_GPS.h>
#include "timestamp32bits.h"
#include <sps30.h>

#define GPSSerial Serial1

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;
// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(216,239,36,54);  // numeric IP for Google (no DNS) 216.239.36.54
char server[] = "us-central1-little-deuce-coupe.cloudfunctions.net";    // name address for Google (using DNS)
// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
WiFiClient client;
Adafruit_GPS GPS(&GPSSerial);
uint32_t timer = millis();
uint32_t delay_timer = 10000;
timestamp32bits stamp = timestamp32bits();

float g_lat = 0;
float g_long = 0;
unsigned long g_epoch_time = stamp.timestamp(21,11,5,5,0,0); //timestamp paramaters order is (year, month, day, hour, minute, second)
float g_pm_sensor_reading = 0.0;

void setup() {
  //Initialize serial and wait for port to open:
  WiFi.setPins(8,7,4,2);
  Serial.begin(115200);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(delay_timer);  
  }
  Serial.println("Connected to wifi");
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
  Serial.println("SPS sensor setup...\n");
  int16_t ret;
  uint8_t auto_clean_days = 4;
  uint32_t auto_clean;

  delay(2000);

  sensirion_i2c_init();

  while (sps30_probe() != 0) {
    Serial.print("SPS sensor probing failed\n");
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
      Serial.print("Gps Read Begin");
      printGpsWaitingMsg = true;
    }
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    GPS.lastNMEA(); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    {
      return; // we can fail to parse a sentence in which case we should just wait for another
    }

    if (millis() - timer > 2000) 
    {
      if (GPS.fix) 
      {
        Serial.print("\nTime: ");
        if (GPS.hour < 10) { Serial.print('0'); }
        Serial.print(GPS.hour, DEC); Serial.print(':');
        if (GPS.minute < 10) { Serial.print('0'); }
        Serial.print(GPS.minute, DEC); Serial.print(':');
        if (GPS.seconds < 10) { Serial.print('0'); }
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        
        Serial.print("Latitude: "); Serial.print(GPS.latitudeDegrees); 
        Serial.print(" Longitude: "); Serial.println(GPS.longitudeDegrees); 
        g_lat = GPS.latitudeDegrees;
        g_long = GPS.longitudeDegrees;
        
        g_epoch_time = stamp.timestamp(GPS.year,GPS.month,GPS.day,GPS.hour, GPS.minute, GPS.seconds); //CONVERT TO EPOCHTIME FOR DB
        spsLoop();
        postLoop();
        printGpsWaitingMsg = false;
      }
      else
      {
        Serial.print("."); 
        dotWaitingCounter++;

        if (dotWaitingCounter > 50)
        {
          Serial.println(); 
          dotWaitingCounter = 0;
        }
      }
    }
  }
}

String deviceIdTitle = "deviceId=nomad-9999-00000005";
String humidtyTitle = "&humidity=1";
String latitudeTitle = "&latitude=";
String longitudeTitle = "&longitude=";
String pmTitle = "&pm2x5Mass=";
String tempTitle = "&temperature=1";
String timestampTitle = "&timestamp=";

void postLoop()
{
  String postData = deviceIdTitle+humidtyTitle+latitudeTitle+String(g_lat)+longitudeTitle+String(g_long)+pmTitle+String(g_pm_sensor_reading)+tempTitle+timestampTitle+String(g_epoch_time);
  Serial.println("\nStarting connection to server...");
  Serial.print("query: "); Serial.println(postData);
  // if you get a connection, report back via serial:
  if (client.connectSSL(server, 443)) {
    Serial.println("connected to server");
    //Make a HTTP request:
    client.println("POST /telemetry HTTP/1.1"); ///tinyFittings/index.php HTTP/1.1
    client.print("Host: "); client.println(server);
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println("Connection: close");
    client.print("Content-Length: "); client.println(postData.length());
    client.println();
    client.println(postData);
  }

  // // if there are incoming bytes available
  // // from the server, read them and print them:
  // while (client.available()) {
  //   char c = client.read();
  //   Serial.write(c);
  // }

  // if the server's disconnected, stop the client:
  if (!client.connected()) {
    Serial.println();
    //Serial.println("disconnecting from server.");
    client.stop();

    // // do nothing forevermore:
    // while (true);
  }

  delay(1000);
}

void loop() {
  gpsLoop();
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






