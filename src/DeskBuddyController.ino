/**
 * Incubator Controller V2
 *
 * Author: Tony Kambourakis
 * License: Apache License v2
 */

extern "C" {
  #include "user_interface.h"
  #include "gpio.h"
}

//#define DEBUG_ESP_WIFI


#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include "SensitiveConfig.h"
#include <Time.h>
#include <SparkFun_APDS9960.h>


void callback(char* topic, byte* payload, unsigned int length);

// Configure 2 line LCD display with I2C interface
//LiquidCrystal_I2C lcd(0x27,16,2);
LiquidCrystal_I2C lcd(0x27,20,4);

bool presenceStatus = false;

// Timers
os_timer_t environmentTimer;
bool isEnvironmentTimerComplete = false;

os_timer_t trayTiltTimer;
bool isTrayTiltTimerComplete = false;

os_timer_t lcdStatusMessageTimer;
bool isLcdStatusMessageTimerComplete = false;


// JSON buffer size
const int BUFFER_SIZE = JSON_OBJECT_SIZE(10);


// WiFI credentials
const char* ssid = WIFI_SSID;
//const char* password = WIFI_PASSWORD;

// IBM Internet of Things MQTT configuration
char server[] = IOT_ORG IOT_BASE_URL;
char topic[] = "iot-2/evt/status/fmt/json";
char tempTopic[] = "iot-2/evt/temp/fmt/json";
char debugTopic[] = "iot-2/evt/debug/fmt/json";
char motionDetectedTopic[] = "iot-2/evt/motion/fmt/json";
char authMethod[] = "use-token-auth";
char token[] = IOT_TOKEN;
char clientId[] = "d:" IOT_ORG ":" IOT_DEVICE_TYPE ":" IOT_DEVICE_ID;

// Global Variables

WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);

uint16_t volts;

// Temperature and Humidity Sensor (DHT22) Configuration
#define DHTPIN D2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// PIN definitions
#define OWNER_STATUS_IN 0 // D3 = GPIO0
#define OWNER_STATUS_OUT 5 // D1 = GPIO5
#define OWNER_MESSAGE  12 // D6 = GPIO12
#define MOTIONSENSORPIN D7 // D7 = GPIO13

// D0 = GPIO16
// D8 = GPIO15 - Interrupt for Gesture Sensor
// RST = Reset

// Timer Trigger Switches

// State Variables
bool motionSensorDetected;
int motionSensorDetectedCount = 0;

int counter = 0;
float h; // humidity
float t; // temperature
float hic; // heat index


// DF Robot APDS9960 Gesture Sensor

//#define APDS9960_INT D8 //GPIO15 - Interrupt pin
#define APDS9960_INT 10 //GPIO10 - Interrupt pin
#define APDS9960_SDA D4 //GPIO2
#define APDS9960_SCL D5 //GPIO14

int isr_flag = 0;


SparkFun_APDS9960 apds = SparkFun_APDS9960();

//*** Initialisation ********************************************

void init_lcd() {
  // SDA = D4 (GPIO2), SCL = D5 (GPIO14)
  lcd.init(2,14);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
}

void configureOTA() {
  Serial.println("Initialising OTA");
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
    ArduinoOTA.setHostname(IOT_DEVICE_ID);

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

    ArduinoOTA.onStart([]() {
      Serial.println("Starting OTA update");
      publishDebug("Starting OTA update");
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnding OTA update");
      publishDebug("Ending OTA update");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      publishDebug("OTA Error");
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.print("Test: ");
    Serial.println(WiFi.localIP());
}

void init_wifi() {
  lcdStatus("Connecting to WiFi");
  Serial.println("DeskBuddy Controller OTA v1");
  Serial.println("Initialising Wifi");

  // change this back

  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  char wifiName[30];
  sprintf(wifiName,"%s",ssid);
  lcdStatus(wifiName);

  if (strcmp (WiFi.SSID().c_str(), ssid) != 0) {
    //WiFi.begin(ssid, password);
    WiFi.begin(ssid);
  }

  while (WiFi.status() != WL_CONNECTED) {
    //Serial.print(WiFi.status());
    //WiFi.printDiag(Serial);
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected with OTA4, IP address: ");
  Serial.println(WiFi.localIP());
  //publishDebug("WiFi Connected with OTA on IP: "+WiFi.localIP());
  Serial.println("About to configure OTA");
  lcdStatus("Starting OTA service");
  configureOTA();
  Serial.println("Configured OTA");
  Serial.println("Checking WiFi again");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
}


void initEnvironmentTimer() {
  // This timer is for taking temperature and humidity readings
  os_timer_setfn(&environmentTimer, environmentTimerFinished, NULL);
}


void startEnvironmentTimer() {
  os_timer_arm(&environmentTimer,3600000, true); // every hour
}

void stopEnvironmentTimer() {
  os_timer_disarm(&environmentTimer);
}

void initLcdStatusMessageTimer() {
  os_timer_setfn(&lcdStatusMessageTimer,lcdStatusTimerFinished, NULL);
}

void startLcdStatusMessageTimer() {
  os_timer_disarm(&lcdStatusMessageTimer);
  os_timer_arm(&lcdStatusMessageTimer,1000,true);
}

void stopLcdStatusMessageTimer() {
  os_timer_disarm(&lcdStatusMessageTimer);
}

void lcdStatus(String message) {
  lcd.setCursor(0, 3);
  lcd.print(message);
}

void lcdStatusTimerFinished(void *pArg) {
  isLcdStatusMessageTimerComplete = true;
}
// Configure DF Robot APDS9960 Gesture Sensors

void configGestureSensor() {
  Wire.begin(APDS9960_SDA, APDS9960_SCL);
  pinMode(APDS9960_INT, INPUT);
  attachInterrupt(APDS9960_INT, gestureInterrupt, FALLING);
  if (apds.init()) {
    lcdStatus("Init Gesture Sensor");
  } else {
    lcdStatus("Gesture Sensor Fail");
  }

  if (apds.enableGestureSensor(true)) {
    lcdStatus("Gesture Sensor On");
  } else {
    lcdStatus("Gesture Sensor Bad");
  }

}


String macToStr(const uint8_t* mac)
{
  String result;

  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
    result += ':';
    }
  return result;
}

void printMacAddress() {
  String clientMac = "";
  unsigned char mac[6];
  WiFi.macAddress(mac);
  clientMac += macToStr(mac);
  Serial.print("Mac address is...");
  Serial.println(clientMac);
}
// Initialisation

void setup() {

  // Confgure Gesture Sensor
//  configGestureSensor();

  // Configure serial port
  Serial.begin(115200);
  Serial.println();

  pinMode(OWNER_STATUS_IN, OUTPUT);
  pinMode(OWNER_STATUS_OUT, OUTPUT);
  pinMode(OWNER_MESSAGE, OUTPUT);

  pinMode(MOTIONSENSORPIN, INPUT);

  attachInterrupt(MOTIONSENSORPIN, motionSensorActivated, RISING);

  digitalWrite(OWNER_STATUS_IN, 0); // switch off
  digitalWrite(OWNER_STATUS_OUT, 0); // switch off
  digitalWrite(OWNER_MESSAGE, 0);
  init_lcd();
  lcdStatus("Initialising");
  delay(2000);
  printMacAddress();
  init_wifi();
  connectWithBroker();
  lcdStatus("Initialising sensors");
  // Start the DHT22
  publishDebug("WiFi On");
  publishDebug("Initialising Sensors");
  dht.begin();
  Serial.println("sensor is starting..");
  Serial.print("Reading Analog...");
  Serial.println(analogRead(0));

  initEnvironmentTimer();
  startEnvironmentTimer();

  // perform initial climate read

  readDHTSensor();
  debugDisplayPayload();
  displayLCD();
  publishPayload();
  publishDebug("Setup complete");
  lcdStatus("Setup complete");
  initLcdStatusMessageTimer();
}

// *************************************************************

// Gesture Sensor Interrupt Routine

static int motionCounter = 0;

void gestureInterrupt() {
  isr_flag = 1;
}

int motionSensor = 0;

void loop() {
  if (isEnvironmentTimerComplete == true) {
    isEnvironmentTimerComplete = false;

    Serial.println("Environment timer completed");
    publishDebug("Environment timer completed");
    ++counter;
    readDHTSensor();
    //connectWithBroker();
    debugDisplayPayload();
    displayLCD();
    publishPayload();
  }

  if (isLcdStatusMessageTimerComplete == true) {
    isLcdStatusMessageTimerComplete = false;
    stopLcdStatusMessageTimer();
    displayLCD();
  }

  if (motionSensorDetected == true) {
    motionSensorDetected = false;
    motionCounter++;

    Serial.println("Motion Detected");

    publishMotionDetected();
    // lcdStatus("Motion Detected "+String(motionCounter));
    //
    // startLcdStatusMessageTimer();
  }

  if (isr_flag == 1) {
    detachInterrupt(APDS9960_INT);
    handleGesture();
    if (digitalRead(APDS9960_INT) == 0) {
      apds.init();
      apds.enableGestureSensor(true);
    }
    isr_flag = 0;
    attachInterrupt(APDS9960_INT, gestureInterrupt, FALLING);
  }

if (WiFi.status() != WL_CONNECTED) {
  lcdStatus("Reconnecting to WiFi");
  WiFi.begin(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    //Serial.print(WiFi.status());
    //WiFi.printDiag(Serial);
    delay(1000);
    Serial.print(".");
  }
  displayLCD();
}

if (!!!client.connected()) {
  lcdStatus("Reconnecting to Broker");
  connectWithBroker();
  displayLCD();
}

  ArduinoOTA.handle();
  client.loop();
}

// Timer Call Backs

void environmentTimerFinished(void *pArg) {
  // don't do too much in here. Just set the state
  // and handle it in the main loop
  isEnvironmentTimerComplete = true;
}


// ************************************************************************


void handleGesture() {
  if (apds.isGestureAvailable()) {
    switch (apds.readGesture()) {
      case DIR_UP:
        lcdStatus("Gesture UP");
        publishDebug("Gesture UP");
        break;
      case DIR_DOWN:
        lcdStatus("Gesture DOWN");
        publishDebug("Gesture DOWN");
        break;
      case DIR_LEFT:
        lcdStatus("Gesture LEFT");
        publishDebug("Gesture LEFT");
        break;
      case DIR_RIGHT:
        lcdStatus("Gesture RIGHT");
        publishDebug("Gesture RIGHT");
        break;
      case DIR_NEAR:
        lcdStatus("Gesture NEAR");
        publishDebug("Gesture NEAR");
        break;
      case DIR_FAR:
        lcdStatus("Gesture FAR");
        publishDebug("Gesture FAR");
        break;
      default:
        lcdStatus("Gesture unknown");
        publishDebug("Gesture "+String(apds.readGesture()));
    }
    startLcdStatusMessageTimer();
  }


}

void readDHTSensor() {
   // reading DHT22

  h = dht.readHumidity();
  t = dht.readTemperature();

  // Check if we fail to read from the DHT sensor
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor");
    publishDebug("Failed to read from DHT sensor");
    //TODO: Do more here
  }

  hic = dht.computeHeatIndex(t, h, false);
}

void connectWithBroker() {

  Serial.println("Calling HTTP first");
  WiFiClient httpClient;

  const int httpPort = 80;
  const char* host = "deskbuddy.mybluemix.net";
  String url = "/climate";
  if (!httpClient.connect(host, httpPort)) {
    Serial.println("connection failed for HTTP");
  }

  httpClient.print(String("GET ") + url+ " HTTP/1.1\r\n" +
        "Host: " + host + "\r\n" +
        "Connection: close\r\n\r\n");
  unsigned long timeout = millis();
  while (httpClient.available() == 0) {
  if (millis() - timeout > 5000) {
    Serial.println(">>> Client Timeout !");
    httpClient.stop();
    return;
    }
  }

  while(httpClient.available()){
      String line = httpClient.readStringUntil('\r');
      Serial.print(line);
  }




  // now do fake login to IBM Guest WiFI

  // const char* guestUserId = "dbuddy01"
  // const char* guestPassword = "v58ety89"




  if (!!!client.connected()) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Connecting to broker");
    Serial.print("Reconnecting client to ");
    Serial.println(server);
    lcd.setCursor(0,1);
    int cursorPosition = 0;

    char connectionStateString[20];
    int connectionState;

    Serial.println("Delaying for 10 seconds before trying to connect with Broker");
    delay(10000);
    while (!!!client.connect(clientId, authMethod, token)) {

      connectionState = client.state();
      sprintf(connectionStateString,"state=%d",connectionState);
      Serial.println(connectionStateString);
      Serial.print(".");
      lcd.setCursor(cursorPosition++,1);
      lcd.print(".");
      delay(500);
    }

    client.subscribe("iot-2/cmd/indicator/fmt/json");

    Serial.println();
  }
}

void publishPayload() {

  String deviceId(IOT_DEVICE_ID);

  String payload = "{\"d\":{\"myName\":\"ESP8266.Test1\",\"counter\":";


  payload += counter;
  payload += ",\"volts\":";
  payload += volts;
  payload += ",\"temperature\":";
  payload += t;
  payload += ",\"humidity\":";
  payload += h;
  payload += ",\"heatIndex\":";
  payload += hic;
  payload += ",\"deviceId\":";

  //payload += ",\"deviceId\":\""+IOT_DEVICE_ID+"\"";
  payload += "\""+deviceId+"\"";
  payload += "}}";

  Serial.print("Sending payload: ");
  Serial.println(payload);

  if (client.publish(tempTopic, (char*) payload.c_str())) {
    Serial.println("Publish ok");
  } else {
    Serial.print("Publish failed with error:");
    Serial.println(client.state());
  }
}

void publishMotionDetected() {
  Serial.println("Publishing motion detected message");
  String payload = "{\"d\":{\"myName\":\"ESP8266.Test1\",\"message\":";

  String motionDetectedMessage = "MotionDetected";

  payload += "\""+motionDetectedMessage+"\"}}";

  if (client.publish(motionDetectedTopic, (char *) payload.c_str())) {
    Serial.println("Published motion detected OK");
  } else {
    Serial.print("Publish motion detected failed with error:");
    Serial.println(client.state());
  }

}

void publishDebug(String debugMessage) {
  Serial.println("Publishing debug message");
  String payload = "{\"d\":{\"myName\":\"ESP8266.Test1\",\"message\":";

  payload += "\""+debugMessage+"\"}}";

  if (client.publish(debugTopic, (char *) payload.c_str())) {
    Serial.println("Published debug OK");
  } else {
    Serial.print("Publish failed with error:");
    Serial.println(client.state());
  }
}

void displayLCD() {

  static char tempStr[15];
  static char humidStr[15];
  static char hicStr[15];

  char firstLine[20];
  char secondLine[20];

  dtostrf(t,2,1,tempStr);
  dtostrf(h,2,1,humidStr);
  dtostrf(hic,2,1,hicStr);

  sprintf(firstLine,"Temp:%sC Hum:%s%",tempStr,humidStr);
  sprintf(secondLine,"HI:%sC C:%d",hicStr,counter);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(firstLine);
  lcd.setCursor(0,1);
  lcd.print(secondLine);
  updateStatus();
}

void updateStatus() {
  if (presenceStatus == true) {
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("Tony is in today");
    digitalWrite(OWNER_STATUS_IN, 1); // switch off
    digitalWrite(OWNER_STATUS_OUT, 0);
    digitalWrite(OWNER_MESSAGE, 0);
  } else {
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("Tony is away today");

    digitalWrite(OWNER_STATUS_IN, 0); // switch off
    digitalWrite(OWNER_STATUS_OUT, 1);
    digitalWrite(OWNER_MESSAGE, 0);
  }
}

void debugDisplayPayload() {
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.println(" *C ");
}

void processJson(char * message) {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);

  if (root.containsKey("status")) {
    if (strcmp(root["status"], "here") == 0) {
      presenceStatus = true;
      lcd.setCursor(0,2);
      lcd.print("                    ");
      lcd.setCursor(0,2);
      lcd.print("Tony is in today");
      digitalWrite(OWNER_STATUS_IN, 1); // switch off
      digitalWrite(OWNER_STATUS_OUT, 0);
      digitalWrite(OWNER_MESSAGE, 0);

    } else if (strcmp(root["status"],"away") == 0) {
      presenceStatus = false;
      lcd.setCursor(0,2);
      lcd.print("                    ");
      lcd.setCursor(0,2);
      lcd.print("Tony is away today");

      digitalWrite(OWNER_STATUS_IN, 0); // switch off
      digitalWrite(OWNER_STATUS_OUT, 1);
      digitalWrite(OWNER_MESSAGE, 0);
    } else {
      lcd.setCursor(0,2);
      lcd.print("                    ");
      lcd.setCursor(0,2);
      lcd.print("Important Message");

      digitalWrite(OWNER_STATUS_IN, 0); // switch off
      digitalWrite(OWNER_STATUS_OUT, 0);
      digitalWrite(OWNER_MESSAGE, 1);

    }
  }

}
void motionSensorActivated() {
  motionSensorDetected = true;
  motionSensorDetectedCount += 1;
}


void callback(char* topic, byte* payload, unsigned int length) {
 Serial.println("callback invoked");
 char message_buff[length+1];
 strncpy(message_buff,(char *)payload, length);
 message_buff[length] = '\0';
 String topicString = String(topic);
 String payloadString = String(message_buff);

 processJson(message_buff);

 String callBackDetails = "callback received on topic ["+topicString+"] with payload ["+payloadString+"]";


 //sprintf(callBackDetails,"callback received on topic %s with payload",topic);
 publishDebug("topic="+topicString);
 publishDebug("payload="+payloadString);
 //publishDebug(callBackDetails);


 lcd.setCursor(0,3);
 lcd.print(payloadString);
}

void lcdClearLine() {
  lcd.print("                ");
}
