#include <FS.h> //this needs to be first, or it all crashes and burns...

#include "DHT.h"
#include <ESP8266WiFi.h>
#include "PubSubClient.h"
#include <Ticker.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <DNSServer.h> 
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson 5.13.3

char mqtt_server[30];
char mqtt_port[6];
char mqtt_user[20];
char mqtt_pass[20];
char kc_wind[4] = "100";
char windguru_uid[30];
char windguru_pass[20];

String st;
String content;
int statusCode;
volatile int windimpulse = 0;

#define VERSION    "\n\n-----------------  Wind Station v1.1 -----------------"
#define NameAP     "WindStationAP"
#define PasswordAP "87654321"

#define USE_Narodmon
#define USE_Windguru

//#define DeepSleepMODE                                      // To enable Deep-sleep, you need to connect GPIO16 (D0 for NodeMcu) to the EXT_RSTB/REST (RST for NodeMcu) pin
#ifdef DeepSleepMODE
  #define SLEEPTIME     5                                    // deep sleep time in minutes, minimum 5min for use narodmon.com and reasonable power saving
  //#define MOSFETPIN     15                                 // experemental, GPI15 (D8 for NodeMcu). MosFET's gate pin for power supply both sensors, off in sleepmode for current drain minimize. Not connect this GPIO directly to sensors you burning it!!!
#endif  

#define BUTTON          4                                    // optional, GPIO4 for Witty Cloud. GPIO0/D3 for NodeMcu (Flash) - not work with deepsleep, set 4!
#define LED             2                                    // GPIO2 for Witty Cloud. GPIO16/D0 for NodeMcu - not work with deepsleep, set 2!
#define DHTPIN          14                                   // GPIO14 (D5 for NodeMcu)
#define WINDPIN         5                                    // GPIO5 (D1 for NodeMcu)

#define MQTT_TOPIC      "windpoint"                          // mqtt topic (Must be unique for each device)

#define DHTTYPE         DHT11                                //
DHT dht(DHTPIN, DHTTYPE, 11);                                //

extern "C" { 
  #include "user_interface.h" 
}

bool sendStatus = true;
bool sensorReport = false;
bool resetWind = true;
bool firstRun = true;

int requestRestart = 0;
int kUpdFreq = 1;  //minutes
int kRetries = 10;
int kNarodmon = 0;
int meterWind = 0;

/*
int VaneMaxADC = 1023;                                      // ADC range for input voltage 0..1V
int VaneOffset = 0;                                         // define the offset for caclulating wind direction 
int calDirection = 0;                                       // calibrated direction after offset applied
*/

const float kKnots = 1.94;                                  // m/s to knots conversion   
const int windPeriodSec = 3;                                // wind measurement period in seconds 1-10sec

float dhtH = 0, dhtT = 0, windMS = 0;
float WindMax = 0, WindAvr = 0, WindMin = 0, WindNarodmon = 0;

String str;

unsigned long TTasks;
unsigned long secTTasks;
unsigned long count = 0;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Ticker btn_timer;

void callback(const MQTT::Publish& pub) {
  Serial.println("MQTT Payload: " + pub.payload_string());
  if (pub.payload_string() == "stat") {
  }
  else if (pub.payload_string() == "reset") {
    requestRestart = 1;
  }
  else if (pub.payload_string() == "sensor") {
    sensorReport = true;
  }
  else if (pub.payload_string() == "adc") {
     mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/debug","ADC:" + String(analogRead(A0))).set_retain().set_qos(1));  
  }
  else { 
    str = pub.payload_string();
    int i = atoi(str.c_str());
    if ((i > 0) && (i < 999)) { //we got kc_wind?
      strcpy(kc_wind, str.c_str());

      Serial.println("saving config");
      DynamicJsonBuffer jsonBuffer;
      JsonObject& json = jsonBuffer.createObject();
      json["mqtt_server"] = mqtt_server;
      json["mqtt_port"] = mqtt_port;
      json["mqtt_user"] = mqtt_user;
      json["mqtt_pass"] = mqtt_pass;
      json["kc_wind"] = kc_wind;
      json["windguru_uid"] = windguru_uid;
      json["windguru_pass"] = windguru_pass;
    
      File configFile = SPIFFS.open("/config.json", "w");
      if (!configFile) {
        Serial.println("failed to open config file for writing");
      }

      json.printTo(Serial);
      json.printTo(configFile);
      configFile.close();
      
      sendStatus = true;
    }
  }
}

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void windcount() {
  windimpulse++;
  digitalWrite(LED, !digitalRead(LED));
}

void setup() {
  Serial.begin(115200);
  Serial.println(VERSION);
  Serial.print("\nESP ChipID: ");
  Serial.println(ESP.getChipId(), HEX);

  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);
          strcpy(kc_wind, json["kc_wind"]);
          strcpy(windguru_uid, json["windguru_uid"]);
          strcpy(windguru_pass, json["windguru_pass"]);
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read configuration

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 30);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 20);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt password", mqtt_pass, 20);
  WiFiManagerParameter custom_kc_wind("kc_wind", "wind correction 1-999%", kc_wind, 4);
  WiFiManagerParameter custom_windguru_uid("windguru_uid", "windguru station UID", windguru_uid, 30);
  WiFiManagerParameter custom_windguru_pass("windguru_pass", "windguru API pass", windguru_pass, 20);

#ifdef DeepSleepMODE
  //pinMode(MOSFETPIN, OUTPUT);
  //digitalWrite(MOSFETPIN, HIGH);
#endif   

  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(WINDPIN, INPUT_PULLUP);
  digitalWrite(LED, HIGH);
  
  firstRun = true;
  btn_timer.attach(0.05, button);
  
  mqttClient.set_callback(callback);

  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);   //set config save notify callback

#ifdef DeepSleepMODE
  wifiManager.setTimeout(20); //sets timeout until configuration portal gets turned off
#else 
  wifiManager.setTimeout(180); //sets timeout until configuration portal gets turned off
#endif   

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  wifiManager.addParameter(&custom_kc_wind);
  wifiManager.addParameter(&custom_windguru_uid);
  wifiManager.addParameter(&custom_windguru_pass);
  
  if(!wifiManager.autoConnect(NameAP, PasswordAP)) {
    Serial.println("failed to connect and hit timeout");
    delay(1000);
    //reset and try again, or maybe put it to deep sleep
#ifdef DeepSleepMODE
    ESP.deepSleep(SLEEPTIME*60000000);  // Sleep for x* minute(s)
#else    
    ESP.reset();
#endif    
    delay(5000);
  } 

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());
  strcpy(kc_wind, custom_kc_wind.getValue());
  strcpy(windguru_uid, custom_windguru_uid.getValue());
  strcpy(windguru_pass, custom_windguru_pass.getValue());
  
  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_user"] = mqtt_user;
    json["mqtt_pass"] = mqtt_pass;
    json["kc_wind"] = kc_wind;
    json["windguru_uid"] = windguru_uid;
    json["windguru_pass"] = windguru_pass;
    
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save parameters
  }  
  
  Serial.print("\nConnecting to WiFi"); 
  while ((WiFi.status() != WL_CONNECTED) && kRetries --) {
    delay(500);
    Serial.print(" .");
  }
  if (WiFi.status() == WL_CONNECTED) {  
    Serial.println(" DONE");
    Serial.print("IP Address is: "); Serial.println(WiFi.localIP());
    Serial.print("macAddress is: "); Serial.println(WiFi.macAddress());
    Serial.print("Connecting to ");Serial.print(mqtt_server);Serial.print(" Broker . .");
    delay(500);
    mqttClient.set_server(mqtt_server, atoi(mqtt_port));
    while (!mqttClient.connect(MQTT::Connect(String(ESP.getChipId(), HEX)).set_keepalive(90).set_auth(mqtt_user, mqtt_pass)) && kRetries --) {
      Serial.print(" .");
      delay(1000);
    }
    if(mqttClient.connected()) {
      Serial.println(" DONE");
      Serial.println("\n----------------------------  Logs  ----------------------------");
      Serial.println();
      mqttClient.subscribe(MQTT_TOPIC);
      blinkLED(LED, 40, 8);
      digitalWrite(LED, LOW);
    }
    else {
      Serial.println(" FAILED!");
      Serial.println("\n----------------------------------------------------------------");
      Serial.println();
    }
  }
  else {
    Serial.println(" WiFi FAILED!");
    Serial.println("\n----------------------------------------------------------------");
    Serial.println();
  }
  attachInterrupt(WINDPIN, windcount, CHANGE);
}

void loop() { 
  mqttClient.loop();
  timedTasks();
  checkStatus();
  if (sensorReport) {
    getSensors();
  }
}

void blinkLED(int pin, int duration, int n) {             
  for(int i=0; i<n; i++)  {  
    digitalWrite(pin, HIGH);        
    delay(duration);
    digitalWrite(pin, LOW);
    delay(duration);
  }
}

void button() {
  if (!digitalRead(BUTTON)) {
    count++;
  } 
  else {
    if (count > 1 && count <= 40) {   
      digitalWrite(LED, !digitalRead(LED));
      sendStatus = true;
    } 
    else if (count >40){
      Serial.println("\n\nESP8266 Rebooting . . . . . . . . Please Wait"); 
      requestRestart = 1;
    } 
    count=0;
  }
}

void checkConnection() {
  if (WiFi.status() == WL_CONNECTED)  {
    if (mqttClient.connected()) {
      Serial.println("mqtt broker connection . . . . . . . . . . OK");
    } 
    else {
      Serial.println("mqtt broker connection . . . . . . . . . . LOST");
      if (requestRestart < 1) requestRestart = 60;
    }
  }
  else { 
    Serial.println("WiFi connection . . . . . . . . . . LOST");
    if (requestRestart < 1) requestRestart = 5;
  }
}

void checkStatus() {
  if (sendStatus) {
    mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/kc_wind", String(kc_wind)).set_retain().set_qos(1));
    sendStatus = false;
  }
  if (requestRestart == 1) {
    blinkLED(LED, 400, 4);
    ESP.restart();
    delay(500);
  }
}

void getSensors() {
  Serial.print("DHT read . . . . . . . . . . . . . . . . . ");
  char message_buff[60];
  String pubString;
  dhtH = dht.readHumidity();
  dhtT = dht.readTemperature();
  if(digitalRead(LED) == LOW)  {
    blinkLED(LED, 100, 1);
  } else {
    blinkLED(LED, 100, 1);
    digitalWrite(LED, HIGH);
  }
  if (isnan(dhtH) || isnan(dhtT)) {
    if (mqttClient.connected())
    mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/debug","DHT Read Error").set_retain().set_qos(1));
    Serial.println("ERROR");
  } else {
    pubString = "{\"Temp\": "+String(dhtT)+", "+"\"Humidity\": "+String(dhtH) + "}";
    pubString.toCharArray(message_buff, pubString.length()+1);
    mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/temp", message_buff).set_retain().set_qos(1));
    Serial.println("OK");
  }

  if (meterWind > 0) { //already made measurement wind power
    pubString = "{\"Min\": "+String(WindMin, 2)+", "+"\"Avr\": "+String(WindAvr/meterWind, 2)+", "+"\"Max\": "+String(WindMax, 2) + "}";
    pubString.toCharArray(message_buff, pubString.length()+1);
    mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/wind", message_buff).set_retain().set_qos(1));
    Serial.println("Wind Min: " + String(WindMin, 2) + " Avr: " + String(WindAvr/meterWind, 2) + " Max: " + String(WindMax, 2));
  
    WindNarodmon = WindAvr/meterWind;

  }

/*
  // Get Wind Direction
  calDirection = map(analogRead(A0), 0, VaneMaxADC, 0, 359) + VaneOffset;
  if(calDirection > 360)
    calDirection = calDirection - 360;
*/
  resetWind = true;
  sensorReport = false;
}

void timedTasks() {
  //windPeriodSec*1sec timer
  if ((millis() > secTTasks + (windPeriodSec*1000)) || (millis() < secTTasks)) { 
    windMS = (float) windimpulse * windPeriodSec*1000 /(millis() - secTTasks) *atoi(kc_wind)/100;
    //Serial.print("windimpulse: " + String(windimpulse) + " milis: " + String(millis() - secTTasks));
    secTTasks = millis();
    windimpulse = 0;
    
    if (firstRun) {
      firstRun = false;
    } else {
      if (resetWind) {
        WindMax = windMS;
        WindAvr = windMS;
        WindMin = windMS;
        meterWind = 1;
        resetWind = false;
      } else {
        if (((windMS > 10) && (WindMin < 1 )) || ((windMS > 15) && (WindMin < 2 )) || ((windMS > 20) && (WindMin < 3 ))) { //try to filter errors of measurement as result of contact bounce (chatter) 
          mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/debug","cur:"+String(windMS,2) + " min:"+String(WindMin,2) + " avr:"+String(WindAvr/meterWind,2) +" max:"+String(WindMax,2)).set_retain().set_qos(1));  
        } else {
          if (WindMax < windMS) WindMax = windMS;
          if (WindMin > windMS) WindMin = windMS;
          WindAvr = WindAvr + windMS;
          meterWind++;
        }
      }
    }
  }
  //Serial.println(" meterWind: " + String(meterWind) + " windMS: " + String(windMS, 2));
  
#ifdef DeepSleepMODE
  if (meterWind == 5) { //after 5 measurements send data and go to sleep
    getSensors();
    #ifdef USE_Narodmon
      SendToNarodmon();
    #endif
    #ifdef USE_Windguru
      SendToWindguru();
    #endif 
    //digitalWrite(MOSFETPIN, LOW);
    ESP.deepSleep(SLEEPTIME*60000000);  // Sleep for SLEEPTIME*1 minute(s)
    delay(500);
  }
#endif
  
  //kUpdFreq minutes timer
  if ((millis() > TTasks + (kUpdFreq*60000)) || (millis() < TTasks)) { 
    TTasks = millis();
    checkConnection();
    //sensorReport = true;
    getSensors();
    if (requestRestart > 1) requestRestart--;

#ifdef USE_Narodmon
    if (kNarodmon >= 4) {
        SendToNarodmon();
        kNarodmon = 0;
      } else 
        kNarodmon++;
#endif        

#ifdef USE_Windguru
     SendToWindguru();
#endif        
  }
}

bool SendToNarodmon() { //send info to narodmon.com
  WiFiClient client;
  String buf;
  buf = "#" + WiFi.macAddress() + "\r\n"; // header
  if (!isnan(dhtT)) buf = buf + "#T1#" + String(dhtT) + "\r\n";
  if (!isnan(dhtH)) buf = buf + "#H1#" + String(dhtH) + "\r\n";
  buf = buf + "#W1#" + String(WindNarodmon, 2) + "\r\n";
  //buf = buf + "#D1#" + String(calDirection) + "\r\n";
  buf = buf + "##\r\n"; // close packet
 
  if (!client.connect("narodmon.ru", 8283)) // try connect
    {
      Serial.println("connection failed");
      return false; // fail;
    } else {
      client.print(buf); // sending data
      Serial.println(buf);
      while (client.available()) {
        String line = client.readStringUntil('\r'); // redirect answer to serial port
        Serial.println(line);      }
    }
  return true; //done
}

bool SendToWindguru() { // send info to windguru.cz
  WiFiClient client;
  HTTPClient http;    //Declare object of class HTTPClient
  String getData, Link;
  unsigned long time;
  
  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
     Link = "http://www.windguru.cz/upload/api.php?";
     time = millis();
     
     //--------------------------md5------------------------------------------------
     MD5Builder md5;
     md5.begin();
     md5.add(String(time) + String(windguru_uid) + String(windguru_pass));
     md5.calculate();
     //Serial.println(md5.toString()); // can be getChars() to getBytes() too. (16 chars) .. see above 
     //--------------------------md5------------------------------------------------
     
     //wind speed during interval (knots)
     getData = "uid=" + String(windguru_uid) + "&salt=" + String(time) + "&hash=" + md5.toString() + "&wind_min=" + String(WindMin * kKnots, 2) + "&wind_avg=" + String(WindAvr * kKnots/meterWind, 2) + "&wind_max=" + String(WindMax * kKnots, 2);
     //wind_direction     wind direction as degrees (0 = north, 90 east etc...) 
     //getData = getData + "&wind_direction=" + String(calDirection);
   
     if (!isnan(dhtT)) getData = getData + "&temperature=" + String(dhtT);
     if (!isnan(dhtH)) getData = getData + "&rh=" + String(dhtH);
     
     Serial.println(Link + getData);
     http.begin(Link + getData);            //Specify request destination
     int httpCode = http.GET();             //Send the request
     if (httpCode > 0) { //Check the returning code
       String payload = http.getString();   //Get the request response payload
       Serial.println(payload);             //Print the response payload
       if (mqttClient.connected() && (payload != "OK"))
         mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/debug",payload).set_retain().set_qos(1));
     }
     http.end();   //Close connection
   } else  {
      Serial.println("wi-fi connection failed");
      return false; // не удалось;
   }
  
  return true; //ушло
}
