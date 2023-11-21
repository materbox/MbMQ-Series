/************* Includes *************/
#include <FS.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <time.h>
#include <stdio.h>
#include <LittleFS.h>             //https://github.com/esp8266/Arduino/tree/master/libraries/LittleFS
#include <MQUnifiedsensor.h>      //https://github.com/miguel5612/MQSensorsLib/tree/master
/************* End Includes *************/

/************* Define default values *************/
#define SERIAL_DEBUG_BAUD   115200            // Baud rate for debug serial
#define CURRENT_FIRMWARE_TITLE    "MB-MQ-Series"
#define CURRENT_FIRMWARE_VERSION  "0.1.0"
const char* deviceName            = "MB-MQ-Series";
unsigned long mtime               = 0;
const int TIME_TO_SEND_TELEMETRY  = 30; //every x seconds to send tellemetry
/************* End Define default values *************/

/************* Double Reset config *************/
#define ESP_DRD_USE_LITTLEFS    true
#define ESP_DRD_USE_SPIFFS      false
#define ESP_DRD_USE_EEPROM      false
#define ESP8266_DRD_USE_RTC     false      
#define DOUBLERESETDETECTOR_DEBUG       true  //false
#include <ESP_DoubleResetDetector.h>    //https://github.com/khoih-prog/ESP_DoubleResetDetector
#define DRD_TIMEOUT 5 // Number of seconds after reset during which a subseqent reset will be considered a double reset.
#define DRD_ADDRESS 0

DoubleResetDetector* drd;
/************* End Double Reset config *************/

/************* Thingsboard *************/
#define THINGSBOARD_ENABLE_PROGMEM 0  // Disable PROGMEM because the ESP8266WiFi library, does not support flash strings.
#define THINGSBOARD_ENABLE_STREAM_UTILS 1 // Enables sending messages that are bigger than the predefined message size
#define THINGSBOARD_ENABLE_DYNAMIC 1
#include "ThingsBoard.h"          //https://github.com/thingsboard/thingsboard-arduino-sdk

char THINGSBOARD_SERVER[40] = "materbox.io";
char TOKEN[36] = "DHT11_DEMO_TOKEN";

uint16_t THINGSBOARD_PORT = 1883U;
const uint16_t MAX_MESSAGE_SIZE = 128U; // Maximum size packets will ever be sent or received by the underlying MQTT client

WiFiClient espClient;
ThingsBoard tb(espClient, MAX_MESSAGE_SIZE);
/************* End Thingsboard *************/

/************* Wifi Manager *************/
const char* modes[] = { "NULL", "STA", "AP", "STA+AP" };

WiFiManager wm;

bool TEST_CP         = false; // always start the configportal, even if ap found
int  TESP_CP_TIMEOUT = 180; // test cp timeout
bool TEST_NET        = true; // do a network test after connect, (gets ntp time)
bool ALLOWONDEMAND   = false; // enable on demand
int  ONDDEMANDPIN    = 0; // gpio for button
bool WMISBLOCKING    = true; // use blocking or non blocking mode, non global params wont work in non blocking
bool STAND_ALONE     = false; // use device without thingsboard server
bool RESET_SETTINGS  = false; //reset WIFI settings - for testing
bool WM_CONNECTED    = false;
bool DRD_DETECTED    = false;
/************* End Wifi Manager *************/

/************* Sensor MQ-series *************/
bool MQ_DETECTED = false;
#define BOARD "ESP8266"
#define VOLTAGE_RESOLUTION 5
#define MQ_ANALOG_PIN A0 //Analog input 0 of your arduino
#define MQ_TYPE "MQ-135" //MQ135
#define ADC_BIT_RESOLUTION 10 // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  
//#define calibration_button 13 //Pin to calibrate your sensor
#include <MQUnifiedsensor.h>      //https://github.com/miguel5612/MQSensorsLib/tree/master

//Declare Sensor
MQUnifiedsensor MQ135(BOARD, VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, MQ_ANALOG_PIN, MQ_TYPE);

struct mq {
          float co; //
          float co2; //
          float alcohol; //
          float toluen;
          float nh4;
          float aceton;
        } mqData;
/************* End Sensor MQ-series *************/

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  Serial.println("\n Starting");
  Serial.println("Error - TEST");
  Serial.println("Information- - TEST");
  Serial.println("[ERROR]  TEST");
  Serial.println("[INFO] TEST");  
  
  setupMqSensor();

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  if (drd->detectDoubleReset()) {
      Serial.println("[INFO] Double Reset Detected");
      DRD_DETECTED = true;
    } else {
      Serial.println("[INFO] No Double Reset Detected");
      DRD_DETECTED = false;
    }

  setupWifiManager(DRD_DETECTED);
}

void loop() {
  if(millis()-mtime > (TIME_TO_SEND_TELEMETRY * 1000)){
    if(WiFi.status() == WL_CONNECTED){
      getTime();
      if (!tb.connected()) {
        // Reconnect to the ThingsBoard server,
        // if a connection was disrupted or has not yet been established
        Serial.printf("[INFO] Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
        if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
          Serial.println("[ERROR] Failed to connect");
        }
      }
      if (MQ_DETECTED) {  
        struct mq mqData;
        mqData = getMqData();
        sendTelemetry(mqData);
      }
    } else {
      Serial.println("No Wifi");
    }
    mtime = millis();
  }
  tb.loop();
}

void setupMqSensor(){
 //Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init(); 
  /* 
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ135.setRL(10);
  */
  /*****************************  MQ CAlibration ********************************************/ 
  // Explanation: 
  // In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
  // and on clean air (Calibration conditions), setting up R0 value.
  // We recomend executing this routine only on setup in laboratory conditions.
  // This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
      while(1);
  }
  /*****************************  MQ CAlibration ********************************************/ 
  MQ_DETECTED = true;
}

void sendTelemetry(struct mq mqData){
  Serial.print("[SENSOR] CO: ");
  Serial.println(mqData.co);
  Serial.print("[SENSOR] CO2: ");
  Serial.println(mqData.co2);
  Serial.print("[SENSOR] ALCOHOL: ");
  Serial.println(mqData.alcohol);
  Serial.print("[SENSOR] TOLUEN: ");
  Serial.println(mqData.toluen);
  Serial.print("[SENSOR] NH4: ");
  Serial.println(mqData.nh4);
  Serial.print("[SENSOR] ACETON: ");
  Serial.println(mqData.aceton);

  tb.sendTelemetryData("co", mqData.co);
  tb.sendTelemetryData("co2", mqData.co2);
  tb.sendTelemetryData("alcohol", mqData.alcohol);
  tb.sendTelemetryData("toluen", mqData.toluen);
  tb.sendTelemetryData("nh4", mqData.nh4);
  tb.sendTelemetryData("aceton", mqData.aceton);
}

struct mq getMqData(){
  mq localgetMqData;
      MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
      MQ135.setA(605.18); MQ135.setB(-3.937); // Configure the equation to calculate CO concentration value
      localgetMqData.co = MQ135.readSensor();

      MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
      localgetMqData.co2 = MQ135.readSensor();

      MQ135.setA(77.255); MQ135.setB(-3.18); //Configure the equation to calculate Alcohol concentration value
      localgetMqData.alcohol = MQ135.readSensor();

      MQ135.setA(44.947); MQ135.setB(-3.445); // Configure the equation to calculate Toluen concentration value
      localgetMqData.toluen = MQ135.readSensor();

      MQ135.setA(102.2 ); MQ135.setB(-2.473); // Configure the equation to calculate NH4 concentration value
      localgetMqData.nh4 = MQ135.readSensor();

      MQ135.setA(34.668); MQ135.setB(-3.369); // Configure the equation to calculate Aceton concentration value
      localgetMqData.aceton = MQ135.readSensor();
  return localgetMqData;

      /*
        Exponential regression:
      GAS      | a      | b
      CO       | 605.18 | -3.937  
      Alcohol  | 77.255 | -3.18 
      CO2      | 110.47 | -2.862
      Toluen  | 44.947 | -3.445
      NH4      | 102.2  | -2.473
      Aceton  | 34.668 | -3.369
      */
}

/************* Wifi Manager *************/
void setupWifiManager(bool DRD_DETECTED){
  // get device id from macAddress
  char deviceid[32] = "";
  byte macAddressArray[6];
  WiFi.macAddress(macAddressArray);
  getDeviceId(macAddressArray, 6, deviceid);

  wm.setDebugOutput(true);
  wm.debugPlatformInfo();

  //reset settings - for testing
  if (RESET_SETTINGS){
    deleteConfigData();
    wm.resetSettings();
    wm.erase();
  }

  loadConfigData();
  WiFiManagerParameter custom_server("server", "MaterBox server", THINGSBOARD_SERVER, 40);
  //WiFiManagerParameter custom_mqtt_port("port", "port", THINGSBOARD_PORT, 6);
  WiFiManagerParameter custom_api_token("apikey", "Token", TOKEN, 32);
  WiFiManagerParameter device_type("devicetype", "Tipo", deviceName, 40, " readonly");
  WiFiManagerParameter device_id("deviceid", "Device Id", deviceid, 40, " readonly");

  // callbacks
  wm.setAPCallback(configModeCallback);
  wm.setWebServerCallback(bindServerCallback);
  wm.setSaveConfigCallback(saveWifiCallback);
  wm.setSaveParamsCallback(saveParamCallback);
  
  // add all your parameters here
  wm.addParameter(&custom_server);
//  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_api_token);
  wm.addParameter(&device_type);
  wm.addParameter(&device_id);

  // invert theme, dark
  wm.setDarkMode(true);

  std::vector<const char *> menu = {"wifi","sep","exit"};
  wm.setMenu(menu); // custom menu, pass vector

  wm.setCountry("US"); // crashing on esp32 2.0

  // set Hostname
  wm.setHostname(("WM_" + wm.getDefaultAPName()).c_str());
  // wm.setHostname("WM_RANDO_1234");

  // show password publicly in form
  wm.setShowPassword(true);
  
  if(!WMISBLOCKING){
    wm.setConfigPortalBlocking(false);
  }

  //sets timeout until configuration portal gets turned off
  wm.setConfigPortalTimeout(180);
  
  // This is sometimes necessary, it is still unknown when and why this is needed
  // but it may solve some race condition or bug in esp SDK/lib
  // wm.setCleanConnect(true); // disconnect before connect, clean connect
  wm.setBreakAfterConfig(true); // needed to use saveWifiCallback

  if(DRD_DETECTED || TEST_CP){
    delay(1000);
    if(!wm.startConfigPortal("MaterBox IoT", "123456789")){
      Serial.println("[INFO] Failed to connect and hit timeout");
    } else {
      Serial.println("[INFO] Wifi connected:)");
      wifiInfo();
    }
  } else {
    if(!wm.autoConnect("MaterBox IoT", "123456789")){
      Serial.println("[INFO] Failed to connect and hit timeout");
    } else {
      Serial.println("[INFO] Wifi connected:)");
      wifiInfo();
    }
  }

  //read updated parameters
  strcpy(THINGSBOARD_SERVER, custom_server.getValue());
  strcpy(TOKEN, custom_api_token.getValue());
  //strcpy(THINGSBOARD_PORT, custom_mqtt_port.getValue());
  printConfigInfo("WifiManager");
  saveConfigData();
}

void saveWifiCallback(){
  Serial.println("[CALLBACK] save settings Callback fired");
  CONFIG_DATA_MODIFIED = true;
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("[CALLBACK] configModeCallback fired");

}

void saveParamCallback(){
  Serial.println("[CALLBACK] saveParamCallback fired");
}

void bindServerCallback(){
  wm.server->on("/custom",handleRoute); // this is now crashing esp32 for some reason
  // wm.server->on("/info",handleRoute); // you can override wm!
}

void handleRoute(){
  Serial.println("[HTTP] handle route");
  wm.server->send(200, "text/plain", "hello from user code");
}

void wifiInfo(){
  // can contain gargbage on esp32 if wifi is not ready yet
  Serial.println("[WIFI] WIFI INFO DEBUG");
  // WiFi.printDiag(Serial);
  Serial.println("[WIFI] SAVED: " + (String)(wm.getWiFiIsSaved() ? "YES" : "NO"));
  Serial.println("[WIFI] SSID: " + (String)wm.getWiFiSSID());
  Serial.println("[WIFI] PASS: " + (String)wm.getWiFiPass());
  Serial.println("[WIFI] HOSTNAME: " + (String)WiFi.getHostname());
}
/************* End Wifi Manager *************/

void saveConfigData() {
      DynamicJsonDocument json(1024);
      json["THINGSBOARD_SERVER"] = THINGSBOARD_SERVER;
      //json["mqtt_port"] = mqtt_port;
      json["TOKEN"] = TOKEN;
      json["STAND_ALONE"] = STAND_ALONE;
      printConfigInfo("Saving data");
      File configFile = LittleFS.open("/config.json", "w");
      if (!configFile) {
        Serial.println("failed to open config file for writing");
      }
      serializeJson(json, Serial);
      serializeJson(json, configFile);
      configFile.close();
      Serial.println();
}

void loadConfigData() {
    //https://www.hackster.io/Neutrino-1/littlefs-read-write-delete-using-esp8266-and-arduino-ide-867180
    //read configuration from FS config.json
  if(!LittleFS.begin()){
    Serial.println("An Error has occurred while mounting LittleFS");
    //Print the error on display
    Serial.println("Mounting Error");
    delay(1000);
    return;
  } else {
    Serial.println("mounted file system");
    if (LittleFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = LittleFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);

        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get());
        serializeJson(json, Serial);
        if (!deserializeError) {
          Serial.println("\nparsed json");
          strcpy(THINGSBOARD_SERVER, json["THINGSBOARD_SERVER"]);
          //strcpy(THINGSBOARD_PORT, json["THINGSBOARD_PORT"]);
          strcpy(TOKEN, json["TOKEN"]);
          STAND_ALONE = json["STAND_ALONE"];
          printConfigInfo("Loaded data");
        } else {
          Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }
  }
}

void printConfigInfo(String FROM){
  Serial.println("[INFO] Print data fired from: " + FROM);
  Serial.println("\tthingsboard server: " + String(THINGSBOARD_SERVER));
  Serial.println("\ttoken: " + String(TOKEN));
  Serial.print("\tstand_alone: ");
  Serial.println((STAND_ALONE) ? "true" : "false");
}

void deleteConfigData(){
   //Remove the file
   LittleFS.remove("/config.json");
}

void getDeviceId(byte macAddressArray[], unsigned int len, char buffer[]){
    for (unsigned int i = 0; i < len; i++){
        byte nib1 = (macAddressArray[i] >> 4) & 0x0F;
        byte nib2 = (macAddressArray[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}

void getTime() {
  int tz           = -6;
  int dst          = 0;
  time_t now       = time(nullptr);
  unsigned timeout = 5000; // try for timeout
  unsigned start   = millis();
  configTime(tz * 3600, dst * 3600, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP time sync: ");
  while (now < 8 * 3600 * 2 ) { // what is this ?
    delay(100);
    Serial.print(".");
    now = time(nullptr);
    if((millis() - start) > timeout){
      Serial.println("\n[ERROR] Failed to get NTP time.");
      return;
    }
  }
  Serial.println("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
}