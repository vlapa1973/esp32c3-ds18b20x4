#include <Arduino.h>
#include <WiFi.h>
#include <OneWire.h>
#include <PubSubClient.h>
#include <DallasTemperature.h>

const uint8_t wire_bus = 1;
OneWire oneWire(wire_bus);

DallasTemperature DS18B20(&oneWire);

DeviceAddress s0, s1, s2, s3;

#include "setenv.h"

const uint8_t pinBMEminus = 3; //  питание датчика -
const uint8_t pinBMEplus = 19; //  питание датчика +
const uint8_t pinVcc = 4;      //  напряжение батареи

WiFiClient espClient;
PubSubClient client(espClient);

//------------------------------------------------
const char *ssid = WiFi_SSID;
const char *pass = WiFi_PASS;

const char *mqtt_client_name = MQTT_CLIENT_NAME;
const char *mqtt_client_topic = MQTT_CLIENT_TOPIC;
const char *mqtt_user = MQTT_USER;
const char *mqtt_pass = MQTT_PASS;
const char *mqtt_server = MQTT_SERVER;
const char *mqtt_port = MQTT_PORT;

const char *outTopicTemp = "/Temp";

//------------------------------------------------
uint8_t countConnect = 20;        //  кол-во попыток соединения
uint16_t countPause = 500;        //  пауза между попытками
uint32_t timeSleep = 30000000;    //  время сна
uint16_t TimeBeforeBedtime = 500; //  время до засыпания
uint8_t countMaxSleep = 120;      //  не передавать данные не более
                                  //      countMaxSleep х timeSleep (60мин)

RTC_DATA_ATTR struct
{
  float t1 = 0;           //  температура 1
  float t2 = 0;           //  температура 2
  float t3 = 0;           //  температура 3
  float t4 = 0;           //  температура 4
  uint8_t countSleep = 0; //  счетчик предельного кол-ва циклов сна
} data;

RTC_DATA_ATTR bool flagNotWork = false;

//-----------------------------------
inline bool mqtt_subscribe(PubSubClient &client, const String &topic)
{
  Serial.print("Subscribing to: ");
  Serial.println(topic);
  return client.subscribe(topic.c_str());
}

//-----------------------------------
inline bool mqtt_publish(PubSubClient &client, const String &topic, const String &value)
{
  Serial.print(topic);
  Serial.print(" = ");
  Serial.println(value);
  return client.publish(topic.c_str(), value.c_str());
}

//-----------------------------------
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16)
      Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

//-----------------------------------
void mqttDataOut(float temp1, float temp2, float temp3, float temp4)
{
  String topic = "/";
  String a = "";
  a += data.t1;
  a += ",";
  a += data.t2;
  a += ",";
  a += data.t3;
  a += ",";
  a += data.t4;
  topic += mqtt_client_topic;
  topic += outTopicTemp;
  mqtt_publish(client, topic, a);
}

//-----------------------------------
bool reconnect()
{
  client.setServer(mqtt_server, String(mqtt_port).toInt());

  Serial.print("MQTT connect : ");
  Serial.println(mqtt_server);

  while (!(client.connect(mqtt_client_name, mqtt_user, mqtt_pass)) && countConnect--)
  {
    Serial.print(countConnect);
    Serial.print('>');
    delay(countPause);
  }

  if (client.connected())
  {
    Serial.println("MQTT connected - OK !");
    return true;
  }
  else
  {
    return false;
  }
}

//-----------------------------------
bool setupWiFi(const char *wifi_ssid, const char *wifi_pass)
{
  WiFi.begin(wifi_ssid, wifi_pass);

  Serial.print("Setup WiFi: ");
  Serial.println(ssid);

  while ((WiFi.status() != WL_CONNECTED) && countConnect--)
  {
    Serial.print(countConnect);
    Serial.print('>');
    delay(countPause);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    // индикация IP
    Serial.print("\nWiFi connected - OK !\n");
    Serial.println(WiFi.localIP());
    // индикация силы сигнала
    int8_t dBm = WiFi.RSSI();
    Serial.print("RSSI dBm = ");
    Serial.println(dBm);
    uint8_t quality_RSSI = 2 * (dBm + 100);
    if (quality_RSSI >= 100)
      quality_RSSI = 100;
    Serial.print("RSSI % = ");
    Serial.println(quality_RSSI);
    Serial.println("=================");
    return true;
  }
  else
  {
    return false;
  }
}

//-----------------------------------
// медиана на 3 значения со своим буфером
uint16_t medianRoom(uint16_t newValRoom)
{
  RTC_DATA_ATTR static uint16_t bufRoom[3] = {0, 0, 0};
  RTC_DATA_ATTR static byte countRoom = 0;

  Serial.println();
  Serial.print(countRoom);
  Serial.print(" - ");
  Serial.print(bufRoom[0]);
  Serial.print("...");
  Serial.print(bufRoom[1]);
  Serial.print("...");
  Serial.print(bufRoom[2]);
  Serial.println();

  if (!(bufRoom[0] + bufRoom[1] + bufRoom[2]))
    bufRoom[0] = bufRoom[1] = bufRoom[2] = newValRoom;

  bufRoom[countRoom] = newValRoom;
  if (countRoom++ >= 2)
    countRoom = 0;
  uint16_t dataRoom = (max(bufRoom[0], bufRoom[1]) == max(bufRoom[1], bufRoom[2]))
                          ? max(bufRoom[0], bufRoom[2])
                          : max(bufRoom[1], min(bufRoom[0], bufRoom[2]));
  // return expRunningAverage(data);
  return dataRoom;
}

//-----------------------------------
void requestSensors()
{
  oneWire.reset_search();
  DS18B20.begin();
  if (!DS18B20.getAddress(s0, 0))
    Serial.println("Unable to find address for Device s0");
  if (!DS18B20.getAddress(s1, 1))
    Serial.println("Unable to find address for Device s1");
  if (!DS18B20.getAddress(s2, 2))
    Serial.println("Unable to find address for Device s2");
  if (!DS18B20.getAddress(s3, 3))
    Serial.println("Unable to find address for Device s3");
}

//-----------------------------------
float printTemperature(DeviceAddress deviceAddress)
{
  float tempC = DS18B20.getTempC(deviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C)
  {
    Serial.println("Error: Could not read temperature data");
    tempC = 00.00;
    return tempC;
  }

  Serial.print("Device Address: ");
  printAddress(deviceAddress);

  Serial.print("  Temp C: ");
  Serial.println(tempC);
  return tempC;
}

//-----------------------------------
void readData()
{
  Serial.print("\nRequesting temperatures... ");
  uint32_t t = millis();
  DS18B20.requestTemperatures();
  Serial.println(millis() - t);

  data.t1 = printTemperature(s0);
  data.t2 = printTemperature(s1);
  data.t3 = printTemperature(s2);
  data.t4 = printTemperature(s3);
}

//-----------------------------------
void setup()
{
  Serial.begin(115200);
  pinMode(pinBMEplus, OUTPUT);
  digitalWrite(pinBMEplus, HIGH);
  pinMode(pinBMEminus, OUTPUT);
  digitalWrite(pinBMEminus, LOW);
  delay(20);

  requestSensors();
  readData();

  if (!setupWiFi(ssid, pass))
  {
    flagNotWork = true;
    esp_deep_sleep(timeSleep);
  }
  else
  {
    flagNotWork = false;
  }

  if (!reconnect())
  {
    flagNotWork = true;
    esp_deep_sleep(timeSleep);
  }
  else
  {
    flagNotWork = false;
  }

  mqttDataOut(data.t1, data.t2, data.t3, data.t4);

  Serial.println("=================");
  Serial.flush();

  delay(TimeBeforeBedtime);
  esp_deep_sleep(timeSleep);
}

void loop() {}