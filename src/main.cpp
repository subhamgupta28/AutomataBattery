#include <Wire.h>
#include "config.h"
#include <SD.h>
#include <SPI.h>
#include <Adafruit_INA219.h>
#include "Automata.h"
#include "ArduinoJson.h"
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BMP280.h>

#define I2C_SDA_PIN 39
#define I2C_SCL_PIN 41
#define PIN 45
#define FAN 18
#define BTN_PIN 4

// const char* HOST = "192.168.29.67";
// int PORT = 8080;

const char *HOST = "raspberry.local";
int PORT = 8010;

uint8_t i2cAddress = 0x69;
Adafruit_BMP280 bmp;
Preferences preferences;
Automata automata("Battery Main", HOST, PORT);
Adafruit_INA219 ina219_a(0x40);
Adafruit_INA219 ina219_b(0x41);
Adafruit_NeoPixel led(1, 48, NEO_RGB + NEO_KHZ800);

JsonDocument doc;
WiFiUDP udp;
const int udpPort = 12345;
char incomingPacket[255];
float temp = 0;
float pressure = 0;

// Pin definitions for ADC inputs
const int adcPin1 = 4;
const int adcPin2 = 6;
const int adcPin3 = 8;
const int adcPin4 = 10;

// Reference voltage for ESP32 ADC (default is 3.3V)
const float referenceVoltage = 3.3;

// Adjusted Voltage divider ratios for Li-ion 4S (Based on resistor values)
const float dividerRatio1 = 2.0;  // 10kΩ / 10kΩ
const float dividerRatio2 = 11.0; // 100kΩ / 10kΩ
const float dividerRatio3 = 20.6; // 100kΩ / 5.1kΩ
const float dividerRatio4 = 51.0; // 100kΩ / 2kΩ

// Calibration factors (adjust these based on multimeter readings)
float calibrationFactor1 = 1.040;
float calibrationFactor2 = 1.1573;
float calibrationFactor3 = 1.0986;
float calibrationFactor4 = 1.3079;

float actualCell1;
float actualCell2;
float actualCell3;
float actualCell4;

float c1_shunt = 0;
float c2_shunt = 0;
float c1_volt = 0;
float c2_volt = 0;
float c1_curr = 0;
float c2_curr = 0;
float c1_pow = 0;
float c2_pow = 0;

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
float totalEnergy = 0;
float percent = 0;
float capacity_mAh = 0;
long d = 8000;
long st = millis();
unsigned long startMillis;
int ch = millis();
long start = millis();
String uptime;
long timestamp;
float chargingTimeHours = 0;
float dischargingTimeHours = 0;
float targetCapacity = 20;
long startTime = 0;
String startTimeStr = "";
String isDischarge = "DISCHARGE";
time_t now;
bool outPow = true;
bool reset = false;
int pwm = 0;
int fan = 80;
void softOff()
{
  for (int i = pwm; i >= 0; i -= 2)
  {
    analogWrite(PIN, i);
    delay(5);
  }
}

void softOn()
{
  for (int i = 0; i < pwm; i += 2)
  {
    analogWrite(PIN, i);
    delay(5);
  }
}
void action(const Action action)
{
  int pw = action.data["pwm"];
  if (action.data.containsKey("pwm"))
  {
    pwm = pw;
    preferences.putInt("pwm", pwm);
  }
  if (action.data.containsKey("fan"))
  {
    fan = action.data["fan"];
    preferences.putInt("fan", fan);
  }

  if (action.data.containsKey("outPow"))
  {
    outPow = action.data["outPow"];
    preferences.putBool("outPow", outPow);
    if (outPow)
      softOn();
    else
      softOff();
  }

  if (action.data.containsKey("toggle"))
  {
    outPow = !outPow;
    if (outPow)
      softOn();
    else
      softOff();
  }

  if (action.data.containsKey("reset"))
  {
    // reset = !reset;
    preferences.putFloat("totalEnergy", 0);
    preferences.putFloat("capacity_mAh", 0);
    // startTime = now;
    // startTimeStr = getTimeStr();
    totalEnergy = 0;
    capacity_mAh = 0;
  }

  String jsonString;
  serializeJson(action.data, jsonString);
  Serial.println(jsonString);
}

void sendData()
{
  automata.sendData(doc);
}
void current_measure_init()
{
  if (!ina219_b.begin())
  {
    Serial.println("Failed to find INA219_B chip");
  }
  if (!ina219_a.begin())
  {
    Serial.println("Failed to find INA219_B chip");
  }
}
void initBMP()
{
  if (!bmp.begin(0x76))
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}
void getData()
{
  pwm = preferences.getInt("pwm", 0);
  fan = preferences.getInt("fan", 0);
  outPow = preferences.getBool("outPow", false);
  totalEnergy = preferences.getFloat("totalEnergy", 0);
  capacity_mAh = preferences.getFloat("capacity_mAh", 0);
}

void setup()
{
  Serial.begin(115200);
  delay(200);
  led.begin();
  led.setBrightness(5);
  led.setPixelColor(0, 180, 250, 50);
  led.show();
  pinMode(PIN, OUTPUT);
  pinMode(FAN, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  analogWrite(PIN, 20);
  analogWrite(FAN, fan);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  preferences.begin("bat", false);
  // preferences.putFloat("totalEnergy", 0);
  //   preferences.putFloat("capacity_mAh", 0);
  getData();

  automata.begin();
  // automata.addAttribute("C1_SHUNT", "Shunt Volt Ch 1", "V");
  // automata.addAttribute("C2_SHUNT", "Shunt Volt Ch 2", "V");
  // automata.addAttribute("C1_VOLT", "Volt Ch 1", "V");
  // automata.addAttribute("C2_VOLT", "Volt Ch 2", "V");

  automata.addAttribute("C1_CURR", "C1", "A", "DATA|AUX");
  automata.addAttribute("C2_CURR", "C2", "A", "DATA|AUX");
  automata.addAttribute("C1_POWER", "P1", "W", "DATA|AUX");
  automata.addAttribute("C2_POWER", "P2", "W", "DATA|AUX");

  // automata.addAttribute("C1", "V1", "V", "DATA|AUX");
  // automata.addAttribute("C2", "V2", "V", "DATA|AUX");
  // automata.addAttribute("C3", "V3", "V", "DATA|AUX");
  // automata.addAttribute("C4", "V4", "V", "DATA|AUX");

  // automata.addAttribute("shuntVoltage", "Shunt Volt", "V");
  automata.addAttribute("busVoltage", "Voltage", "V", "DATA|AUX");
  automata.addAttribute("power", "Power", "W", "DATA|MAIN");
  automata.addAttribute("current", "Current", "A", "DATA|MAIN");
  automata.addAttribute("percent", "Percent", "%", "DATA|MAIN");
  automata.addAttribute("temp", "Temp", "°C", "DATA|MAIN");
  automata.addAttribute("totalEnergy", "Energy", "Wh", "DATA|MAIN");
  // automata.addAttribute("loadVoltage", "Load Volt", "V");
  automata.addAttribute("capacity", "Capacity", "Ah", "DATA|MAIN");
  JsonDocument doc;
  doc["max"] = 255;
  doc["min"] = 0;
  doc["values"] = "0,255";

  automata.addAttribute("pwm", "Light", "", "ACTION|SLIDER", doc);
  automata.addAttribute("fan", "Fan", "", "ACTION|MENU|SLIDER", doc);
  // automata.addAttribute("remaining", "Remaining Time", "Date");

  automata.addAttribute("outPow", "Light", "", "ACTION|SWITCH");
  automata.addAttribute("toggle", "Toggle", "", "ACTION|MENU|BTN");
  automata.addAttribute("reset", "Reset", "", "ACTION|MENU|BTN");
automata.addAttribute("button", "Button", "", "ACTION|MENU|BTN");
  automata.addAttribute("dischargingTime", "Time Left", "Hr", "DATA|MAIN");
  // automata.addAttribute("upTime", "Up Time", "Hours", "DATA|MAIN");

  automata.registerDevice();
  automata.onActionReceived(action);
  automata.delayedUpdate(sendData);
  current_measure_init();
  initBMP();
  startMillis = millis();
  analogWrite(PIN, 20);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  udp.begin(udpPort);
}

String getTimeStr()
{
  String dateTime;
  struct tm timeInfo;
  if (getLocalTime(&timeInfo))
  {
    dateTime = String(timeInfo.tm_year + 1900) + "/" + String(timeInfo.tm_mon + 1) + "/" + String(timeInfo.tm_mday) + " " + String(timeInfo.tm_hour) + ":" + String(timeInfo.tm_min) + ":" + String(timeInfo.tm_sec);
  }
  return dateTime;
}

void saveData()
{
  preferences.putFloat("totalEnergy", totalEnergy);
  preferences.putFloat("capacity_mAh", capacity_mAh);
}

void upt()
{
  unsigned long currentMillis = millis();
  unsigned long uptimeMillis = currentMillis - startMillis; // Calculate total uptime in milliseconds

  unsigned long uptimeSeconds = uptimeMillis / 1000; // Convert milliseconds to seconds
  unsigned long seconds = uptimeSeconds % 60;        // Calculate remaining seconds
  unsigned long minutes = (uptimeSeconds / 60) % 60; // Calculate remaining minutes
  unsigned long hours = uptimeSeconds / 3600;        // Calculate total hours

  char uptimeString[12];                                               // Buffer to hold the formatted uptime string
  sprintf(uptimeString, "%02lu:%02lu:%02lu", hours, minutes, seconds); // Format uptime into HH:MM:SS

  // Serial.print("Uptime: ");
  // Serial.println(uptimeString);
  uptime = String(uptimeString);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void readBMP()
{
  if (bmp.takeForcedMeasurement())
  {
    temp = bmp.readTemperature();
    pressure = bmp.readPressure();
  }
  else
  {
    Serial.println("Forced measurement failed!");
  }
}
void readPower()
{

  c1_shunt = ina219_a.getShuntVoltage_mV();
  c1_volt = ina219_a.getBusVoltage_V();
  c1_curr = ina219_a.getCurrent_mA() * 40;

  c2_shunt = ina219_b.getShuntVoltage_mV();
  c2_volt = ina219_b.getBusVoltage_V();
  c2_curr = ina219_b.getCurrent_mA() * 10;

  c1_curr = c1_curr / 1000;

  c1_pow = c1_volt * c1_curr;

  c2_curr = c2_curr / 1000;

  c2_pow = c2_volt * c2_curr;

  busvoltage = (c1_volt + c2_volt) / 2;
  shuntvoltage = (c1_shunt + c2_shunt) / 2;
  float curr = 0;
  if (c1_curr > c2_curr)
    curr = c1_curr + c2_curr;
  else
    curr = c2_curr + c1_curr;

  current_mA = curr;
  power_mW = busvoltage * current_mA;
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  float timeInterval = (currentMillis - previousMillis) / 1000.0;

  totalEnergy += power_mW * (timeInterval / (60 * 60));
  capacity_mAh += current_mA * (timeInterval / (60 * 60));
  isDischarge = curr < 0 ? "DISCHARGING" : "CHARGING";
  percent = mapf(busvoltage, 19.2, 25.2, 0.0, 100.0);

  if (isDischarge == "CHARGING")
  {
    chargingTimeHours = (targetCapacity - capacity_mAh) / (power_mW / loadvoltage);
  }
  else
  {
    dischargingTimeHours = (targetCapacity - abs(capacity_mAh)) / abs(current_mA);
  }

  if (percent < 25)
  {
  }

  if (percent > 100)
  {
    startTime = now;
    startTimeStr = getTimeStr();
    percent = 100;
  }
  if (percent < 0)
    percent = 0;
  previousMillis = currentMillis;
}

void listenUDP()
{
  // if (WiFi.status() == WL_CONNECTED)
  // {
  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    int len = udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = '\0';
    }
    Serial.printf("Received packet: %s\n", incomingPacket);
    Serial.print("From IP: ");
    Serial.println(udp.remoteIP());
  }

  // }
}
float readVoltage(int pin, float ratio)
{
  int rawADC = analogRead(pin);
  float voltage = (rawADC / 4095.0) * referenceVoltage;
  return voltage * ratio;
}

void readCell()
{
  float cell1Voltage = readVoltage(adcPin1, dividerRatio1);
  float cell2Voltage = readVoltage(adcPin2, dividerRatio2);
  float cell3Voltage = readVoltage(adcPin3, dividerRatio3);
  float cell4Voltage = readVoltage(adcPin4, dividerRatio4);

  // Calculate actual cell voltages
  actualCell1 = cell1Voltage;
  actualCell2 = cell2Voltage - cell1Voltage;
  actualCell3 = cell3Voltage - cell2Voltage;
  actualCell4 = cell4Voltage - cell3Voltage;

  // Print the voltages to Serial Monitor
  // Serial.print("Cell 1 Voltage: "); Serial.print(actualCell1); Serial.println(" V");
  // Serial.print("Cell 2 Voltage: "); Serial.print(actualCell2); Serial.println(" V");
  // Serial.print("Cell 3 Voltage: "); Serial.print(actualCell3); Serial.println(" V");
  // Serial.print("Cell 4 Voltage: "); Serial.print(actualCell4); Serial.println(" V");
}
void loop()
{

  led.setPixelColor(0, 0, 0, 250);
  led.show();
  automata.loop();
  readBMP();
  readPower();
  // upt();
  // listenUDP();
  // time(&now);
  // readCell();
  doc["temp"] = temp;
  // doc["C1"] = String(actualCell1 * calibrationFactor1, 2);
  // doc["C2"] = String(actualCell2 * calibrationFactor2, 2);
  // doc["C3"] = String(actualCell3 * calibrationFactor3, 2);
  // doc["C4"] = String(actualCell4 * calibrationFactor4, 2);
  // if (digitalRead(0) == LOW)
  // {
  //   preferences.putFloat("totalEnergy", 0);
  //   preferences.putFloat("capacity_mAh", 0);
  //   startTime = now;
  //   startTimeStr = getTimeStr();
  //   totalEnergy = 0;
  //   capacity_mAh = 0;
  //   delay(1000);
  // }

  // doc["C1_SHUNT"] = String(c1_shunt, 2);
  // doc["C2_SHUNT"] = String(c2_shunt, 2);
  // doc["C1_VOLT"] = String(c1_volt, 2);
  // doc["C2_VOLT"] = String(c2_volt, 2);
  doc["pwm"] = pwm;
  doc["fan"] = fan;
  doc["outPow"] = outPow;
  doc["reset"] = reset;
  doc["C1_CURR"] = String(c1_curr, 2);
  doc["C1_POWER"] = String(c1_pow, 2);
  doc["C2_CURR"] = String(c2_curr, 2);
  doc["C2_POWER"] = String(c2_pow, 2);
  // doc["shuntVoltage"] = String(shuntvoltage, 3);
  doc["busVoltage"] = String(busvoltage, 2);
  doc["current"] = String(current_mA, 2);
  doc["power"] = String(power_mW, 2);
  doc["totalEnergy"] = String(totalEnergy, 2);
  // doc["button"] = digitalRead(BTN_PIN);
  // doc["loadVoltage"] = String(loadvoltage, 3);
  doc["percent"] = String(percent, 2);
  doc["capacity"] = String(capacity_mAh, 2);
  // doc["dateTime"] = getTimeStr();
  // doc["status"] = isDischarge;
  // doc["startTime"] = startTimeStr;
  doc["dischargingTime"] = String(dischargingTimeHours, 2);
  // time(&now);
  // doc["upTime"] = uptime;
  if (outPow)
    analogWrite(PIN, pwm);
  else
    analogWrite(PIN, 0);

  analogWrite(FAN, fan);

  if (digitalRead(BTN_PIN) == LOW)
  {
    JsonDocument doc;
    doc["button"] = digitalRead(BTN_PIN);
    doc["key"] = "button";
    automata.sendAction(doc);

    led.setPixelColor(0, 250, 0, 0);
    led.show();
    delay(800);
  }

  if ((millis() - start) > 500)
  {
    led.setPixelColor(0, 250, 250, 250);
    led.show();
    automata.sendLive(doc);
    start = millis();
  }
  if ((millis() - st) > d)
  {
    saveData();
    st = millis();
  }

  delay(100);
  led.setPixelColor(0, 0, 0, 0);
  led.show();
}
