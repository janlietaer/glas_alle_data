//MAC servor 30:AE:A4:84:1F:14
//MAC  glas  24:6F:28:99:A2:D0
//https://esp32.com/viewtopic.php?t=13435

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Button2.h>
#include "esp_adc_cal.h"
#include "TTGO_T_Display.h"
#include "bmp.h"
#include <EEPROM.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;

#define CHANNEL 3
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0
#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN 0x10
#endif

#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_CS 5
#define TFT_DC 16
#define TFT_RST 23

#define TFT_BL 4 // Display backlight control pin
#define ADC_EN 14
#define ADC_PIN 34
#define BUTTON_1 35
#define BUTTON_2 0

typedef struct Glas_Servo_struct_esp_now
{
  uint16_t data1;
  uint32_t data2;
  uint32_t telnr;
  uint8_t wat;
} Glas_Servo_struct_esp_now;

Glas_Servo_struct_esp_now Glas_naar_servo;

typedef struct Servo_glas_struct_esp_now
{
  uint16_t nr;
  uint32_t data1;
  uint32_t data2;
  byte actie;
} Servo_glas_struct_esp_now;

Servo_glas_struct_esp_now Servo_naar_glas;

//MAC address van servo 30:AE:A4:84:1F:14
uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0x84, 0x1F, 0x14};

unsigned long timet;
unsigned long timetB;
unsigned long timevorige;
unsigned long timenu;
unsigned long time_now = 0;
unsigned long vorigesensor;

uint8_t system2;
uint8_t gyro;
uint8_t accel;
uint8_t mag;
boolean calibrate = false;
boolean writecalibratefat = false;
boolean isPaired = true;

String success;

static volatile boolean Halt_Door_Nieuwe_Communicatie = false;
static volatile boolean STOP = false;
static volatile int32_t nieuw_doel;
static volatile int32_t nieuw_van;
static volatile int32_t Huidige;
int looptellersensor = 0;
double xdata = 0;
double ydata = 0;
double zdata = 0;

uint16_t GeslaagdVerstuurd, Misluktgestuurd, GeslaagdOntvangen;

esp_now_peer_info_t slave;
sensors_event_t orientationData, angVelocityData, linearAccelData, VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER, VECTOR_GRAVITY;
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library

TaskHandle_t Task1;
TaskHandle_t Task2;

void printEvent(sensors_event_t *event, String Wat)
{
 
  double x = -1000000, y = -1000000, z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER)
  { x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;  }

  xdata = x;
  ydata = y;
  zdata = z;
}



void schermschrijven(double xx, double yy, double zz)
{
  String dd;

  if (abs(zz) < 10)
  {
    dd = String(zz, 2);
  }
  else
  {
    dd = String(zz, 0);
  }

  tft.fillScreen(TFT_WHITE);
  if (zz > 0)
  {
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.drawString("Z " + dd, tft.width() / 2 - 8, tft.height() / 2, 4);
  }
  else
  {
    tft.setTextColor(TFT_RED, TFT_WHITE);
    tft.drawString("Z " + dd, tft.width() / 2 - 8, tft.height() / 2, 4);
  }

  if (abs(xx) < 10)
  {
    dd = String(xx, 2);
  }
  else
  {
    dd = String(xx, 0);
  }
  if (xx > 0)
  {
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.drawString("X " + dd, (tft.width() / 2) - 8, 2, 4);
  }
  else
  {
    tft.setTextColor(TFT_RED, TFT_WHITE);
    tft.drawString("X " + dd, (tft.width() / 2) - 18, tft.height() - 6, 4);
  }

  if (abs(xx) < 10)
  {
    dd = String(yy, 2);
  }
  else
  {
    dd = String(yy, 0);
  }
  if (yy > 0)
  {
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.drawString("Y " + dd, 0, tft.height() / 2, 4);
  }
  else
  {
    tft.setTextColor(TFT_RED, TFT_WHITE);
    tft.drawString("Y " + dd, tft.width() - 25, tft.height() / 2, 4);
  }
}

void codeForTaskScherm(void *parameter)
{
  tft.setRotation(1);
  tft.fillScreen(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  for (;;)
  {
    schermschrijven(xdata, ydata, zdata);
    delay(200);
  }
};

void codeForTaskSensor(void *parameter)
{
  for (;;)
  {
    delay(1000);
    looptellersensor++;
    if (looptellersensor == 4000)
    {
      looptellersensor = 0;
      Serial.print("4000 sensoruitlezingen in :");
      Serial.println(millis() - vorigesensor);
      vorigesensor = millis();
    }
     bno.getEvent(&VECTOR_GRAVITY, Adafruit_BNO055::VECTOR_GRAVITY);
    printEvent(&VECTOR_GRAVITY, "V_GRAVITY");
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}

// Init ESP Now with fallback
void InitESPNow()
{
  WiFi.disconnect();
  Glas_naar_servo.telnr = 0;
  Glas_naar_servo.data1 = 0;
  Glas_naar_servo.data2 = 0;

  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESPNow Init Success");
  }
  else
  {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}
 void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
 {
   if (status == ESP_NOW_SEND_SUCCESS)
   {
     GeslaagdVerstuurd++;
   }
   else
   {
     Misluktgestuurd++;
   }
 }

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&Servo_naar_glas, incomingData, sizeof(Servo_naar_glas));

  Serial.print("VAN SERVO  Bytes received: ");

  Serial.print(len);
  Serial.print("\t");
  Serial.print("Nr: ");
  Serial.print(Servo_naar_glas.nr);
  Serial.print("\t");
  Serial.print("Data1: ");
  Serial.print(Servo_naar_glas.data1);
  Serial.print("\t");
  Serial.print("Data2: ");
  Serial.print(Servo_naar_glas.data2);
  Serial.print("\t");
  Serial.print("Actie: ");
  Serial.println(Servo_naar_glas.actie);


  switch (Servo_naar_glas.actie)
  {
 
  case 3:
    nieuw_van = Servo_naar_glas.data1;
    nieuw_doel = Servo_naar_glas.data2;
    STOP = false;
    break;
  case 7:
    STOP = true;
    break;
  }

  Halt_Door_Nieuwe_Communicatie = true;
}

void Hoofdtaak_Data_naar_Servo()
{
  int32_t step_van;
 
  int32_t step_doel;

 
  step_van = nieuw_van;
  if (nieuw_van != Huidige)
  {
    Serial.print("FOUT verloren stappen:");
     Serial.print("    Huidige:");
    Serial.print(Huidige);
    Serial.print("   nieuw_van:");
    Serial.println(nieuw_van);   
   
    Huidige= nieuw_van;
  }

  Halt_Door_Nieuwe_Communicatie = false;

  while ((Huidige != nieuw_doel) && (STOP == false))
  {

    step_van = Huidige;
    step_doel = Huidige ;
    if (Huidige > (nieuw_doel + 100))
    {
      step_doel = Huidige - 100;
    }
    else if (Huidige > nieuw_doel)
    {
      step_doel = Huidige - 1;
    }
    else if (Huidige < (nieuw_doel - 100))
    {
      step_doel = Huidige + 100;
    }
    else if (Huidige < nieuw_doel)
    {
      step_doel = Huidige + 1;
    }

    Glas_naar_servo.telnr++;
    Glas_naar_servo.data1 = step_van;
    Glas_naar_servo.data2 = step_doel;
    Glas_naar_servo.wat = 1;

    esp_now_send(broadcastAddress, (uint8_t *)&Glas_naar_servo, sizeof(Glas_naar_servo));
/**
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&Glas_naar_servo, sizeof(Glas_naar_servo));

    if (result == ESP_OK)
    {
      GeslaagdVerstuurd++;
    }
    else
    {
      Misluktgestuurd++;
    }
**/
    Huidige = step_doel;

    delay(5);
  }
  Servo_naar_glas.actie = 0;
}

void setup()
{
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Checkk your wiring or I2C ADDR!");
    while (1)
      ;
  }

  int eeAddress = 0;
  long bnoID;
  EEPROM.get(eeAddress, bnoID);
  adafruit_bno055_offsets_t calibrationData;

  //Sets calibration from read data
  eeAddress += sizeof(long);
  EEPROM.get(eeAddress, calibrationData);
  bno.setSensorOffsets(calibrationData);
  //hold

  delay(100);
  bno.setExtCrystalUse(true); //mag en  moet is aanwezig  !!!
  Serial.begin(115200);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_MODE_STA);
  Serial.println("ESPNow/Basic/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic

  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  timetB = micros();

  Serial.println("Start");
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0, 0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);

  if (TFT_BL > 0)
  {                                         // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
    pinMode(TFT_BL, OUTPUT);                // Set backlight pin to output mode
    digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
  }
  tft.setSwapBytes(true);
  tft.pushImage(0, 0, 135, 240, ttgo);
  delay(100);
  tft.setRotation(0);
  int i = 1;
  while (i--)
  {
    tft.fillScreen(TFT_RED);
    delay(100);
    tft.fillScreen(TFT_BLUE);
    delay(100);
    tft.fillScreen(TFT_GREEN);
    delay(100);
  }

  tft.fillScreen(TFT_WHITE);

  xTaskCreatePinnedToCore(
      codeForTaskSensor,              /* Task function. */
      "Task_Sensor",                  /* name of task. */
      10000, /* Stack size of task */ //deze groter maken als het mis gaat
      NULL,                           /* parameter of the task */
      1,                              /* priority of the task */
      &Task1,                         /* Task handle to keep track of created task */
      0);                             /* Core */

  xTaskCreatePinnedToCore(
      codeForTaskScherm,              /* Task function. */
      "Task_scherm",                  /* name of task. */
      10000, /* Stack size of task */ //deze groter maken als het mis gaat
      NULL,                           /* parameter of the task */
      2,                              /* priority of the task */
      &Task2,                         /* Task handle to keep track of created task */
      0);                             /* Core */

  timevorige = millis();
  vorigesensor = millis();
}

void WachtenMicros(unsigned long AantalMicros)
{
  time_now = micros();
  time_now = time_now + AantalMicros;
  do
  {
  } while (time_now > micros());
}

void loop()
{
  if (Halt_Door_Nieuwe_Communicatie == true)
  {
    Hoofdtaak_Data_naar_Servo();
    Serial.print("NAARSERVO Misluktgestuurd : ");
    Serial.print(Misluktgestuurd);
    Serial.print("\t");
    Serial.print("   GeslaagdVerstuurd : ");
    
    Serial.println(GeslaagdVerstuurd);
  }
}