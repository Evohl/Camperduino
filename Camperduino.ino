#include <U8g2lib.h>
#include <RTClib.h>
#include "DHT.h"
#include <SoftwareSerial.h>

// Serial output for debugging 0 = off; 1 = BLE; 2 = GasSensor; 3 = Controller ; 4 = Ultrsonic
int debug = 1;

#define LCDWidth                        u8g2.getDisplayWidth()
#define ALIGN_CENTER(t)                 ((LCDWidth - (u8g2.getUTF8Width(t))) / 2)

#define Logo_width 100
#define Logo_height 49
static const unsigned char Logo_bits[] U8X8_PROGMEM  = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x1c,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
  0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x80, 0x01, 0x00, 0xc0,
  0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0xc0, 0x03, 0x08,
  0xf0, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0xc0, 0x03,
  0x10, 0xf8, 0x07, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0xe0,
  0x03, 0x30, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00,
  0xe0, 0x01, 0x60, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
  0x00, 0xe0, 0x01, 0xe0, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x0f, 0x00, 0xf0, 0x01, 0xe0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc,
  0x00, 0x0f, 0x00, 0xf0, 0x00, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xfe, 0x01, 0x0f, 0x00, 0xf0, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x80,
  0x81, 0xff, 0x07, 0xff, 0x01, 0xf8, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x1c,
  0xc0, 0xc3, 0xff, 0x07, 0xff, 0x0f, 0xf8, 0x00, 0xf0, 0x01, 0x00, 0x00,
  0x3e, 0xe0, 0xc3, 0x8f, 0x0f, 0xff, 0x0f, 0x78, 0x00, 0xf0, 0x01, 0x00,
  0x00, 0x3e, 0xe0, 0xc3, 0x03, 0x0f, 0xff, 0x1f, 0x78, 0x00, 0xf0, 0x00,
  0x00, 0x00, 0x7c, 0xf0, 0xe1, 0x03, 0x1e, 0x1f, 0x3e, 0x7c, 0x00, 0xf8,
  0x00, 0x70, 0x00, 0x7c, 0xf0, 0xe1, 0x01, 0x1e, 0x0f, 0x3c, 0x3c, 0x00,
  0xf8, 0x00, 0xf8, 0x00, 0xf8, 0xf8, 0xe0, 0x01, 0x9e, 0x0f, 0x3c, 0x3c,
  0x00, 0x78, 0x30, 0xfc, 0x01, 0xf8, 0xf8, 0xe0, 0x01, 0x9e, 0x0f, 0x3c,
  0x3e, 0x00, 0x78, 0x78, 0xfc, 0x01, 0xf0, 0x78, 0xe0, 0x03, 0x8f, 0x0f,
  0x3c, 0x3e, 0x00, 0x78, 0xfc, 0xfc, 0x01, 0xf0, 0x7d, 0xe0, 0x03, 0x8f,
  0x0f, 0x3c, 0x1e, 0x00, 0x78, 0xfc, 0xf8, 0x00, 0xe0, 0x3d, 0xc0, 0xcf,
  0x8f, 0x0f, 0x3e, 0x1e, 0x00, 0x78, 0x78, 0x70, 0x00, 0xe0, 0x3f, 0xc0,
  0xff, 0x87, 0x0f, 0x3e, 0x1e, 0x00, 0xf8, 0x30, 0x00, 0x00, 0xc0, 0x1f,
  0x80, 0xff, 0x03, 0x07, 0x1e, 0x3e, 0x00, 0xf8, 0x00, 0x00, 0x00, 0xc0,
  0x1f, 0x00, 0xfe, 0x01, 0x00, 0x1e, 0x7c, 0x00, 0xf0, 0x00, 0x00, 0x00,
  0xc0, 0x0f, 0x00, 0x78, 0x00, 0x00, 0x0c, 0xfc, 0x01, 0xf0, 0x01, 0x00,
  0x00, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x01, 0xe0, 0x01,
  0x00, 0x00, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0xe0,
  0x03, 0x00, 0x00, 0x80, 0x07, 0x00, 0x00, 0xfc, 0x07, 0x00, 0x00, 0x00,
  0xc0, 0x07, 0x00, 0x00, 0x00, 0x03, 0x00, 0xfc, 0xff, 0xff, 0x03, 0x00,
  0x00, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x7f,
  0x00, 0x00, 0x80, 0x1f, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0x03, 0x00,
  0xf8, 0x0f, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x1f, 0x00,
  0x00, 0x00, 0x38, 0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0xfe, 0xff, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xfc, 0x03, 0x00, 0x00, 0xff, 0x1f,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x0f, 0x00, 0xfc, 0xff,
  0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0xf0, 0xff,
  0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff,
  0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff,
  0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xfe, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xc0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00
};

#define wlogo_width 20
#define wlogo_height 20
static const unsigned char wlogo_bits[] U8X8_PROGMEM  = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0xe0, 0x3d, 0x00,
  0x00, 0x07, 0x00, 0x00, 0x02, 0x00, 0x00, 0x02, 0x00, 0x00, 0x07, 0x00,
  0x84, 0x0d, 0x00, 0xfc, 0xf8, 0x00, 0x44, 0x10, 0x01, 0xfc, 0x78, 0x02,
  0x84, 0x8d, 0x02, 0x00, 0x87, 0x02, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define wlogow_width 20
#define wlogow_height 20
static const unsigned char wlogow_bits[] U8X8_PROGMEM  = {
   0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0xe0, 0x3d, 0x00, 0x00, 0x07, 0x00,
   0x00, 0x02, 0x00, 0x00, 0x02, 0x00, 0x00, 0x07, 0x00, 0x84, 0x0d, 0x00,
   0xfc, 0xf8, 0x00, 0x44, 0x10, 0x01, 0xfc, 0x78, 0x02, 0x84, 0x8d, 0x02,
   0x00, 0x87, 0x02, 0x00, 0x80, 0x03, 0xc2, 0x39, 0x00, 0x24, 0x4b, 0x00,
   0x38, 0xc9, 0x01, 0x20, 0x4b, 0x02, 0xc0, 0x39, 0x04, 0x00, 0x00, 0x00
};

#define glogo_width 20
#define glogo_height 20
static const unsigned char glogo_bits[] U8X8_PROGMEM  = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x1f, 0x00, 0x40, 0x20, 0x00,
  0x40, 0x2f, 0x00, 0x40, 0x26, 0x00, 0xc0, 0x3f, 0x00, 0x60, 0x60, 0x00,
  0x20, 0x40, 0x00, 0x20, 0x40, 0x00, 0xa0, 0x5f, 0x00, 0x20, 0x40, 0x00,
  0x20, 0x40, 0x00, 0x20, 0x40, 0x00, 0x60, 0x60, 0x00, 0xc0, 0x3f, 0x00,
  0x60, 0x60, 0x00, 0xe0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define warn_width 20
#define warn_height 20
static const unsigned char warn_bits[] U8X8_PROGMEM  = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x09, 0x00,
  0x00, 0x09, 0x00, 0x80, 0x10, 0x00, 0x80, 0x10, 0x00, 0x40, 0x26, 0x00,
  0x40, 0x26, 0x00, 0x20, 0x46, 0x00, 0x20, 0x46, 0x00, 0x10, 0x86, 0x00,
  0x10, 0x86, 0x00, 0x08, 0x06, 0x01, 0x08, 0x00, 0x01, 0x04, 0x06, 0x02,
  0x04, 0x06, 0x02, 0x04, 0x00, 0x02, 0xf8, 0xff, 0x01, 0x00, 0x00, 0x00
};

#define nocon_width 20
#define nocon_height 20
static const unsigned char nocon_bits[] U8X8_PROGMEM  = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0xa0, 0x00,
  0x00, 0x10, 0x01, 0x00, 0x50, 0x01, 0x00, 0x48, 0x02, 0x00, 0x08, 0x02,
  0x1e, 0x44, 0x04, 0x60, 0x04, 0x04, 0x8e, 0xf8, 0x03, 0x30, 0x01, 0x00,
  0x46, 0x02, 0x00, 0x98, 0x04, 0x00, 0x20, 0x05, 0x00, 0x4c, 0x09, 0x00,
  0x52, 0x0a, 0x00, 0x92, 0x0a, 0x00, 0x8c, 0x0a, 0x00, 0x00, 0x00, 0x00
};

// ********************* Configuration *************************
#define TempSensorPin 26
#define tempwarnlevel 5

//
RTC_DS3231 rtc;

// Bluetooth
HardwareSerial SerialBT = Serial3;
String readData = "";  
char sendData = "";

// Ultrasonic
static short echoPin = 17;
static short trigPin = 16;
static short Tankheight = 22;   // Gesammthöhe cm
static short Tankfull = 19;     // von Boden bis überlauf cm
#define Volumen 20.5    // Liter

// Rotary Controller
#define CLK 3
#define DT 4
#define BTN 2

int rcounter = 0;
int currentStateCLK;
int lastStateCLK;

volatile boolean turned;   // rotary was turned
volatile boolean fired;    // knob was pushed
volatile boolean up;  // true when turned cw

// Gas Sensor
#define Gas_analog 4
#define Gas_digital 25
int GasThres = 150;

// Display
//U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ 18, /* data=*/ 23, /* CS=*/ 5, /* reset=*/ 13); // ESP32
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=(blau LCD E)*/ 13, /* data=(Aubergine LCD RW)*/ 11, /* CS=(Grau LCD RS)*/ 10, /* reset=*/ 8);
// ********************** End Config ***************************

// Buzzer
short freq = 2000;
static short channel = 9;
static short defaultdelay = 100;
static short mainscreendelay = 200;

// Page State
int State;

// Temp Sensor
DHT dht11(TempSensorPin, DHT11);

// Ultrasonic
long duration; // variable for the duration of sound wave travel
int distance;
int levelact;
int volumenact;

float Gaslevel = 0;
float WaterLevel = 0;
short waterlevelpos = 108;

short menupos = 0;
static short menuitems = 4;
static short menulinehight = 12;

short settingsmenupos = 0;
static short settingsmenuitems = 3;
static short settingsmenulinehight = 12;

short devselectpos = 0;
static short devselectitems = 0;
static short devselectlinehight = 12;

DateTime aktuell;
bool Sommerzeit;
int hournow;
int minutenow;
int yearnow;
int monthnow;
int daynow;
int timesetupstate;

void ListenBLE(){
   if (SerialBT.available()){      // Daten liegen an
      readData = SerialBT.read(); // Nachricht lesen
      Serial.print(readData);
   }  
   while (Serial.available() ) {  
    sendData = Serial.read();  
    if (sendData != 0) {           // Read user input if available.
      delay(10);
      Serial.print(sendData);
      SerialBT.write(sendData);    
    } 
   }
   sendData = 0;
}

long bauds[] = {
    // major 
    9600, 57600, 115200,
    
    // others
    19200, 38400, 4800, 2400, 1200, 230400
  };

bool detectBleBaudRate() {
  Serial.println("Detecting BLE baud rate:");
  for (int i=0; i<(sizeof(bauds)/sizeof(long)); i++) {
    Serial.write("Checking ");
    long cur_baud = bauds[i];
    Serial.println(cur_baud, DEC);
    
    SerialBT.begin(cur_baud);
    SerialBT.write("AT");
    SerialBT.flush();
    delay(50);
    String response = SerialBT.readString();
    if (response == "OK") {
      Serial.println("Detected");
      return true;
    } else {
       SerialBT.end();
    }
  }
  return false;
}

void TempWarn() {
}

void u8g2_prepare(void) {
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void MainScreen(float Temperature, float Humidity, float GasLevel, float WaterLevel, bool waterwarn) {

  u8g2_prepare();
  u8g2.clearBuffer();

  String temp_str;
  String hum_str;
  String gas_str;
  String water_str;

  temp_str = String(Temperature,1); //converting ftemp (the float variable above) to a string 
  hum_str = String(Humidity,0); //converting Humidity (the float variable above) to a string
  gas_str = String(GasLevel,0); //converting GasLevel (the float variable above) to a string 
  water_str = String(WaterLevel,0); //converting WaterLevel (the float variable above) to a string
  
  waterlevelpos = 108;

  String wochentag;
  switch (aktuell.dayOfTheWeek())
  {
    case 0:
      wochentag = "Sonntag";
      break;
    case 1:
      wochentag = "Montag";
      break;
    case 2:
      wochentag = "Dienstag";
      break;
    case 3:
      wochentag = "Mittwoch";
      break;
    case 4:
      wochentag = "Donnerstag";
      break;
    case 5:
      wochentag = "Freitag";
      break;
    case 6:
      wochentag = "Samstag";
      break;
  }

  String monat;
  switch (aktuell.month())
  {
    case 1:
      monat = "Januar";
      break;
    case 2:
      monat = "Februar";
      break;
    case 3:
      monat = "März";
      break;
    case 4:
      monat = "April";
      break;
    case 5:
      monat = "Mai";
      break;
    case 6:
      monat = "Juni";
      break;
    case 7:
      monat = "Juli";
      break;
    case 8:
      monat = "August";
      break;
    case 9:
      monat = "September";
      break;
    case 10:
      monat = "Oktober";
      break;
    case 11:
      monat = "November";
      break;
    case 12:
      monat = "Dezember";
      break;
   }
   
  u8g2.setFontMode(1);
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_t0_18_tf);
  String warn = " ";
  char buf2[] = "hh:mm"; 
  u8g2.drawUTF8( 2, 0, aktuell.toString(buf2));
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawUTF8( 3, 16, wochentag.c_str());
  u8g2.drawUTF8( 3, 25, (String(aktuell.day()) + " " + monat + " " + String(aktuell.year())).c_str());
  u8g2.setFont(u8g2_font_t0_11_tf);
  if (Temperature <= tempwarnlevel) { 
    warn = "* ";
    TempWarn();
  }
  u8g2.drawLine( 3, 15, 62, 15);
  u8g2.drawLine( 3, 34, 62, 34);
  u8g2.drawUTF8( 2, 35, (temp_str + "°C" + warn + hum_str + "%").c_str());
  u8g2.setFont(u8g2_font_5x7_tf);

  int plusvalue_w = 14;
  int plusvalue_g = 14;

  if (water_str == "100") plusvalue_w++;
  if (gas_str == "100") plusvalue_g++;

  u8g2.setDrawColor(1);
  u8g2.drawBox(waterlevelpos, 64 - (WaterLevel / 100 * 64), 20, (WaterLevel / 100 * 64) + 1);
  u8g2.setDrawColor(2);
  u8g2.drawUTF8(waterlevelpos + plusvalue_w - u8g2.getUTF8Width(water_str.c_str()), 56, (water_str + "%").c_str());
  if (waterwarn) u8g2.drawXBMP(waterlevelpos, 0, wlogow_width, wlogow_height, wlogow_bits);
  else u8g2.drawXBMP(waterlevelpos, 0, wlogo_width, wlogo_height, wlogo_bits);
  u8g2.setDrawColor(1);
  u8g2.sendBuffer();
}

void MainMenu() {
  u8g2_prepare();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_profont11_tf);
  String myarrow = F("->");
  if (menupos == 0) myarrow = "<-";
  u8g2.drawUTF8( 0, menupos, myarrow.c_str());
  u8g2.drawUTF8( 14, 0, "Zurück");
  u8g2.drawUTF8( 14, 12, "Informationen");
  u8g2.drawUTF8( 14, 24, "Einstellungen");
  u8g2.drawUTF8( 14, 36, "Über");
  u8g2.sendBuffer();
}
void Settings() {
  u8g2_prepare();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_profont11_tf);
  String myarrow = "->";
  if (settingsmenupos == 0) myarrow = F("<-");
  u8g2.drawUTF8( 0, settingsmenupos, myarrow.c_str());
  u8g2.drawUTF8( 14, 0, "Zurück");
  u8g2.drawUTF8( 14, 12, "Zeit Einstellung");
  u8g2.drawUTF8( 14, 24, "Wasser Indikatror");
  u8g2.sendBuffer();
}

int WaterArcusticWarn(float WaterLevel) {
  u8g2_prepare();
  u8g2.clearBuffer();
  u8g2.setFontMode(1);
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_profont11_tf);
  if (WaterLevel > 100) WaterLevel = 100;
  if (WaterLevel < 0) WaterLevel = 0;
  if (WaterLevel < 90) freq = 2300;
  if (WaterLevel < 50) freq = 2000;
  if (WaterLevel > 90) freq = 2700;
  String water_str;
  water_str = String(WaterLevel, 0); //converting Humidity (the float variable above) to a string
  u8g2.drawUTF8(ALIGN_CENTER("Wasser Indikatror"), 20, "Wasser Indikatror");
  u8g2.drawUTF8(ALIGN_CENTER((water_str + "%").c_str()), 32, (water_str + "%").c_str());
  u8g2.sendBuffer();
  noTone(channel);
  delay(1000 - WaterLevel * 10 + 1);
  return 0;
}

void About() {
  u8g2_prepare();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_profont11_tf);
  u8g2.drawUTF8(ALIGN_CENTER("Camperduino V0.3"), 50, "Camperduino V0.3");
  u8g2.drawXBMP(((LCDWidth - Logo_width) / 2), 0, Logo_width, Logo_height, Logo_bits);
  u8g2.sendBuffer();
}

void Info() {
  u8g2_prepare();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_profont10_tf);
  u8g2.drawUTF8(0, 0, "Systeminformationen:");
  String twinfo = String("Tempwarnung: <" + String(tempwarnlevel) + "°C");
  u8g2.drawUTF8(5, 10, twinfo.c_str());
  u8g2.sendBuffer();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  u8g2.begin();
  dht11.begin();
  About();
  u8g2.setBitmapMode(1);
  pinMode(Gas_digital, INPUT);
  tone(channel, 1500);
  delay(100);
  tone(channel, 2000);
  delay(100);
  tone(channel, 2500);
  delay(100);
  noTone(channel);
  if (detectBleBaudRate())
    Serial.write("Ready, type AT commands\n\n");
  else
    Serial.write("Not ready. Halt");
  delay(2000);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  rtc.begin();
  pinMode(CLK,INPUT_PULLUP);
  pinMode(DT,INPUT_PULLUP);
  pinMode(BTN,INPUT_PULLUP);
  lastStateCLK = digitalRead(CLK);
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // Call updateEncoder() when any high/low changed seen
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(0, firebutton, FALLING);
  attachInterrupt(1, updateEncoder, CHANGE);  
  State = 0;
  fired = false;
  turned = false;
}

void firebutton(){
  if(!turned && !fired){
    fired = true;
    if (debug == 3) Serial.println("Button Fired!");
  }
}

void updateEncoder(){
  if(!turned && !fired){
    // Read the current state of CLK
    currentStateCLK = digitalRead(CLK);

    // If last and current state of CLK are different, then pulse occurred
    // React to only 1 state change to avoid double count
    if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
      // If the DT state is different than the CLK state then
      // the encoder is rotating CCW so decrement
      if (digitalRead(DT) != currentStateCLK) {
        rcounter --;
        up = false;
        if (debug == 3) Serial.println("Rotary Down");
      } else {
        // Encoder is rotating CW so increment
        rcounter ++;
        up = true;
        if (debug == 3) Serial.println("Rotary Up");
      }
      turned = true;
    }
  // Remember last CLK state
  lastStateCLK = currentStateCLK;
  }
}

void loop() {
  int currentdelay = defaultdelay;
  aktuell = rtc.now();
  if (State == 7) {
    tone(channel, freq);
  }
  ListenBLE();
  if(State == 0){
    int gassensorAnalog = analogRead(Gas_analog);
    int gassensorDigital = digitalRead(Gas_digital);
    if (debug == 2) {
      Serial.print("Gas Sensor: ");
      Serial.print(gassensorAnalog);
      Serial.print("\t");
      Serial.print("Gas Class: ");
      Serial.print(gassensorDigital);
      Serial.print("\t");
      Serial.print("\t");
      Serial.println("");
    }
    if (gassensorAnalog > GasThres)
    {
      tone(channel, 3500);
      delay(200);
      noTone(channel);
    }
  }
  if(State == 0 || State == 7){
    // UltraSonic
    // Clears the trigPin condition
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    if (debug == 4) Serial.println(String(duration));    
  }
  /*
     State 0 = Hauptbildschirm
     State 1 = Hauptmenü
     State 2 = Settings
     State 3 = About
     State 6 = Device infos
     State 7 = WaterArcustics
     State 9 = DATE/TIME Setup
  */

  if (fired) {
    if (State == 0 && fired == true) {
      State = 1; 
      fired = false;     
    }
    if (State == 1 && fired == true) {
      if (menupos == 0) {
        State = 0;
      }
      if (menupos == 12) {
        State = 6;
      }
      if (menupos == 24) {
        State = 2;
      }
      if (menupos == 36) {
        State = 3;
      }
      fired = false; 
    }
    else if (State == 2 && fired == true) {
      if (settingsmenupos == 0) {
        State = 1;
      }
      if (settingsmenupos == 12) {
        aktuell = rtc.now();
        hournow = aktuell.hour();
        minutenow = aktuell.minute();
        yearnow = aktuell.year();
        monthnow = aktuell.month();
        daynow = aktuell.day();
        timesetupstate = 0;
        State = 9;
      }  
      if (settingsmenupos == 24) {
        State = 7;
      }
      fired = false; 
    }
    else if (State == 3 && fired == true) {
      State = 1;
      fired = false; 
    }
    else if (State == 6 && fired == true) {
      State = 1;
      fired = false; 
    }
    else if (State == 7 && fired == true) {
      State = 2;
      noTone(channel);
      fired = false; 
    }
  }  // end if fired

  if (turned && State != 0)
  {
    if (State == 1) {
      if (up) {
        if (menupos < menulinehight * (menuitems - 1)) {
          menupos = menupos + menulinehight;
        }
      }
      else  {
        if (menupos >= menulinehight) {
          menupos = menupos - menulinehight;
        }
      }
    }
    else if (State == 2) {
      if (up) {
        if (settingsmenupos < settingsmenulinehight * (settingsmenuitems - 1)) {
          settingsmenupos = settingsmenupos + settingsmenulinehight;
        }
      }
      else  {
        if (settingsmenupos >= menulinehight) {
          settingsmenupos = settingsmenupos - settingsmenulinehight;
        }
      }
    }  
  }

  if (State == 0) {
    float Temperature;
    float Humidity;
    bool waterwarn = false;

    Temperature = dht11.readTemperature(); // Gets the values of the temperature
    Humidity = dht11.readHumidity(); // Gets the values of the humidity

    //Fuellstand berechnen
    levelact = (Tankheight - distance);
    //Fuellstand in Prozent berechnen
    WaterLevel = (levelact * 100 / Tankfull);
    if (duration == 0) {
      waterwarn = true;
      WaterLevel = 0;
    }
    if (WaterLevel < 0) WaterLevel = 0;
    if (WaterLevel > 100) WaterLevel = 100;
    //Berechnung Volumen
    volumenact = ((Volumen / 100) * WaterLevel);
    MainScreen(Temperature, Humidity, Gaslevel, WaterLevel, waterwarn);
    currentdelay = mainscreendelay;
  }

  else if (State == 1) {
    MainMenu();
  }

  else if (State == 2) {
    Settings();
  }

  else if (State == 3) {
    About();
  }

  else if (State == 6) {
    Info();
  }
  else if (State == 7) {
    levelact = (Tankheight - distance);
    WaterLevel = (levelact * 100 / Tankfull);
    currentdelay = WaterArcusticWarn(WaterLevel);
  }
 
  else if (State == 9) {
    String hourstring = String(hournow);
    String minutestring = String(minutenow);
    String daystring = String(daynow);
    String monthstring = String(monthnow);
    String yearstring = String(yearnow);
    int boxx;
    int boxy;
    int boxheight = 15;
    int boxwith = 19;  
    if (hournow < 10) hourstring = "0" + String(hournow);
    if (minutenow < 10) minutestring = "0" + String(minutenow);
    if (monthnow < 10) monthstring = "0" + String(monthnow);
    if (daynow < 10) daystring = "0" + String(daynow);
    String Datestring = daystring + "." + monthstring + "." + yearstring;
    String Timestring = hourstring + ":" + minutestring;
    if (timesetupstate == 0){boxx = 18;boxy=20;}
    if (timesetupstate == 1){boxx = 45;boxy=20;}
    if (timesetupstate == 2){boxx = 72;boxy=20; boxwith=38;}
    if (timesetupstate == 3){boxx = 41;boxy=35;}
    if (timesetupstate == 4){boxx = 68;boxy=35;}
    
    u8g2_prepare();
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_profont11_tf);
    u8g2.drawUTF8(ALIGN_CENTER("Zeit Einstellung"), 8, "Zeit Einstellung");
    u8g2.setFont(u8g2_font_t0_18_tf);
    u8g2.drawBox(boxx,boxy,boxwith,boxheight);
    u8g2.setDrawColor(2);
    u8g2.drawUTF8(ALIGN_CENTER(Datestring.c_str()), 20, Datestring.c_str());
    u8g2.drawUTF8(ALIGN_CENTER(Timestring.c_str()), 35, Timestring.c_str());
    u8g2.sendBuffer();
   if(timesetupstate == 0){
      if(turned){
        if(up){
          if(daynow == 31) daynow = 1;
          else daynow = daynow + 1;
        }    
        if(!up){
          if(daynow == 1) daynow = 31;
          else daynow = daynow - 1;
        }
      }
    }
       if(timesetupstate == 1){
      if(turned){
        if(up){
          if(monthnow == 12) monthnow = 1;
          else monthnow = monthnow + 1;
        }    
        if(!up){
          if(monthnow == 1) monthnow = 12;
          else monthnow = monthnow - 1;
        }
      }
    }
       if(timesetupstate == 2){
      if(turned){
        if(up){
          if(yearnow > 3000) yearnow = 3000;
          else yearnow = yearnow + 1;
        }    
        if(!up){
          if(yearnow < 2020) yearnow = 2020;
          else yearnow = yearnow - 1;
        }
      }
    }
    
    if(timesetupstate == 3){
      if(turned){
        if(up){
          if(hournow == 24) hournow = 0;
          else hournow = hournow + 1;
        }    
        if(!up){
          if(hournow == 0) hournow = 24;
          else hournow = hournow - 1;
        }
      }
    }
    if(timesetupstate == 4){
      if(turned){
        if(up){
          if(minutenow == 59) minutenow = 0;
          else minutenow = minutenow + 1;
        }    
        if(!up){
          if(minutenow == 0) minutenow = 59;
          else minutenow = minutenow - 1;
        }
      }
    }

    if(fired){
      if(timesetupstate == 4) {
        rtc.adjust(DateTime(yearnow, monthnow, daynow, hournow, minutenow, 30));
        State = 0;
      }
      else timesetupstate = timesetupstate + 1;
      fired = false;
    }
    currentdelay = 50;
  }
  turned = false;
  delay(currentdelay);  
}
