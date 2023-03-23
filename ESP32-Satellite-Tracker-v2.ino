    /* ====================================================================
   Copyright (c) 2019 Thorsten Godau (dl9sec). All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:
   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
   3. Neither the name of the author(s) nor the names of any contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.
   THIS SOFTWARE IS PROVIDED BY THE AUTHOR(S) AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR(S) OR CONTRIBUTORS BE LIABLE
   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
   OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
   SUCH DAMAGE.
   ====================================================================*/

/* If the SPIFFS wont load ("SPIFFS initialisation failed!") run this in a seperate sketch and try again:

#include "SPIFFS.h"

void setup() {
  SPIFFS.format();
}
void loop() {
}

*/
 
//
//https://github.com/sridel92/SatTracker_esp32_v2
//
// You will need this library:
//https://github.com/Hopperpop/Sgp4-Library
//
// Call up the SPIFFS FLASH filing system this is part of the ESP Core
#define FS_NO_GLOBALS
#include <FS.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

// for the BMP files
// So I used image magick with the following settings to get the proper color bit depth and compression:
// magick convert -depth 24 -compress none -background black -alpha remove image.bmp new-image.bmp
// https://imagemagick.org/script/download.php#macosx
////////////////////////////////////////////////////////

//#ifdef ESP32
#include "SPIFFS.h"  // For ESP32 only
//#endif

//#include <WiFi.h>
#include <HTTPClient.h>
#include "time.h"
#include <TimeLib.h>
#include <WiFiUdp.h>
#include "settings.h"                               // Important: see settings.h to configure your settings!!!
//#include <../TFT_eSPI_Setups/User_Setup_st7789.h>    // this is usefull for opening a custom library setup
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_SPIFlash.h>    // SPI / QSPI flash library
#include <Adafruit_ImageReader.h> // Image-reading functions
time_t getNtpTime();
#include <Sgp4.h>
#include <Ticker.h>   //https://github.com/sstaub/Ticker

Sgp4 sat;
Ticker tkSecond;  // this lib create some task at schedule time (second...)

#include "Free_Fonts.h"       // Include the header file attached to this sketch
//#include <TFT_eSPI.h>         // Hardware-specific library
//TFT_eSPI tft = TFT_eSPI();    // Invoke custom library


// Replace with your Wi-Fi network credentials
const char* ssid = "XXXXXXXX";
const char* password = "XXXXXXXXXXXXXX";


Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

int year_;
int mon_; 
int day_; 
int hr_; 
int min_; 
double sec_;
long        count    = 0;           // Loop count
double      dSatLAT  = 0;           // Satellite latitude
double      dSatLON  = 0;           // Satellite longitude
double      dSatAZ   = 0;           // Satellite azimuth
double      dSatEL   = 0;           // Satellite elevation
double      dSunLAT  = 0;           // Sun latitude
double      dSunLON  = 0;           // Sun longitude
double      dSunAZ   = 0;           // Sun azimuth
double      dSunEL   = 0;           // Sun elevation
char        acBuffer[20];           // Buffer for ASCII time
int         xpos      = 180;        // Position of watch
int         xposInit  = 0;
//int         ypos      = 120;        // Position of watch _Bottom right
int         ypos      = 5;          // Position of watch _Top right
int         ysecs     = ypos;
int         xyfont    = 2;
//int         xd        = 15;
//int         yd        = 190;
int         xd        = 0;
int         yd        = 0;
int         IssLat    = 0;
int         IssLon    = 0;
byte        xcolon    = 0, xsecs = 0;
uint32_t    targetTime = 0;                 // for next 1 second timeout
unsigned long unixtime = 1617906546;
//int timezone = 12 ;                       //utc + 12
int framerate;
static uint8_t conv2d(const char* p);       // Forward declaration needed for IDE 1.6.x

int ledState = LOW;               // ledState used to set the LED
  const int LOOP_BUTTON   = 13;   // at startup, bring the GPIO13 at ground, tracker is run on LOOP mode
  const int UP_Switch     = 12;   // at startup, bring the GPIO12 at groung, the screen is rotate by 180 degres
  const int PIN_LED       =  2;   // the GPIO2 is now disconnected from the screen (modify setup.h) The GPIO2 led on the board can alert during Europe pass

int loop_button_state = 0;
int button_state      = 0;
bool UPState          = true;
bool UP               = true;
bool TIMELOOP         = false;
bool Running          = false;

unsigned int colour = 0;
const long Blink    = 1000;                 // time to refresh screen (milliseconds)
unsigned long RefreshPreviousMillis = 0;    // will store last time screen was was refreshed
unsigned long BlinkPreviousMillis   = 0;    // will store last time screen was was refreshed
unsigned long previousMillis        = 0;        
unsigned long interval              = 1000;

// *************************************************************
// celestrak.org.		1638	IN	A	104.168.149.178
String CelestrakUrl    = "http://celestrak.org";      //Web address to get TLE (CELESTRAK)
char TLENameChar[][21] = { "sat0", "sat1", "sat2", "sat3", "sat3", "sat3" };
char TLE1Char[][71] =    { "sat0", "sat1", "sat2", "sat3", "sat3", "sat3" };
char TLE2Char[][71] =    { "sat0", "sat1", "sat2", "sat3", "sat3", "sat3" };

//int Sat = 4;        //  Enter the number of tracked satellite
int Sat = 6;        //  Enter the number of tracked satellite
// Enter the code of the tracked satellites from Celestrak
char *SatTleURL[] = {
  "/NORAD/elements/gp.php?CATNR=25544&FORMAT=TLE",
  "/NORAD/elements/gp.php?CATNR=31135&FORMAT=TLE",
  "/NORAD/elements/gp.php?CATNR=33591&FORMAT=TLE",
  "/NORAD/elements/gp.php?CATNR=43013&FORMAT=TLE",
  "/NORAD/elements/gp.php?CATNR=11060&FORMAT=TLE",
  "/NORAD/elements/gp.php?CATNR=37846&FORMAT=TLE",
  "/NORAD/elements/gp.php?CATNR=48274&FORMAT=TLE",
  /*
    "/satcat/tle.php?CATNR=25544",   
    "/satcat/tle.php?CATNR=23455",
    "/satcat/tle.php?CATNR=33591",
    "/satcat/tle.php?CATNR=43013",
    "/satcat/tle.php?CATNR=11060",
    "/satcat/tle.php?CATNR=42982",
  */
    };
/*
 * 25544 : ISS          -BLUE
 * 31135 : AGILE        -RED
 * 33591 : NOAA 19      -YELLOW
 * 43013 : NOAA 20      -ORANGE
 * 11060 : TIROS N      -WHITE
 * 37846 : Galileo      -MAGENTA
 * 48274 : TIANHE       -CYAN
 */


HTTPClient client;
String HourStr[10];

const char* ntpServer = "pool.ntp.org";   // NTP server to request epoch time
unsigned long epochTime;                  // Variable to save current epoch time
unsigned long getTime() {                 // Function that gets current epoch time
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
  Ticker tkSecond;  // this lib create some task at schedule time (second...)
}

/* You only need to format SPIFFS the first time you run a
   test or else use the SPIFFS plugin to create a partition
   https://github.com/me-no-dev/arduino-esp32fs-plugin */
#define FORMAT_SPIFFS_IF_FAILED true

  // Needed to display bmp image on Adafruit ESP32 Feather TFT
  //
  // SPI or QSPI flash filesystem (i.e. CIRCUITPY drive)
  #if defined(__SAMD51__) || defined(NRF52840_XXAA)
    Adafruit_FlashTransport_QSPI flashTransport(PIN_QSPI_SCK, PIN_QSPI_CS,
      PIN_QSPI_IO0, PIN_QSPI_IO1, PIN_QSPI_IO2, PIN_QSPI_IO3);
  #else
//    #if (SPI_INTERFACES_COUNT == 1)
      Adafruit_FlashTransport_SPI flashTransport(SS, &SPI);
//    #else
//      Adafruit_FlashTransport_SPI flashTransport(SS1, &SPI1);
//    #endif
  #endif
  Adafruit_SPIFlash    flash(&flashTransport);
  FatFileSystem        filesys;
  Adafruit_ImageReader reader(filesys); // Image-reader, pass in flash filesys
/*
  // Flash system error handling
  if(!flash.begin()) {
    Serial.println(F("flash begin() failed"));
    for(;;);
  }
  if(!filesys.begin(&flash)) {
    Serial.println(F("filesys begin() failed"));
    for(;;);
  }
*/

//void Second_Tick();
void Second_Tick() {
  unixtime += 1;      
  invjday(sat.satJd , timeZone,true, year_, mon_, day_, hr_, min_, sec_);
  #ifdef DEBUG
  Serial.println(String(day_) + '/' + String(mon_) + '/' + String(year_) + ' ' + String(hr_) + ':' + String(min_) + ':' + String(sec_));
  Serial.println("azimuth = " + String( sat.satAz) + " elevation = " + String(sat.satEl) + " distance = " + String(sat.satDist));
  Serial.println("latitude = " + String( sat.satLat) + " longitude = " + String( sat.satLon) + " altitude = " + String( sat.satAlt));
  switch(sat.satVis){
    case -2:
      Serial.println("Visible : Under horizon");
      //tft.setFreeFont(FF1);                 // Select the font
      //tft.drawString("Visible : Under horizon", 150, 210, 2);
    break;
    case -1:
      Serial.println("Visible : Daylight");
      //tft.setFreeFont(FF1);                 // Select the font
      //tft.drawString("Visible : Daylight", 150, 210, 2);
      break;
    default:
      Serial.println("Visible : " + String(sat.satVis));   //0:eclipsed - 1000:visible
      //tft.setFreeFont(FF1);                                // Select the font
      //tft.drawString("*** Visible ***", 200, 230, 2);
      //tft.drawFloat(sat.satVis, 190, 210, 2);
      break;
  }

  Serial.println("Framerate: " + String(framerate) + " calc/sec");
  Serial.println();
  #endif
  framerate=0;
}


void setup()
{
  //WiFi.mode(WIFI_STA);         // explicitly set mode, esp defaults to STA+AP
  Serial.begin(115200);
  

  WiFiManager wm;

    // Up or Down ?
    // const int UP_SwItch = 36;
    // int UpState           = 0;
    // bool UP               = false;
  pinMode(UP_Switch, INPUT_PULLUP);    
  UPState = digitalRead(UP_Switch);

  if (UPState == HIGH) {
      UP = true;
    } else {
     UP = false;
    }

/*
    tft.begin();
   if ( UP == true ) {
      tft.setRotation(1);                 // 1 landscape - 3 reverse landscape  
      } else {
      tft.setRotation(3);
      }
*/

  // turn on backlight
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  
  tft.init(135, 240);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.println("  Satellite");
  tft.println("   Tracker");
  tft.println("");
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("Connecting to WiFi");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    tft.print(".");
  }
  tft.println(WiFi.localIP());
  delay(1500);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0,0);
/*
      tft.drawString("Sat Tracker portal", 20, 80, FONT2);
      tft.drawString("Use mobile or laptop", 20, 100, FONT2);
      tft.drawString("search for wifi lan 'Esp32_SatPortal' ", 20, 120, FONT2);
      tft.drawString("-",   20, 140, FONT2);
      tft.drawString("--",  20, 150, FONT2);
      tft.drawString("---", 20, 160, FONT2);
*/

/*
tft.setTextColor(ST77XX_GREEN);
tft.setTextSize(2);
tft.setCursor(0, 0);
tft.println("Sat Tracker portal");
tft.println("");
//tft.setCursor(20, 100);
tft.setTextColor(ST77XX_WHITE);
tft.println("Use mobile or laptop");

//tft.setCursor(20, 120);
tft.println("search for wifi lan 'Esp32_SatPortal'");
  delay(5000);

tft.setTextColor(ST77XX_YELLOW);
tft.setTextSize(3);

*/
/*
tft.drawChar(20, 140, '-');
tft.drawChar(35, 140, '-');
tft.drawChar(50, 140, '-');
*/

/*
tft.setCursor(20, 140);
tft.print("-");
tft.setCursor(35, 140);
tft.print("-");
tft.setCursor(50, 140);
tft.print("-");

tft.setTextSize(2);
tft.setCursor(20, 150);
tft.print("--");

tft.setTextSize(3);
tft.setCursor(20, 160);
tft.print("---");
*/
    //reset settings - wipe credentials for testing
    //wm.resetSettings();     // uncomment for reset settings - wipe credentials for testing

    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result

  //wm.autoConnect();
  bool res;
    res = wm.autoConnect();                          // auto generated AP name from chipid
    res = wm.autoConnect("Esp32_SatPortal");            // anonymous ap
    res = wm.autoConnect("Esp32_SatPortal","password"); // password protected ap

    if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
        /*
        tft.drawString("--", 20, 150, FONT2);
        tft.drawString("---> connected to residential Wifi", 20, 160, FONT2);
        */
        tft.setTextColor(ST77XX_WHITE); // Set text color
        tft.setTextSize(2); // Set text size

        // Draw "--" character by character
        /*
        tft.drawChar(20, 150, '-', ST77XX_WHITE, ST77XX_BLACK, 2);
        tft.drawChar(32, 150, '-', ST77XX_WHITE, ST77XX_BLACK, 2);
        */
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setTextSize(2);
        tft.setCursor(20, 150);
        tft.print('-');
        tft.setCursor(32, 150);
        tft.print('-');


       // Draw "---> connected to residential Wifi" string character by character
        char text[] = "---> connected to residential Wifi";
        int x = 20, y = 160;
        for (int i = 0; i < strlen(text); i++) {
 //         tft.drawChar(x, y, text[i], ST77XX_WHITE, ST77XX_BLACK, 2);
            tft.setCursor(x, y);
            tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
            tft.setTextSize(2);
            tft.print(text[i]);
 
          x += 6; // Increment x coordinate for next character
}

      //  delay(1000);
        tft.fillScreen(ST77XX_BLACK);
    }

  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS initialisation failed!");
    while (1) yield();        // Stay here twiddling thumbs waiting
  }
  Serial.println("\r\nSPIFFS initialized.");

  // loop or not ?
  pinMode(LOOP_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  loop_button_state = digitalRead(LOOP_BUTTON);


if (loop_button_state == LOW) {
    TIMELOOP = true;
    digitalWrite(PIN_LED, HIGH);
//    tft.drawString("TIME warp...", 10, 220, FONT2);
   tft.setTextColor(ST77XX_WHITE); // Set text color to white
    tft.setTextSize(2); // Set text size to 2
    int xPos = 10; // Starting x position for text
    //int yPos = 220; // Starting y position for text
    int yPos = 100; 
    String text = "TIME warp..."; // Text to draw

// Draw each character individually
for (int i = 0; i < text.length(); i++) {
//  tft.drawChar(xPos, yPos, text[i], ST77XX_WHITE, ST77XX_BLACK, 2);
    tft.setCursor(xPos, yPos);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setTextSize(2);
    tft.print(text[i]);

  xPos += 12; // Move x position to the right for the next character
}
       
  } else {
 TIMELOOP = false;
// tft.drawString("TIME normal...", 10, 220, FONT2);
tft.setTextColor(ST77XX_WHITE);
tft.setTextSize(2);

char text[] = "TIME normal...";
//int x = 10, y = 220;
int x = 0, y = 0;
uint16_t textWidth = 0;
int16_t x1, y1;
uint16_t w, h;
tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
textWidth = w; // Get the width of the text
for (int i = 0; text[i] != '\0'; i++) {
  // tft.drawChar(x, y, text[i], FONT2);
  tft.setCursor(x, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print(text[i]);
  x += textWidth * 2 / strlen(text); // Calculate the spacing between characters

}


  }


for (int i=0; i<21; i++) {
  digitalWrite(PIN_LED, Running);
  Running = !Running;
  delay(50);
  }


//  Slapsh screen
  //drawBmp("/ISS_20years.bmp", 0, 0);  // 320x106
  delay(500);
 // tft.setFont(TT1);               // Select the orginal small TomThumb font
//  tft.drawString("Satellite tracker", 20, 120, FONT2);
    tft.setTextSize(2); // set text size to 2

//    String text = "Satellite tracker";
    tft.fillScreen(ST77XX_BLACK);
    String text = "Engage TARDIS";
    //int16_t x = 20; // starting x position
    //int16_t y = 120; // starting y position
    int16_t x = 0; // starting x position
    int16_t y = 0; // starting y position
//    uint16_t textWidth = tft.getStringWidth(text); // get the width of the text
    uint8_t textSize = 2; 
    uint16_t textWidth = text.length() * textSize * 6; // 6 is an approximate width of each character


// draw the text character by character
for (int i = 0; i < text.length(); i++) {
//  tft.drawChar(x + (i * (textWidth / text.length())), y, text.charAt(i), ST77XX_WHITE, ST77XX_BLACK, 2);
    tft.setCursor(x + (i * (textWidth / text.length())), y);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.print(text.charAt(i));

}

  delay(1000);

/*
  tft.fillRect(0,107, 320,240, ST77XX_BLACK);
  delay(500);
  tft.fillRect(0,0, 320,106, ST77XX_WHITE);
  delay(500);
*/
  //drawBmp("/Ntp.bmp", 20, 10); // 100x72 
//  tft.drawString("I'm getting time from NTP server...", 10, 130, FONT2);
    tft.setTextColor(ST77XX_WHITE); // set text color to white
    tft.setTextSize(2); // set text size to 2
    tft.println("");
    tft.println("");
    //String text = "I'm getting time from NTP server...";
    tft.println("Getting time from");
    tft.setTextColor(ST77XX_BLUE);
    tft.print("NTP ");
    tft.setTextColor(ST77XX_WHITE);
    tft.println("server...");
    delay(2000);
    tft.fillScreen(ST77XX_BLACK);


    int textLength = text.length();
    for (int i = 0; i < textLength; i++) {
//      tft.drawChar(10 + (i * 12), 130, text.charAt(i), FONT2);
        tft.setCursor(0, 0);
        tft.setTextSize(2);
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        for (int i = 0; i < text.length(); i++) {
//  tft.print(text.charAt(i));
}

}
  configTime(0, 0, ntpServer);
  epochTime = getTime();
//  tft.print(epochTime);
  //tft.drawNumber(epochTime, 10, 170, FONT4);  //ok
String epochTimeStr = String(epochTime); // convert the epoch time to a string
//int16_t xa = 10; // x coordinate of the starting position
//int16_t ya = 170; // y coordinate of the starting position
int16_t xa = 10; // x coordinate of the starting position
int16_t ya = 70; // y coordinate of the starting position
uint8_t font_size = FONT4; // font size to use

// iterate through each character in the string and print it to the display
for (int i = 0; i < epochTimeStr.length(); i++) {
  char c = epochTimeStr.charAt(i);
 // tft.drawChar(x + (i * (font_size * 6)), y, c, WHITE, BLACK, font_size);

 tft.setCursor(xa + (i * (font_size * 6)), ya);
tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
tft.setTextSize(font_size);
tft.print(c);

}


  delay(1000);
  //tft.fillRect(0,107, 320,240, ST77XX_BLACK);
  tft.fillRect(0,107, 240,135, ST77XX_BLACK);
  //delay(500);
  //drawBmp("/Celestrak.bmp", 0, 0);
  // Examplke for a tft image
  // tft.drawRGBBitmap(0, 0, ISS_20years_bmp, BMP_WIDTH, BMP_HEIGHT);
  //delay(200);
  //tft.drawString("Connecting to Celestrak", 20, 120, FONT2);
  //delay(200);
  //tft.drawString("Loocking for the lastest TLE", 20, 135, FONT2);
  tft.setTextColor(ST77XX_RED); // Set text color to white
  tft.setTextSize(2); // Set text size to 2
  tft.setCursor(0, 0);

  String message1 = "Connecting to       Celestrak/NORAD...";
  tft.setTextColor(ST77XX_WHITE);
  String message2 = "Looking for the     latest TLE...";

  for (int i = 0; i < message1.length(); i++) { 
//    tft.drawChar(20 + i*6, 120, message1.charAt(i), ST77XX_WHITE, ST77XX_BLACK, 2);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setTextSize(2);
    //tft.setCursor(20, 120);
    tft.setCursor(0, 0);
    for(int i=0; i<message1.length(); i++){
      tft.print(message1.charAt(i));
      //tft.setCursor(tft.getCursorX() + 6, tft.getCursorY());
}
tft.setTextColor(ST77XX_BLACK);

    delay(50);
}

delay(200);

for (int i = 0; i < message2.length(); i++) {
//  tft.drawChar(20 + i*6, 135, message2.charAt(i), ST77XX_WHITE, ST77XX_BLACK, 2);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 40);
//    for(int i=0; i<message1.length(); i++){
    for(int i=0; i<message2.length(); i++){

      tft.print(message2.charAt(i));
//      tft.setCursor(tft.getCursorX() + 6, tft.getCursorY());
}
tft.setTextColor(ST77XX_BLACK);
  delay(50);
}


// Here we are going the download the TLE definition of each Satellite
for (int i = 0; i < Sat; i++) {
    #ifdef DEBUG
    Serial.println("");
    Serial.print("Main : Satellite number ");
    Serial.println(i);
    Serial.println("");
    Serial.println("Main ----------> go to GetTLE <------------------");
    #endif
    GetTLE(i);
  }

  //tft.drawString("TLE1 & 2 uplaoded...", 160, 210, FONT2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // set text color and background color
  tft.setTextSize(2); // set text size
  tft.setCursor(20, 100); // set cursor position
  tft.print("T"); // draw 'T' character
  tft.print("L"); // draw 'L' character
  tft.print("E"); // draw 'E' character
  tft.print("1"); // draw '1' character
  tft.print(" "); // draw space character
  tft.print("&"); // draw '&' character
  tft.print(" "); // draw space character
  tft.print("2"); // draw '2' character
  tft.print(" "); // draw space character
  tft.print("uploaded"); // draw 'uploaded' string

  delay(500);
  //tft.fillRect(0,0, 320,120, ST77XX_WHITE);
  tft.fillRect(0,0, 240,135, ST77XX_WHITE);
  delay(100);
  //tft.fillRect(0,121, 320,240, ST77XX_BLACK);
  tft.fillRect(0,0, 240,135, ST77XX_WHITE);
  /*
  delay(100);
  drawBmp("/Nasa.bmp", 0, 122);     // 120x102
  delay(100);
  drawBmp("/Esa.bmp", 130, 125);    // 132x60
  delay(100);
  drawBmp("/CsaAsc.bmp", 0, 5);     // 110x106
  delay(100);
  drawBmp("/Jaxa.bmp", 20, 10);     // 120x77
  delay(100);
  drawBmp("/MissionAlpha.bmp", 160, 0);
  delay(1000);
  drawBmp("/Crew2.bmp", 0, 0);
  delay(1000);
*/
  int i = 0;

  sat.site(dMyLAT, dMyLON  ,dMyALT); //set site latitude[°], longitude[°] and altitude[m]
  
  //Display TLE epoch time
  double jdC = sat.satrec.jdsatepoch;
  invjday(jdC , timeZone, true, year_, mon_, day_, hr_, min_, sec_);
 
  tkSecond.attach(1,Second_Tick);  
  //targetTime = millis() + 1000;
  tft.fillScreen(ST77XX_BLACK);
  epochTime = 1619830800;


} // EndSetup

//*** Copied from BMP_functions.ino and then modified ***

/*
void drawBmp(const char *filename, int16_t x, int16_t y) {

  const uint16_t screenWidth = 240;
  const uint16_t screenHeight = 135;

  if ((x >= screenWidth) || (y >= screenHeight)) return;

  //fs::File bmpFS;
  // I don't have an SD card
  fs::File bmpFS = SPIFFS.open(filename, "r");
  // Open requested file on SD card
  //bmpFS = SPIFFS.open(filename, "r");

  if (!bmpFS)
  {
    Serial.print("File not found");
    return;
  }
  uint32_t seekOffset;
  uint16_t w, h, row, col;
  uint8_t  r, g, b;
  uint32_t startTime = millis();

  if (read16(bmpFS) == 0x4D42)
  {
    read32(bmpFS);
    read32(bmpFS);
    seekOffset = read32(bmpFS);
    read32(bmpFS);
    w = read32(bmpFS);
    h = read32(bmpFS);

    if ((read16(bmpFS) == 1) && (read16(bmpFS) == 24) && (read32(bmpFS) == 0))
    {
      y += h - 1;

      bool swapBytes = true; // Set to true or false based on your hardware

      bmpFS.seek(seekOffset);

      uint16_t padding = (4 - ((w * 3) & 3)) & 3;
      uint8_t lineBuffer[w * 3 + padding];

      for (row = 0; row < h; row++) {
        
        bmpFS.read(lineBuffer, sizeof(lineBuffer));
        uint8_t*  bptr = lineBuffer;
        uint16_t* tptr = (uint16_t*)lineBuffer;
        // Convert 24 to 16 bit colours
        for (uint16_t col = 0; col < w; col++)
        {
          b = *bptr++;
          g = *bptr++;
          r = *bptr++;
          uint16_t color = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
          if (swapBytes) {
            color = (color >> 8) | (color << 8);
          }
          tft.drawPixel(x + col, y, color);
        }
        y--; // y is decremented as the BMP image is drawn bottom up
      }
      // Serial.print("Loaded in "); Serial.print(millis() - startTime);
      // Serial.println(" ms");
    }
    else Serial.println("BMP format not recognized.");
  }
  bmpFS.close();
}



// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(fs::File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(fs::File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

*/
// *************************

// Copied from GetTLE.ino
//
void GetTLE(int16_t i) {
//
// Connecting to Cletrak for getting TwoLine element (TLE)
// HTTP request
String SatUrl = String(CelestrakUrl) + String(SatTleURL[i]);  // this is the complete url search for this "Sat"
Serial.println(SatUrl); 

HTTPClient http;
http.begin(SatUrl);
int httpCode = http.GET();

if (httpCode > 0) { //Check for the returning code
  String payload = http.getString();
  String TLEName = payload.substring(0,20);
  String TLE1 =    payload.substring(26,96);
  String TLE2 =    payload.substring(97,167);
  
  Serial.println("Payload from Celestrak");
  Serial.println(TLEName);
  Serial.println(TLE1); 
  Serial.println(TLE2);
  Serial.println("end of string...");

  Serial.println("-----------------------------------------------");

  TLEName.toCharArray(TLENameChar[i],21); // récupère le param dans le tableau de char  
  TLE1.toCharArray(TLE1Char[i],71); // récupère le param dans le tableau de char
  TLE2.toCharArray(TLE2Char[i],71); // récupère le param dans le tableau de char

#ifdef DEBUG
  Serial.print("TLENameChar = ");
  Serial.println(TLENameChar[i]); // affiche le tableau de char
  Serial.print("TLE1Char = ");
  Serial.println(TLE1Char[i]); // affiche le tableau de char
  Serial.print("TLE1Char = ");
  Serial.println(TLE2Char[i]); // affiche le tableau de char
  Serial.println("----------------------------------------------");
#endif

    //tft.drawString(TLENameChar[i], 20, 150+(i*14), FONT2);
    tft.setCursor(20, 150+(i*14));
    //tft.setFont(&FONT2);
    tft.print(TLENameChar[i]);


  }else {
      Serial.println("Error on HTTP request");
  }

http.end(); //Free the resources
}

// From drawMarkerColor.ino

// *******************************************************************
// Draw a + mark centered on x,y
/*
void drawMarker(int x, int y)
{
  //tft.drawLine(x - 4, y, x + 4, y, ST77XX_BLUE);
  //tft.drawLine(x, y - 4, x, y + 4, ST77XX_BLUE);
  tft.drawCircle(x, y, 1, ST77XX_BLUE);
}
*/
void drawMarker(int x, int y)
{
  // Draw vertical line
  tft.drawFastVLine(x, y - 5, 11, ST77XX_BLUE);
  
  // Draw horizontal line
  tft.drawFastHLine(x - 5, y, 11, ST77XX_BLUE);

    float xx = ( ( sat.satLon + 180 ) *  240 ) / 360 ;    // Longitude is 360 wide but screen 240 pixels
    float yy = 0;
    int xTft = xx;
    int yTft = (90 - sat.satLat);
  tft.drawCircle(xTft, yTft, 3, ST77XX_BLUE);
}
// *******************************************************************
// Draw a + mark centred on x,y
// i is the sat number and set the color of the mark
//

void drawMarkerColor(int x, int y, int i)
{
 
  switch (i) {
    case 0:
      tft.drawLine(x - 2, y, x + 2, y, ST77XX_BLUE);
      tft.drawLine(x, y - 1, x, y + 1, ST77XX_BLUE);
      tft.fillRect(xd, yd, 25, 4, ST77XX_BLUE);
      tft.setCursor(xd+30, yd);
      tft.print(TLENameChar[i]);
      //tft.setCursor(xd+90, yd);
      //tft.print(x);
      //tft.setCursor(xd+110, yd);
      //tft.print(y);
      break;

    case 1:
      tft.drawLine(x-1, y, x+1, y, ST77XX_RED);
      tft.drawLine(x, y-1, x, y+1, ST77XX_RED);
      tft.fillRect(xd, yd+15, 25, 4, ST77XX_RED);
      tft.setCursor(xd+30, yd+15);
      tft.print(TLENameChar[i]);
      //tft.setCursor(xd+90, yd+15);
      //tft.print(x);
      //tft.setCursor(xd+110, yd+15);
      //tft.print(y);
      break;

    case 2:
      tft.drawLine(x, y, x, y, ST77XX_YELLOW);
      tft.drawLine(x, y, x, y, ST77XX_YELLOW);
      tft.fillRect(xd, yd+30, 25, 4, ST77XX_YELLOW);
      tft.setCursor(xd+30, yd+30);
      tft.print(TLENameChar[i]);
      //tft.setCursor(xd+90, yd+30);
      //tft.print(x);
      //tft.setCursor(xd+110, yd+30);
      //tft.print(y);
      break;

    case 3:
      tft.drawLine(x, y, x, y, ST77XX_ORANGE);
      tft.drawLine(x, y, x, y, ST77XX_ORANGE);
      tft.fillRect(xd, yd+45, 25, 4, ST77XX_ORANGE);
      tft.setCursor(xd+30, yd+45);
      tft.print(TLENameChar[i]);
      //tft.setCursor(xd+90, yd+45);
      //tft.print(x);
      //tft.setCursor(xd+110, yd+45);
      //tft.print(y);
      break;

    case 4:
      tft.drawLine(x, y, x, y, ST77XX_WHITE);
      tft.drawLine(x, y, x, y, ST77XX_WHITE);
      tft.fillRect(xd, yd+45, 25, 4, ST77XX_WHITE);
      tft.setCursor(xd+30, yd+45);
      tft.print(TLENameChar[i]);
      //tft.setCursor(xd+90, yd+45);
      //tft.print(x);
      //tft.setCursor(xd+110, yd+45);
      //tft.print(y);
      break;

    case 5:
      tft.drawLine(x, y, x, y, ST77XX_MAGENTA);
      tft.drawLine(x, y, x, y, ST77XX_MAGENTA);
      tft.fillRect(xd, yd+45, 25, 4, ST77XX_MAGENTA);
      tft.setCursor(xd+30, yd+45);
      tft.print(TLENameChar[i]);
      //tft.setCursor(xd+90, yd+45);
      //tft.print(x);
      //tft.setCursor(xd+110, yd+45);
      //tft.print(y);
      break;

    case 6:
      tft.drawLine(x, y, x, y, ST77XX_CYAN);
      tft.drawLine(x, y, x, y, ST77XX_CYAN);
      tft.fillRect(xd, yd+45, 25, 4, ST77XX_CYAN);
      tft.setCursor(xd+30, yd+45);
      tft.print(TLENameChar[i]);
      //tft.setCursor(xd+90, yd+45);
      //tft.print(x);
      //tft.setCursor(xd+110, yd+45);
      //tft.print(y);
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }
}


// *******************************************************************
void DrawMap(){
  //
  //Adafruit load image into RAM- Faster!
  //Adafruit_Image img;
  //ImageReturnCode stat;
  //stat = reader.loadBMP("/new-globe_map.bmp", img);
  //img.draw(tft, 0, 0);
  //
  //Other method to load image Adafruit
  //ImageReturnCode stat;
  //stat = reader.drawBMP("/image.h", tft, 0, 0);
  // Print status of image load to Serial console for troubleshooting
  //reader.printStatus(stat);

//This calls image.h - Removed  
  //tft.drawRGBBitmap(0, 0, (uint16_t *)image, 240, 135);
  //tft.pushImage( 0, 0, 240, 135, (uint16_t *)image.pixel_data);

    //tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_GREEN);
    tft.setTextSize(2);
    tft.setCursor(90, 0);
    tft.print("World");

  //int xMyPos = 160+dMyLON; //
  //int yMyPos = 90-dMyLAT; //
  //
  int xMyPos = 60+dMyLON; //
  int yMyPos = 90-dMyLAT; //
    tft.drawLine(xMyPos - 15, yMyPos, xMyPos + 15, yMyPos, ST77XX_YELLOW);
    tft.drawLine(xMyPos, yMyPos - 15, xMyPos, yMyPos + 15, ST77XX_YELLOW);
    //tft.drawRect(180-30, 30, 50, 45, ST77XX_YELLOW);
    //tft.drawRect(140-30, 30, 50, 45, ST77XX_YELLOW); //Boston, MA

  // Draw 2 lines for lat 0 longitude 0 (Red lines in center of screen)

  tft.drawCircle(xMyPos, yMyPos, 2, ST77XX_YELLOW);
  tft.drawCircle(xMyPos, yMyPos, 6, ST77XX_YELLOW);
  tft.drawCircle(xMyPos, yMyPos, 8, ST77XX_YELLOW);
  
  /*
The coordinates of longitude 0 and latitude 0 intersect at a point in the Atlantic Ocean off the west coast of Africa. 
This point is known as the Prime Meridian and is the designated starting point for measuring longitude. 
Latitude 0 is also known as the equator, which is an imaginary line that circles the Earth at 0 degrees latitude, 
dividing it into the Northern and Southern Hemispheres. The Prime Meridian and the equator intersect at a point in the 
Gulf of Guinea, about 380 miles (610 km) south of Tema, Ghana.
*/
  //tft.drawLine(160, 0, 160, 180, ST77XX_RED);
  //tft.drawLine(0, 90, 320, 90, ST77XX_RED);
// For the Adafruit TFT
tft.drawLine(0, 67, 239, 67, ST77XX_BLUE); // horizontal line
tft.drawLine(120, 0, 120, 134, ST77XX_BLUE); // vertical line


//////////////   SATELLITE COLOR AND NAME AT BOTTOM OF SCREEN  ////////////
// Print color line and satellite name- Ben
      //tft.fillRect(0, yd, 25, 3, ST77XX_BLUE);            // fillRect(0, 0, _width, _height, color);
      //tft.drawString(TLENameChar[i], 30, yd);

int i = 0;
tft.fillRect(0, 105, 25, 5, ST77XX_BLUE);            // fillRect(x, y, _width, _height, color);
tft.setCursor(30, 105);
tft.setTextColor(ST77XX_WHITE);
tft.setTextSize(1);
tft.print(TLENameChar[i]);

int ii = 1;
//tft.fillRect(0, 115, 25, 5, ST77XX_ORANGE);           // fillRect(x, y, _width, _height, color);
tft.fillRect(0, 115, 25, 5, ST77XX_RED);
tft.setCursor(30, 115);
tft.setTextColor(ST77XX_WHITE);
tft.setTextSize(1);
tft.print(TLENameChar[ii]);

int iii = 2;
//tft.fillRect(0, 125, 25, 5, ST77XX_CYAN);            // fillRect(x, y, _width, _height, color);
tft.fillRect(0, 125, 25, 5, ST77XX_YELLOW);
tft.setCursor(30, 125);
tft.setTextColor(ST77XX_WHITE);
tft.setTextSize(1);
tft.print(TLENameChar[iii]);

int iiii = 3;
tft.fillRect(125, 105, 25, 5, ST77XX_ORANGE);            // fillRect(x, y, _width, _height, color);
tft.setCursor(155, 105);
tft.setTextColor(ST77XX_WHITE);
tft.setTextSize(1);
tft.print(TLENameChar[iiii]);

int iiiii = 4;
tft.fillRect(125, 115, 25, 5, ST77XX_WHITE);            // fillRect(x, y, _width, _height, color);
tft.setCursor(155, 115);
tft.setTextColor(ST77XX_WHITE);
tft.setTextSize(1);
tft.print(TLENameChar[iiiii]);

int iiiiii = 5;
tft.fillRect(125, 125, 25, 5, ST77XX_MAGENTA);            // fillRect(x, y, _width, _height, color);
tft.setCursor(155, 125);
tft.setTextColor(ST77XX_WHITE);
tft.setTextSize(1);
tft.print(TLENameChar[iiiiii]);


}


// *******************************************************************
void DrawMapEurope() {
  //drawBmp("/MapEuropeNew.bmp", 0, 0);
    //tft.fillRect(300,0, 240,135, ST77XX_BLACK);
    tft.fillRect(0,0, 240,135, ST77XX_BLACK);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(90, 120);
    tft.print("Europe");
  //
  // Europe croping is
  // startin at lat 75° to lat 30°
  // ending at lon : -15 to lon 45°
  // 
  // screen is 300 x 225
  //   
  // drawCircle(int32_t x0, int32_t y0, int32_t r, uint32_t color),
// double  dMyLAT  =  48.91052;  // Latitude  - Asnières (France): N -> +, S -> -
// double  dMyLON  =   2.29100;  // Longitude - Asnières (France): E -> +, W -> -
//dMyLAT  =  48.91052;
//dMyLON  =   2.29100;
dMyLAT  =  42.361145;   //Boston, MA
dMyLON  =   -71.057083; //Boston, MA
//
//int xMyPos = ( 2.29100 + 15 ) * 5; // LON
//int yMyPos = ( 60 - 48.91052 ) * 5; // LAT
int xMyPos = ( -71.057083 + 15 ) * 5; // LON // Boston
int yMyPos = ( 60 - 42.361145 ) * 5; // LAT // Boston
//
 // Boston, MA  LAT 42.361145, LON  -71.057083 
 //
 // int xMyPos = ( -71.057083 + 15 ) * 5; //LON
 // int yMyPos = ( 60 - 42.361145 ) * 5; //LAT
 
//  tft.drawCircle(xMyPos, yMyPos, 2, ST77XX_YELLOW);
//  tft.drawCircle(xMyPos, yMyPos, 6, ST77XX_YELLOW);
//  tft.drawCircle(xMyPos, yMyPos, 8, ST77XX_YELLOW);
  

//  //test pointing some known aera on the Europe
//  for (int b = 0; b < 61; b = b + 15) {
//    for (int c = 0; c < 46; c = c + 15) {
//      tft.drawCircle(b * 5, c * 5, 5, ST77XX_YELLOW);
//    }
//  }


  int LonA =round( (-23.6772 + 30 ) * 5); // //64.757511, -23.677213 (west iceland)
  int LatA =round( (  60 -64.7575 ) * 5);
  tft.drawCircle(LonA, LatA, 4, ST77XX_RED);
  LonA =round( ( -10.3207 + 15 ) * 5); // //51.849399, -10.320701 (Kerry Ireland)
  LatA =round( ( 60 - 51.8493 ) * 5);
  tft.drawCircle(LonA, LatA, 4, ST77XX_RED);
  LonA =round( ( 18.173264 + 15 ) * 5); // //40.357003, 18.173264 (LECCE ITALIE)
  LatA =round( ( 60 - 40.357003 ) * 5);
  tft.drawCircle(LonA, LatA, 4, ST77XX_RED);
// Draw 1 line for longitude 0
  tft.drawLine(15*5,  0, 15*5, 225, ST77XX_RED);
}


/*
// *******************************************************************
void DrawMapJapan() {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(90, 120);
    tft.print("Japan");
  //drawBmp("/MapJapan300.bmp", 0, 0);
  //
  //  croping is
  // startin at lat 75° to lat 30°
  // ending at lon : 105 to lon 165°
  // 
  // screen is 300 x 225
  //   
  // drawCircle(int32_t x0, int32_t y0, int32_t r, uint32_t color),
// double  dMyLAT  =  48.91052;  // Latitude  - Asnières (France): N -> +, S -> -
// double  dMyLON  =   2.29100;  // Longitude - Asnières (France): E -> +, W -> -
dMyLAT  =  42.361145;   //Boston, MA
dMyLON  =  -71.057083; //Boston, MA
  //int xMyPos = ( 2.29100 + 30 ) * 5; //
  //int yMyPos = ( 75 - 48.91052 ) * 5; //

 // Boston, MA  LAT 42.361145, LON  -71.057083 
 //
  int xMyPos = (-71.057083); //LON
  int yMyPos = (42.361145); //LAT
  
  tft.drawCircle(xMyPos, yMyPos, 2, ST77XX_RED);
  tft.drawCircle(xMyPos, yMyPos, 6, ST77XX_RED);
}
*/
// *******************************************************************
// ***** Mark a dot when ISS is above my head
//
void SatAlert()
{
//dMyLAT  =  48.91052;  // Latitude  - Asnières (France): N -> +, S -> -
//dMyLON  =   2.29100;  // Longitude - Asnières (France): E -> +, W -> -
dMyLAT  =  42.361145;   //Boston, MA
dMyLON  =   -71.057083; //Boston, MA
//sat.satLon
//sat.satLat
/*
if (digitalRead(2) == HIGH && digitalRead(3) == HIGH) { // if BOTH the switches read HIGH
*/

//       47     <     50 + 5  et      47 >          50 - 5          
if ( sat.satLon < 50  )
    {
      //tft.drawCircle(200, 200, 20, ST77XX_YELLOW);
      Serial.println("yellow");
    }
    else
    {
      //tft.drawCircle(200, 200, 20, ST77XX_RED);
      Serial.println("red.....");
    }
}

// *******************************************************************
// ***** splash screen
//
void SplashScreen(){
  //
  
}




void loop()
{
  IssLat = 0;
  IssLon = 0;
    if (TIMELOOP == true) {  // to create a time loop, timelapse from the same date
    epochTime = epochTime + 5;
    }
    else {
    epochTime = getTime();
    }

  //DrawWatch();
  //tft.setFreeFont(TT1);     // Select the orginal small TomThumb font
  int i = 0;
  int b = 0;
  int c = 0;
  int ZoomLatStart = 60;
  int ZoomLatEnd   = ZoomLatStart -45 ;
  int ZoomLonStart = -15;  // value -15
  int ZoomLonEnd   = ZoomLonStart + 60;   // value is 45
//
//                                                   ******************
DrawMap();  // *****************************************   World Map *********************
//                                                   ******************
// Satellite draw path
while ( ( IssLon < ZoomLonStart && IssLat < ZoomLatStart ) or IssLon > ZoomLonEnd or IssLat < ZoomLatEnd ) {

    digitalWrite(PIN_LED, 0);
  
    if (TIMELOOP == true) {  // to create a time loop, timelapse from the same date
    epochTime = epochTime + 5;
    //tft.printNumber(1, 5, 5, FONT4);
    //tft.drawString("TIME warp...", 1, 5, FONT2);
    }
    else {
    epochTime = getTime();
    //tft.drawString("TIME normal...", 1, 5, FONT2);
    //tft.printNumber(0, 5, 5);
    }
    uint16_t colors[] = {ST77XX_BLUE, ST77XX_RED, ST77XX_YELLOW, ST77XX_ORANGE, ST77XX_WHITE, ST77XX_MAGENTA, ST77XX_CYAN};
  for (i=0; i<Sat; i++) {
   // tft.setFreeFont(TT1); 
    sat.init(TLENameChar[i],TLE1Char[i],TLE2Char[i]);     // initialize satellite parameters 
    sat.findsat(epochTime);
    
      if (i==0) {
         IssLat = sat.satLat;
         IssLon = sat.satLon;
        }
        
    float xx = ( ( sat.satLon + 180 ) *  240 ) / 360 ;    // Longitude is 360 wide but screen 240 pixels
    float yy = 0;
    int xTft = xx;
    int yTft = (90 - sat.satLat);
    // Why the satellites are all red
    //drawMarkerColor( xTft, yTft, i);
    //tft.drawPixel(xTft, yTft, ST77XX_RED);
    tft.drawPixel(xTft, yTft, colors[i % 6]);
    }

    #ifdef DEBUG
    Serial.print("do world =  ");
    Serial.print(b);
    b++;
    Serial.print(" -->> lat : ");
    Serial.print(sat.satLat);
    Serial.print(" - Lon : ");
    Serial.println(sat.satLon);
    #endif
  //Keep
    //tft.setTextSize(2);  
    
    DrawWatch();
  }

//tft.fillScreen(ST77XX_BLACK);
//
//

//
//                                                          ******************
DrawMapEurope();  // *****************************************   Europe Map *********************
//                                                          ******************
// europe map is starting at lat : 75N and lon : -30W
//                 ending at lat : 30N and lon :  30E
// map is 60° wide and 45° high
// but the screen is from 0 to 300 pixel wide    =  Lon  = xTFT
//                        0 to 225 pixels high   =  Lat  = yTFT
// 
// sgp4 is provide Lat values from -180 to 180 and Lon values from 9à to -90
// for sat.lon value - xTFT = (sat.satLon + 30 ) * 5
//     sat.Lat value - yTft = (sat.satLat -15 ) * 5
//
//  ISS
  while  (  IssLon + 1 >= ZoomLonStart ) {
 
    if (TIMELOOP == true) {  // to create a time loop, timelapse from the same date
    epochTime = epochTime + 5;
    }
    else {
    epochTime = getTime();
    }

    sat.init(TLENameChar[0],TLE1Char[0],TLE2Char[0]);     // initialize satellite parameters 
    sat.findsat(epochTime);
    IssLat = sat.satLat;
    IssLon = sat.satLon;

    //int xx = ( sat.satLon - ZoomLonStart) * 5 ;           // Longitude is 360 wide but screen 320 pixels
    //int yy = ( 60 - sat.satLat ) * 5 ;
    int xx = (sat.satLon + 90) * 2;  //for my adafruit esp32 tft that is 240, 135
    //int yy = (sat.satLat + 75) * 3;
    int yy = (sat.satLat + 75) * 2;
    int xTft = xx;
    int yTft = yy;
    tft.drawCircle(xTft, yTft, 3, ST77XX_BLUE);
    
    #ifdef DEBUG // serial is sending lat and lon
    Serial.print("Europe = ");
    Serial.print(c);
    c++;
    Serial.print("-> lat: ");
    Serial.print(sat.satLat);
    Serial.print(" y = ");
    Serial.print(yTft);
    Serial.print(" / Lon: ");
    Serial.print(sat.satLon);
    Serial.print(" x = ");
    Serial.println(xTft);
    #endif
     // tft.setFreeFont(TT1);                            // Select the orginal small TomThumb font
      tft.fillRect(0, yd, 25, 3, ST77XX_BLUE);            // fillRect(0, 0, _width, _height, color);
      //tft.drawString(TLENameChar[i], 30, yd);
      tft.setCursor(30, yd);
      tft.print(TLENameChar[i]);
    
      tft.setTextColor(ST77XX_WHITE); // Set text color to white
      tft.setTextSize(2); // Set text size to 2

// A two-line element set (TLE) is a data format encoding a list of orbital elements of an Earth-orbiting object for a given point in time, the epoch.

  String message2 = "Looking for the latest TLE";

    int x = 30; // X position of the first character
    int y = yd; // Y position of the characters
    for (int j = 0; j < strlen(TLENameChar[i]); j++) {
    //  tft.drawChar(x, y, TLENameChar[i][j], FONT2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(20, 135);
    //for(int i=0; i<message2.length(); i++){
    // Ben - we use message2.c_str() to convert the message2 string to a C-string and assign it to the message2CStr variable. We then pass message2CStr to the strlen() function to get the length of the string. Finally, we loop over the characters in message2 and print them to the display. 
    const char* message2CStr = message2.c_str();
    for(int i=0; i<strlen(message2CStr); i++){
      tft.print(message2.charAt(i));
      tft.setCursor(tft.getCursorX() + 6, tft.getCursorY());
}
    tft.setTextColor(ST77XX_BLACK);

      x += 6; // Increase x position to space the characters out
}

  //    tft.printNumber(sat.satLon,     90, yd);
  //    tft.printNumber(sat.satLat,    110, yd);
int16_t x_lon = 90; // x coordinate for longitude
int16_t x_lat = 110; // x coordinate for latitude
int16_t yc = yd; // y coordinate for both

String lon_str = String(sat.satLon, 6); // convert longitude to string with 6 decimal places
String lat_str = String(sat.satLat, 6); // convert latitude to string with 6 decimal places

uint8_t font_size = 2; // font size to use
int16_t char_width = font_size * 6; // approximate width of each character

// print the longitude value to the display
for (int i = 0; i < lon_str.length(); i++) {
  char c = lon_str.charAt(i);
 // tft.drawChar(x_lon + (i * char_width), y, c, WHITE, BLACK, font_size);
tft.setCursor(x_lon + (i * char_width), yc);
tft.setTextColor(ST77XX_WHITE);
tft.setTextSize(font_size);
tft.print(c);

}

// print the latitude value to the display
for (int i = 0; i < lat_str.length(); i++) {
  char c = lat_str.charAt(i);
 // tft.drawChar(x_lat + (i * char_width), y, c, WHITE, BLACK, font_size);
tft.setCursor(x_lon + (i * char_width), yc);
tft.setTextColor(ST77XX_WHITE);
tft.setTextSize(font_size);
tft.print(c);
  
}
  
       
      if (  IssLon >= ZoomLonEnd or IssLat <= ZoomLatEnd ) {
        break;
      }
    DrawWatch();

    if (TIMELOOP == true) {  // to create a time loop, timelapse from the same date
    delay(20);
    }
    else {
    digitalWrite(PIN_LED, 1);
    delay(500);
    digitalWrite(PIN_LED, 0);
    delay(1500);
    }
  }
  
  tft.fillRect(0,120, 240,135, ST77XX_BLACK);
  tft.fillScreen(ST77XX_BLACK);
  
} // end of loop

// *******************************************************************


void printDigits(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


// *******************************************************************
// Progress bar helper
void drawProgress(uint8_t percentage, String text) {
"xxx";
}


// *******************************************************************
/*-------- NTP code ----------*/
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
      //return secsSince1900 - 2208988800UL + (timeZone - 5) * SECS_PER_HOUR; // EDT -5, EST -4
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// *******************************************************************
// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}


// *******************************************************************
// Function to extract numbers from compile time string
static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}


// *******************************************************************
/*
void DrawWatch(){    
 // tft.setFreeFont(TT1);           // Select the font
  // Draw hours and minutes
  xposInit = xpos;
   // if (hr_ < 10) xpos  += tft.drawChar('0',    xpos, ypos, xyfont);     // Add hours leading zero for 24 hr clock
      if (hr_ < 10) xpos  += tft.drawChar(xpos, ypos, '0', ST77XX_WHITE, ST77XX_BLACK, 2);     // Add hours leading zero for 24 hr clock
      xpos              += tft.drawNumber(hr_,  xpos, ypos, xyfont);     // Draw hours
      xcolon = xpos;                                                     // Save colon coord for later to flash on/off later
      xpos              += tft.drawChar(':',    xpos, ypos, xyfont);     // adding a char space for ':' in xpos
    if (min_ < 10) xpos += tft.drawChar('0',    xpos, ypos, xyfont);     // Add minutes leading zero
      xpos              += tft.drawNumber(min_, xpos, ypos, xyfont);     // Draw minutes
      xpos              += tft.drawChar(':',    xpos, ypos, xyfont);     // adding a char space for ':' in xpos
      xsecs = xpos;                                                      // Save seconds 'x' position for later display updates
    if (sec_ < 10) xpos += tft.drawChar('0',    xpos, ypos, xyfont);     // Add leading zero
                           tft.drawNumber(sec_, xpos, ypos, xyfont);     // Draw seconds
   xpos = xposInit;
}
*/

void DrawWatch() {
  // Set time zone to EST
  setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0/2", 1);

  // Get current time in EST
  time_t now;
  struct tm * timeinfo;
  time(&now);
  timeinfo = localtime(&now);
  int hr_ = timeinfo->tm_hour;
  int min_ = timeinfo->tm_min;
  int sec_ = timeinfo->tm_sec;

  // Draw hours and minutes
  xposInit = xpos;
  if (hr_ < 10) {
    tft.setCursor(xpos, ypos);
    tft.print('0');
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds("0", 0, 0, &x1, &y1, &w, &h);
    xpos += w;
  }
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(xpos, ypos);
  tft.printf("%02d:%02d:%02d", hr_, min_, sec_);
  xpos = xposInit;
}



// ************************Origional location of void Second_Tick () {*******************************************


// *******************************************************************
// There follows a crude way of flagging that this example sketch needs fonts which
// have not been enbabled in the User_Setup.h file inside the TFT_HX8357 library.
//
// These lines produce errors during compile time if settings in User_Setup are not correct
//
// The error will be "does not name a type" but ignore this and read the text between ''
// it will indicate which font or feature needs to be enabled
//
// Either delete all the following lines if you do not want warnings, or change the lines
// to suit your sketch modifications.
/*
#ifndef LOAD_GLCD
//ERROR_Please_enable_LOAD_GLCD_in_User_Setup
#endif

#ifndef LOAD_GFXFF
ERROR_Please_enable_LOAD_GFXFF_in_User_Setup!
#endif
*/
