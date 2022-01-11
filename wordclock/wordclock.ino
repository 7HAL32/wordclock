#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <FastLED.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <FS.h>
#include <DoubleResetDetector.h>

#include "src/grid.h"
#include "src/wifi.h"
#include "src/types.h"
#include "src/color.h"
#include "src/config.h"
#include "src/dnd.h"
#include "src/led.h"
#include "src/time.h"
#include "src/httpServer.h"
#include "src/gui.h"
#include "src/controller.h"
#include "src/healthcheck.h"

// Number of seconds after reset during which a 
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 3

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

// PRIDE
#define DATA_PIN    D4
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    122
#define BRIGHTNESS  255

CRGB leds[NUM_LEDS];
bool partyMode = false;

void setup() {
  delay(3000); // 3 second delay for recovery
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println();
  Serial.println("Hello world");
  SPIFFS.begin();

  
  if (drd.detectDoubleReset()) {
    Serial.println("reset with partyMode");
    Serial.println(partyMode);
    partyMode = !partyMode;

    // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS)
    .setCorrection(TypicalLEDStrip)
    .setDither(BRIGHTNESS < 255);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  } else {
    Serial.println("No Double Reset Detected");
    Config::load();

  Wifi::setup();
  HttpServer::setup();
  Led::setup();
  Time::setup();

  Grid::setTime(Time::hour, Time::minute);
  
  if (Config::healthcheck ){
    // run simple LED check at start (uncomment to deactivate)
    Healthcheck::run();
  }
  }
  
 
  
  

  if (DATA_PIN != D4){
    // If we use a data pin different than d4, we will deactivate d4 because it triggers the blue board status led
    // I use D3 as data pin, because d4 is connected with the board led. 
    pinMode(D4, OUTPUT); // Define LED pin as output
    digitalWrite(D4, HIGH); // Switch the blue board status LED OFF
  }


  
}

void loop() {
  // Call the double reset detector loop method every so often,
  // so that it can recognise when the timeout expires.
  // You can also call drd.stop() when you wish to no longer
  // consider the next reset as a double reset.
  drd.loop();

  if (partyMode){
    pride();
    FastLED.show(); 
  } else {
    Time::loop();
    HttpServer::loop();
  }

}


// This function draws rainbows with an ever-changing,
// widely-varying set of parameters.
void pride() 
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;
 
  uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);
  
  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5,9);
  uint16_t brightnesstheta16 = sPseudotime;
  
  for( uint16_t i = 0 ; i < NUM_LEDS; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);
    
    CRGB newcolor = CHSV( hue8, sat8, bri8);
    
    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS-1) - pixelnumber;
    
    nblend( leds[pixelnumber], newcolor, 64);
  }
}
