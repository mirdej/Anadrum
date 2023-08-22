#include <Arduino.h>
#include "FastLED.h"
#include "ANADRUM_Pins.h"

#define NUM_PIXELS 4
long dimmer_off_time = 5000;
hw_timer_t *phase_attack_timer = NULL;
CRGB pixel[NUM_PIXELS];

void test(void *p)
{
  while (1)
  {
    log_v("Alive");
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}
//========================================================================================
//----------------------------------------------------------------------------------------
//		                                                              SETUP
void setup()
{
  Serial.begin(115200);
  delay(2000); // Delay booting to give time to connect a serial monitor

  FastLED.addLeds<SK6812, PIN_PIX, GRB>(pixel, NUM_PIXELS);
  FastLED.setBrightness(100);
  pixel[0] = CRGB::Blue;
  pixel[1] = CRGB::Green;
  pixel[2] = CRGB::Yellow;
  pixel[3] = CRGB::Orange;
  FastLED.show();

  xTaskCreate(
      test,
      "Test Loop",
      10000,
      NULL,
      0,
      NULL);

  log_v("Setup Done");
}

//========================================================================================
//----------------------------------------------------------------------------------------
//	                                                                            LOOP
void loop()
{
}