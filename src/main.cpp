#include <Arduino.h>
#include "FastLED.h"
#include "ANADRUM_Pins.h"
#include "ANADRUM_Tasks.h"
#include <MIDI.h>

#define NUM_CHANNELS 4
#define NUM_PIXELS 4
#define STEP_TIME 80
#define WAIT_TIME_AFTER_FIRE 20
#define MIN_STEPS_TO_MOVE 400

#include "ANADRUM_Channel.h"

long dimmer_off_time = 5000;
hw_timer_t *phase_attack_timer = NULL;
TaskHandle_t led_task_handle;
TaskHandle_t channel_task_handle;

boolean do_fire;
int current_channel;
int fired_channel;

CRGB pixel[NUM_PIXELS];
int next_firing_instance;

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, DIN_MIDI);

ANADRUM_Channel channel[4];

void IRAM_ATTR zerocross_interrupt();

//----------------------------------------------------------------------------------------
//                                                            End Zero Crossing
void IRAM_ATTR onTimer()
{
  digitalWrite(PIN_FIRE[current_channel], HIGH);
  fired_channel = current_channel;
}

//----------------------------------------------------------------------------------------
//                                                            Zero Crossing
void IRAM_ATTR zerocross_interrupt() //
{

  digitalWrite(PIN_FIRE[fired_channel], LOW);

  if (!do_fire)
    return;

  if (!digitalRead(PIN_ZEROCROSS))
    return;

  timerWrite(phase_attack_timer, 0);
  timerAlarmEnable(phase_attack_timer);
  do_fire = false;
}

//----------------------------------------------------------------------------------------
void test(void *p)
{
  while (1)
  {
    log_v("Alive");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
//----------------------------------------------------------------------------------------
void channel_task(void *p)
{
  int has_steps;
  while (1)
  {
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
      channel[i].loop();
      has_steps += channel[i].steps_to_move;
    }
    if (!has_steps)
    {
      vTaskDelay(CHANNEL__TASK_DELAY / portTICK_PERIOD_MS);
    }
    else
    {
      delayMicroseconds(STEP_TIME);
    }
  }
}

//----------------------------------------------------------------------------------------
void led_task(void *p)
{
  while (1)
  {
    int pix;
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
      pix = NUM_CHANNELS - 1 - i;
      if (channel[i].ready)
      {
        pixel[pix] = CRGB::Green;
      }
      else
      {
        pixel[pix] = CRGB::Red;
      }
      if (channel[i].steps_to_move > 0)
      {
        pixel[pix] = CRGB::Yellow;
      }
    }
    FastLED.show();
    vTaskDelay(LED_TASK_DELAY / portTICK_PERIOD_MS);
  }
}

//----------------------------------------------------------------------------------------
//		                                                              MIDI NOTEON

void handle_note_on(uint8_t chann, uint8_t pitch, uint8_t vel)
{
  // find next ready channel that has the least of shots fired
  int c, s;
  s = 12000;
  c = NUM_CHANNELS;
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    if (channel[i].ready)
    {
      if (channel[i].shots_fired < s)
      {
        s = channel[i].shots_fired;
        c = i;
      }
    }
  }

  if (c < NUM_CHANNELS)
  {
    channel[c].fire();
    do_fire = true; // trigger next zerocross
    current_channel = c;
    log_v("Fire channel %d", c);
  }
  else
  {
    log_e("No Channel ready to fire");
  }
}

//----------------------------------------------------------------------------------------
//		                                                              MIDI CC

void handle_controlchange(uint8_t chann, uint8_t cc, uint8_t val)
{
  log_v("CC: %d %d", cc, val);
  if (cc == 10)
  {
    dimmer_off_time = map(val, 0, 127, 9000, 500);
    timerAlarmWrite(phase_attack_timer, dimmer_off_time, false);
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

  Serial1.begin(31250, SERIAL_8N1, PIN_MIDI_RX);
  DIN_MIDI.begin(); // Launch MIDI, by default listening to channel 1.
  DIN_MIDI.setHandleNoteOn(handle_note_on);
  DIN_MIDI.setHandleControlChange(handle_controlchange);

  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    channel[i].begin(PIN_ENABLE[i], PIN_DIR[i], PIN_PULSE[i], PIN_FIRE[i], PIN_STOP_A[i]);
  }

  // xTaskCreate(test,"Test Loop",10000,NULL,0,NULL);

  xTaskCreatePinnedToCore(
      led_task,            /* Function to implement the task */
      "LED Task",          /* Name of the task */
      LED_TASK_STACK_SIZE, /* Stack size in words */
      NULL,                /* Task input parameter */
      LED_TASK_PRIORITY,   /* Priority of the task */
      &led_task_handle,    /* Task handle. */
      LED_TASK_CORE);      /* Core where the task should run */

  xTaskCreatePinnedToCore(
      channel_task,             /* Function to implement the task */
      "CHANNEL Task",           /* Name of the task */
      CHANNEL__TASK_STACK_SIZE, /* Stack size in words */
      NULL,                     /* Task input parameter */
      CHANNEL__TASK_PRIORITY,   /* Priority of the task */
      &channel_task_handle,     /* Task handle. */
      CHANNEL__TASK_CORE);

  pinMode(PIN_ZEROCROSS, INPUT_PULLDOWN);
  attachInterrupt(PIN_ZEROCROSS, zerocross_interrupt, RISING);

  phase_attack_timer = timerBegin(2, 80, true);
  timerAttachInterrupt(phase_attack_timer, &onTimer, true);
  timerAlarmWrite(phase_attack_timer, dimmer_off_time, false);

  log_v("Setup Done");
}

//========================================================================================
//----------------------------------------------------------------------------------------
//	                                                                            LOOP
void loop()
{
  DIN_MIDI.read();
  delay(1);
}