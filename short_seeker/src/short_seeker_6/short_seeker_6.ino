// Modular tester
// short seeker module
// Jef Collin 2024


// revisions
// 1.0 first release




// todo





#include <TFT_eSPI.h>
#include <SPI.h>
#include "Free_Fonts.h"
#include "esp_sleep.h"
#include <Wire.h>
#include "ADS1X15.h"

#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include "driver/dac.h"

// use alps or other decoder
#define Use_Alps_Encoder false

TFT_eSPI LCD_DISPLAY = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

TFT_eSprite LCD_SPRITE2 = TFT_eSprite(&LCD_DISPLAY);


// Define the C array for the 'Ω' character 20 pix
const uint8_t OmegaBitmap[20][20] = {
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};

// shortseek io pins

#define shortseek_relay_current1 13
#define shortseek_relay_current2 19
#define shortseek_relay_gain1 16
#define shortseek_relay_gain2 17

// LCD
#define Backlight_control 26

// current setting
uint8_t shortseek_current = 1;
// gain setting
uint8_t shortseek_gain = 1;

// mute audio
boolean shortseek_mute = true;
// out of window range
boolean shortseek_out_of_range = false;
// keep track of dac status
boolean shortseek_dac_running = false;
// mode for led bar
boolean shortseek_singleLED = true;
// auto or manual mode
boolean shortseek_manualmode = true;
// automatic calibration state machine
uint8_t shortseek_auto_state = 0;
// keep track of measured ohm value
float shortseek_auto_reference = 0;
// timer for autorecalibrate
unsigned long shortseek_auto_timer1;

// audio volume
uint8_t shortseek_volume = 0;

uint8_t shortseek_sensitivity = 0;

uint8_t shortseek_button2mode = 0;

// sensitivity range
uint8_t shortseek_range = 3;
// led bar
uint8_t shortseek_gauge = 0;

uint8_t shortseek_previousgauge = 99;

uint8_t shortseek_averagefactor = 2;

// timer for updates
unsigned long shortseek_updatetimer;
// interval for measurement updates
uint16_t shortseek_updateinterval = 10;

float shortseek_voltage = 0;
float shortseek_ohm = 0;
float shortseek_previousohm = 999;

// voltage offset per range
float shortseek_zero_range_0 = 0;
float shortseek_zero_range_1 = 0;
float shortseek_zero_range_2 = 0;
float shortseek_zero_range_3 = 0;

float shortseek_reference = 0;
float shortseek_window = 100;

// default frequency and range
double shortseek_frequency = 1000;
double shortseek_frequency_low = 300;
double shortseek_frequency_high = 1600;

// sleep or startup
boolean shortseek_normalstart = false;

// Menu variables
const int numMenuItems = 4;

// Menu variables
const char* menuItems[] = {"Return", "Manual mode", "Single LED", "Go to sleep"};
int selectedMenu = 0;
int menuOffset = 0;

uint8_t ScreenMode = 0;
boolean ForceNewMenu = true;

// adc module
ADS1115 ADS(0x48);

// rotary encoder
// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

#if (Use_Alps_Encoder)
// Alps EC11 encoder requires half step tables, others need full step
#define R_START 0x0
#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5

const unsigned char ttable[6][4] = {
  // R_START (00)
  {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
  // R_CCW_BEGIN
  {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
  // R_CW_BEGIN
  {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
  // R_START_M (11)
  {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
  // R_CW_BEGIN_M
  {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
  // R_CCW_BEGIN_M
  {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};

#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6
#define R_START 0x0

const unsigned char ttable[7][4] = {
  // R_START
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif

unsigned char Encoder_1_State = R_START;
unsigned char Encoder_2_State = R_START;

// rotary encoders io pins
char Encoder_1_Pin1 = 36;
char Encoder_1_Pin2 = 39;
char Encoder_1_Key = 34;

char Encoder_2_Pin1 = 33;
char Encoder_2_Pin2 = 32;
char Encoder_2_Key = 35;

// track rotary encoder changes
int EncoderCounter1 = 0;
int EncoderCounter2 = 0;

long unsigned timer_encoderbutton1;
long unsigned timer_encoderbutton2;

boolean Encoder_Key1_Long_Press = false;
boolean Encoder_Key2_Long_Press = false;

// interrupt routine for rotary encoder
void IRAM_ATTR isr1() {
  unsigned char pinstate = (digitalRead(Encoder_1_Pin2) << 1) | digitalRead(Encoder_1_Pin1);
  Encoder_1_State = ttable[Encoder_1_State & 0xf][pinstate];
  unsigned char result = Encoder_1_State & 0x30;
  if (result == DIR_CW) {
    if (EncoderCounter1 < 30) {
      EncoderCounter1++;
    }
  } else if (result == DIR_CCW) {
    if (EncoderCounter1 > -30) {
      EncoderCounter1--;
    }
  }
}

// interrupt routine for rotary encoder
void IRAM_ATTR isr2() {
  unsigned char pinstate = (digitalRead(Encoder_2_Pin2) << 1) | digitalRead(Encoder_2_Pin1);
  Encoder_2_State = ttable[Encoder_2_State & 0xf][pinstate];
  unsigned char result = Encoder_2_State & 0x30;
  if (result == DIR_CW) {
    if (EncoderCounter2 < 30) {
      EncoderCounter2++;
    }
  } else if (result == DIR_CCW) {
    if (EncoderCounter2 > -30) {
      EncoderCounter2--;
    }
  }
}

void setup() {

  //  Serial.begin(115200);

  // setup encoder pins
  pinMode(Encoder_1_Pin1, INPUT);
  pinMode(Encoder_1_Pin2, INPUT);
  pinMode(Encoder_1_Key, INPUT);

  pinMode(Encoder_2_Pin1, INPUT);
  pinMode(Encoder_2_Pin2, INPUT);
  pinMode(Encoder_2_Key, INPUT);

  // setup i2c interface
  Wire.begin();

  // setup relays
  pinMode(shortseek_relay_current1, OUTPUT);
  pinMode(shortseek_relay_current2, OUTPUT);
  pinMode(shortseek_relay_gain1, OUTPUT);
  pinMode(shortseek_relay_gain2, OUTPUT);

  // set current
  ShortseekSetCurrent(0);

  // set gain
  ShortseekSetGain(0);

  pinMode(Backlight_control, OUTPUT);

  // turn off backlight
  digitalWrite(Backlight_control, HIGH);

  // Setup the LCD
  LCD_DISPLAY.init();
  LCD_DISPLAY.setRotation(3);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  delay(100);
  // turn on backlight
  digitalWrite(Backlight_control, LOW);

  // check for key pressed, enable normal start otherwise go to sleep
  if (digitalRead(Encoder_1_Key) == 0) {
    shortseek_normalstart = true;
  }

  DisplaySplash();

  if (shortseek_normalstart) {
    // wait until key is no longer pressed
    while (digitalRead(Encoder_1_Key) == 0) {}
  }

  if (shortseek_normalstart) {
    delay(4000);
  }
  else {
    delay(2500);
  }

  LCD_DISPLAY.fillScreen(TFT_BLACK);

  // goto sleep
  if (!shortseek_normalstart) {
    GoToSleep();
  }

  // adc setup
  ADS.begin();
  ADS.setGain(8);      //
  // was 3 in first test
  ADS.setDataRate(4);  //
  ADS.readADC(0);      // first read to trigger
  ADS.setMode(1);      // single mode
  ADS.readADC(0);      // first read to trigger

  // get current state to start otherwise encoder might not react to first click
  unsigned char temppinstate1 = (digitalRead(Encoder_1_Pin2) << 1) | digitalRead(Encoder_1_Pin1);
  Encoder_1_State = ttable[Encoder_1_State & 0xf][temppinstate1];

  unsigned char temppinstate2 = (digitalRead(Encoder_2_Pin2) << 1) | digitalRead(Encoder_2_Pin1);
  Encoder_2_State = ttable[Encoder_2_State & 0xf][temppinstate2];

  // setup encoder interrupts
  attachInterrupt(Encoder_1_Pin1, isr1, CHANGE);
  attachInterrupt(Encoder_1_Pin2, isr1, CHANGE);
  attachInterrupt(Encoder_2_Pin1, isr2, CHANGE);
  attachInterrupt(Encoder_2_Pin2, isr2, CHANGE);

   LCD_SPRITE2.createSprite(160, 128);
   
  BuildScreen(0);

  // initial measurement
  ShortseekSetRange(shortseek_range);
  shortseek_voltage = ShortseekReadVoltage();
  shortseek_voltage = ShortseekReadVoltage();
  shortseek_voltage = ShortseekReadVoltage();
  shortseek_ohm = ShortseekCalculateOhm(shortseek_voltage);
  shortseek_reference = shortseek_ohm;
  shortseek_window = ShortseekSetSensitivity(shortseek_sensitivity, shortseek_range);

  DAC_SetFrequency(1000);

  // force first read
  shortseek_updatetimer = millis() - 1000;

}

void loop() {

  switch (ScreenMode) {
    case 0:
      // running mode
      // key 1 short reset, long menu
      if (digitalRead(Encoder_1_Key) == 0) {
        timer_encoderbutton1 = millis();
        Encoder_Key1_Long_Press = false;
        // wait until key is no longer pressed or time expired
        while (digitalRead(Encoder_1_Key) == 0) {
          if (millis() - timer_encoderbutton1 > 1000) {
            Encoder_Key1_Long_Press = true;
            break;
          }
        }
        if (Encoder_Key1_Long_Press) {
          // long press
          ScreenMode = 1;
          selectedMenu = 0;
          menuOffset = 0;
          EncoderCounter1 = 0;
          DrawMenu();
          while (digitalRead(Encoder_1_Key) == 0) {}
          //little debounce
          delay(200);
        }
        else {
          // short press
          if (shortseek_manualmode) {
            // ignore if open
            if (ShortseekCheckRange(shortseek_ohm, shortseek_range)) {
              // reset current ohm value to be center reference
              shortseek_reference = shortseek_ohm;
            }
          }
          else {
            // automatic mode
            // if not open, same as manual
            if (ShortseekCheckRange(shortseek_ohm, shortseek_range)) {
              // reset current ohm value to be center reference
              shortseek_reference = shortseek_ohm;
            }
            else {
              // open probes
              shortseek_auto_state = 0;
              BuildScreen(4);
            }
          }
          // force rebuild
          shortseek_previousgauge = 99;
          shortseek_previousohm = 999;
        }
      }

      // key 2 short switch mode range or sensitivity, long set zero
      if (digitalRead(Encoder_2_Key) == 0) {
        timer_encoderbutton2 = millis();
        Encoder_Key2_Long_Press = false;
        // wait until key is no longer pressed or time expired
        while (digitalRead(Encoder_2_Key) == 0) {
          if (millis() - timer_encoderbutton2 > 1000) {
            Encoder_Key2_Long_Press = true;
            break;
          }
        }
        if (Encoder_Key2_Long_Press) {
          // long press
          // disable auto mode
          shortseek_manualmode = true;
          BuildScreen(5);
          // zero set only if probes not open
          if (ShortseekCheckRange(shortseek_ohm, shortseek_range)) {
            ShortseekSetZero(&shortseek_zero_range_0, &shortseek_zero_range_1, &shortseek_zero_range_2, &shortseek_zero_range_3);
          }
          // reset range
          ShortseekSetRange(shortseek_range);
          while (digitalRead(Encoder_2_Key) == 0) {}
          //little debounce
          delay(200);
          BuildScreen(0);
        }
        else {
          // short press
          if (shortseek_button2mode == 0) {
            shortseek_button2mode = 1;
          }
          else {
            shortseek_button2mode = 0;
          }
          BuildScreen(3);
        }
        shortseek_previousgauge = 99;
        shortseek_previousohm = 999;
      }

      if (EncoderCounter1 != 0) {
        if (EncoderCounter1 > 0) {
          if (shortseek_volume < 4) {
            shortseek_volume++;
            ShowVolume(shortseek_volume);
            ShortseekSetVolume(shortseek_volume);
          }
          EncoderCounter1--;
        }
        else {
          if (shortseek_volume > 0) {
            shortseek_volume--;
            ShowVolume(shortseek_volume);
            ShortseekSetVolume(shortseek_volume);
          }
          EncoderCounter1++;
        }
      }

      if (EncoderCounter2 != 0) {
        if (shortseek_button2mode == 0) {
          // range
          if (EncoderCounter2 > 0) {
            if (shortseek_range < 3) {
              shortseek_range++;
              ShortseekSetRange(shortseek_range);
              shortseek_window = ShortseekSetSensitivity(shortseek_sensitivity, shortseek_range);
              BuildScreen(2);
              if (shortseek_auto_state == 2 and !shortseek_manualmode) {
                shortseek_auto_state = 0;
                BuildScreen(4);
              }
            }
            EncoderCounter2--;
          }
          else {
            if (shortseek_range > 0) {
              shortseek_range--;
              ShortseekSetRange(shortseek_range);
              shortseek_window = ShortseekSetSensitivity(shortseek_sensitivity, shortseek_range);
              BuildScreen(2);
              if (shortseek_auto_state == 2 and !shortseek_manualmode) {
                shortseek_auto_state = 0;
                BuildScreen(4);
              }
            }
            EncoderCounter2++;
          }
        }
        else {
          // sensitivity
          if (EncoderCounter2 != 0) {
            if (EncoderCounter2 > 0) {
              if (shortseek_sensitivity < 75) {
                shortseek_sensitivity++;
                ShowSensitivity(shortseek_sensitivity);
                shortseek_window = ShortseekSetSensitivity(shortseek_sensitivity, shortseek_range);
                if (shortseek_auto_state == 2 and !shortseek_manualmode) {
                  shortseek_auto_state = 0;
                  BuildScreen(4);
                }
              }
              EncoderCounter2--;
            }
            else {
              if (shortseek_sensitivity > 0) {
                shortseek_sensitivity--;
                ShowSensitivity(shortseek_sensitivity);
                shortseek_window = ShortseekSetSensitivity(shortseek_sensitivity, shortseek_range);
                if (shortseek_auto_state == 2 and !shortseek_manualmode) {
                  shortseek_auto_state = 0;
                  BuildScreen(4);
                }
              }
              EncoderCounter2++;
            }
          }
        }
        shortseek_previousgauge = 99;
        shortseek_previousohm = 999;
      }
      break;

    case 1:
      // menu mode
      // check for key pressed
      if (digitalRead(Encoder_1_Key) == 0) {
        switch (selectedMenu) {
          case 0:
            // return
            // rebuild all
            ScreenMode = 0;
            BuildScreen(ScreenMode);
            shortseek_previousgauge = 99;
            shortseek_previousohm = 999;
            ShowSensitivity(shortseek_sensitivity);
            ShowVolume(shortseek_volume);
            break;

          case 1:
            // toggle manual/automatic
            shortseek_manualmode = !shortseek_manualmode;
            DrawMenu();
            break;

          case 2:
            // toggle led display
            shortseek_singleLED = !shortseek_singleLED;
            DrawMenu();
            break;

          case 3:
            // sleep
            LCD_DISPLAY.fillScreen(TFT_BLACK);
            // wait until key is no longer pressed
            while (digitalRead(Encoder_1_Key) == 0) {}
            // debounce to avoid early wakeup
            delay(100);
            GoToSleep();
            break;

        }
        // wait until key is no longer pressed
        while (digitalRead(Encoder_1_Key) == 0) {}
        // debounce
        delay(100);
      }
      // scroll menu
      if (EncoderCounter1 != 0) {
        if (EncoderCounter1 > 0) {
          if (selectedMenu < numMenuItems - 1) {
            selectedMenu++;
            if (selectedMenu >= menuOffset + 4) {
              menuOffset++;
            } else if (selectedMenu < menuOffset) {
              menuOffset--;
            }
            DrawMenu();
          }
          EncoderCounter1--;
        }
        if (EncoderCounter1 < 0) {
          if (selectedMenu > 0) {
            selectedMenu--;
            if (selectedMenu >= menuOffset + 4) {
              menuOffset++;
            } else if (selectedMenu < menuOffset) {
              menuOffset--;
            }
            DrawMenu();
          }
          EncoderCounter1++;
        }
      }
      break;
  }

  // measurement and update part
  if (ScreenMode == 0 and millis() - shortseek_updatetimer >= shortseek_updateinterval) {

    shortseek_voltage = 0;
    for (uint8_t x = 1; x <= shortseek_averagefactor; x++) {
      shortseek_voltage += ShortseekReadVoltage();
    }

    shortseek_voltage = shortseek_voltage / shortseek_averagefactor;

    // calculate resistance
    shortseek_ohm = ShortseekCalculateOhm(shortseek_voltage);
    ShowOhm(shortseek_ohm, shortseek_range);
    // calculate led bar
    shortseek_gauge = ShortseekCalcGauge(shortseek_ohm, shortseek_reference, shortseek_window, shortseek_range);
    // update led bar
    if (shortseek_singleLED) {
      ShowGaugeSingleLed(shortseek_gauge);
    }
    else {
      ShowGaugeBar(shortseek_gauge);
    }

    shortseek_frequency = ShortseekCalcAudio(shortseek_ohm, shortseek_reference, shortseek_window, shortseek_range, shortseek_frequency_low, shortseek_frequency_high);
    DAC_SetFrequency(shortseek_frequency);
    if (!shortseek_mute and shortseek_out_of_range and shortseek_dac_running) {
      DAC_Stop();
    }
    else if (!shortseek_mute and !shortseek_out_of_range and !shortseek_dac_running) {
      DAC_Start();
      ShortseekSetVolume(shortseek_volume);
    }

    // reset update timer
    shortseek_updatetimer = millis();

    // automatic cal mode state machine
    if (!shortseek_manualmode) {

      switch (shortseek_auto_state) {
        case 0:
          // first state, waiting for first contact
          if (ShortseekCheckRange(shortseek_ohm, shortseek_range)) {
            // we have a measurement, start timer
            shortseek_auto_timer1 = millis();
            // keep for reference
            shortseek_auto_reference = shortseek_ohm;
            //next state
            shortseek_auto_state = 1;
          }
          break;

        case 1:
          // checking for measurement within range and time limit
          if (ShortseekCheckRange(shortseek_ohm, shortseek_range) and (shortseek_ohm >= (shortseek_auto_reference * 0.9)) and (shortseek_ohm <= (shortseek_auto_reference * 1.1))) {
            // still a measurement that is within 10% of the previous one
            if (millis() - shortseek_auto_timer1 > 1000) {
              // stable within +- 10% after 1 second
              shortseek_reference = shortseek_ohm;
              shortseek_auto_state = 2;
              BuildScreen(4);
            }
          }
          else {
            // no more measurement or not within limits, restart
            shortseek_auto_state = 0;
            BuildScreen(4);
          }
          break;

        case 2:
          // stable state
          if (ShortseekCheckRange(shortseek_ohm, shortseek_range)) {
            // valid but outside of window, recalibrate
            if (shortseek_gauge == 0 or shortseek_gauge == 16) {
              if ( millis() - shortseek_auto_timer1 > 1000) {
                // back in calibrating mode
                shortseek_auto_state = 0;
                BuildScreen(4);
              }
            }
            else {
              shortseek_auto_timer1 = millis();
            }
          }
          else {
            shortseek_auto_timer1 = millis();
          }
          break;
      }
    }
  }

}


// build screen
void BuildScreen(uint8_t updatemode) {
  // 0 = all
  // 1 = mute
  // 2 range
  // 3 sens or range selected
  // 4 calibrating in auto mode
  // 5 calibrating offset

  if (updatemode == 0) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);

    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Volume", 0, 35);

    // volume bar
    LCD_DISPLAY.drawLine(83, 37, 159, 37, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(159, 37, 159, 48, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(83, 37, 83, 48, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(83, 48, 159, 48, TFT_DARKGREY);
    // sensitivity bar
    LCD_DISPLAY.drawLine(83, 87, 159, 87, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(159, 87, 159, 98, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(83, 87, 83, 98, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(83, 98, 159, 98, TFT_DARKGREY);
    // gauge bar
    LCD_DISPLAY.drawLine(4, 108, 155, 108, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(155, 108, 155, 127, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(4, 108, 4, 127, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(4, 127, 155, 127, TFT_DARKGREY);

    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TR_DATUM);
    if (!shortseek_manualmode) {
      LCD_DISPLAY.drawString("Auto", 120, 0);
    }
  }

  // update mute status
  if (updatemode == 0 or updatemode == 1) {
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TR_DATUM);
    if (shortseek_mute) {
      LCD_DISPLAY.drawString("Mute", 159, 0);
    }
    else {
      LCD_DISPLAY.drawString("     ", 159, 0);
    }
  }

  // update range setting
  if (updatemode == 0 or updatemode == 2) {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextPadding(30);
    LCD_DISPLAY.setTextDatum(TR_DATUM);
    switch (shortseek_range) {
      case 0:
        LCD_DISPLAY.drawString("0.2 ", 135, 60);
        break;

      case 1:
        LCD_DISPLAY.drawString("2 ", 135, 60);
        break;

      case 2:
        LCD_DISPLAY.drawString("20 ", 135, 60);
        break;

      case 3:
        LCD_DISPLAY.drawString("200 ", 135, 60);
        break;

    }
    LCD_DISPLAY.setTextPadding(0);
    draw20PixBitmap(139, 59, OmegaBitmap, 20, 20, TFT_WHITE);
  }

  // range or sensitivity mode
  if (updatemode == 0 or updatemode == 3) {
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    if (shortseek_button2mode == 0) {
      LCD_DISPLAY.drawString("Range", 0, 60);
    }
    else {
      LCD_DISPLAY.drawString("Sensitivity", 0, 85);
    }
    LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
    if (shortseek_button2mode == 0) {
      LCD_DISPLAY.drawString("Sensitivity", 0, 85);
    }
    else {
      LCD_DISPLAY.drawString("Range", 0, 60);
    }
  }

  if (updatemode == 0 or updatemode == 4) {
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextColor(TFT_ORANGE, TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TR_DATUM);
    if (!shortseek_manualmode and shortseek_auto_state < 2) {
      LCD_DISPLAY.drawString("Calibrating", 159, 15);
    }
    else {
      LCD_DISPLAY.drawString("           ", 159, 15);
    }
  }

  if (updatemode == 5) {
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextColor(TFT_ORANGE, TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TR_DATUM);
    LCD_DISPLAY.drawString("Calibrating", 159, 15);
  }

}

// 3 color gauge
void ShowGaugeBar(uint8_t gauge_value) {
  uint8_t gauge_posx = 5;
  if (gauge_value != shortseek_previousgauge) {
    for (uint8_t gauge_ledcount = 1; gauge_ledcount <= 15; gauge_ledcount++) {
      if (gauge_value >= gauge_ledcount) {
        if (gauge_ledcount <= 5) {
          LCD_DISPLAY.fillRect(gauge_posx, 109, 10, 18, TFT_RED);
        }
        else {
          if (gauge_ledcount > 5 and gauge_ledcount <= 10) {
            LCD_DISPLAY.fillRect(gauge_posx, 109, 10, 18, TFT_ORANGE);
          }
          else {
            if (gauge_ledcount > 10) {
              LCD_DISPLAY.fillRect(gauge_posx, 109, 10, 18, TFT_GREEN);
            }
          }
        }
      }
      else {
        // draw blanc
        LCD_DISPLAY.fillRect(gauge_posx, 109, 10, 18, TFT_BLACK);
      }
      gauge_posx = gauge_posx + 10;
    }
  }
  shortseek_previousgauge = gauge_value;
}


// 3 color gauge
void ShowGaugeSingleLed(uint8_t gauge_value) {
  uint8_t gauge_posx = 0;
  if (gauge_value != shortseek_previousgauge) {
    if (gauge_value < 1 or gauge_value > 15) {
      LCD_DISPLAY.fillRect(5, 109, 150, 18, TFT_BLACK);
    }
    else {
      LCD_DISPLAY.fillRect(5, 109, 150, 18, TFT_BLACK);
      gauge_posx = ((gauge_value - 1) * 10) + 5;
      if (gauge_value <= 5) {
        LCD_DISPLAY.fillRect(gauge_posx, 109, 10, 18, TFT_RED);
      }
      else {
        if (gauge_value > 5 and gauge_value <= 10) {
          LCD_DISPLAY.fillRect(gauge_posx, 109, 10, 18, TFT_ORANGE);
        }
        else {
          if (gauge_value > 10) {
            LCD_DISPLAY.fillRect(gauge_posx, 109, 10, 18, TFT_GREEN);
          }
        }
      }
    }
  }
  shortseek_previousgauge = gauge_value;
}

// volume bar
void ShowVolume(uint8_t volume) {
  // volume max 5 steps
  // 0 = mute

  switch (volume) {
    case 0:
      // mute
      LCD_DISPLAY.fillRect(84, 38, 75, 10, TFT_BLACK);
      break;

    case 1:
      LCD_DISPLAY.fillRect(84, 38, 19, 10, TFT_WHITE);
      LCD_DISPLAY.fillRect(103, 38, 54, 10, TFT_BLACK);
      break;

    case 2:
      LCD_DISPLAY.fillRect(84, 38, 38, 10, TFT_WHITE);
      LCD_DISPLAY.fillRect(122, 38, 36, 10, TFT_BLACK);
      break;

    case 3:
      LCD_DISPLAY.fillRect(84, 38, 57, 10, TFT_WHITE);
      LCD_DISPLAY.fillRect(141, 38, 18, 10, TFT_BLACK);
      break;

    case 4:
      LCD_DISPLAY.fillRect(84, 38, 75, 10, TFT_WHITE);
      break;

  }
}

// sensitivity bar
void ShowSensitivity(uint8_t sens_value) {
  uint8_t sense_posx = 84;
  for (uint8_t sens_barcount = 1; sens_barcount <= 75; sens_barcount++) {
    if (sens_value >= sens_barcount) {
      LCD_DISPLAY.fillRect(sense_posx, 88, 1, 10, TFT_WHITE);
    }
    else {
      // draw blanc
      LCD_DISPLAY.fillRect(sense_posx, 88, 1, 10, TFT_BLACK);
    }
    sense_posx++;
  }
}

void ShowOhm(float ohm, uint8_t range) {
  if (ohm != shortseek_previousohm) {
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.setTextPadding(58);
    switch (range) {
      case 0:
        // 200 mOhm
        if (ohm > 0.21) {
          LCD_DISPLAY.drawString(">range", 0, 2);
        }
        else {
          LCD_DISPLAY.drawFloat(ohm, 4, 0, 2);
        }
        break;

      case 1:
        // 2 Ohm
        if (ohm > 2.1) {
          LCD_DISPLAY.drawString(">range", 0, 2);
        }
        else {
          LCD_DISPLAY.drawFloat(ohm, 3, 0, 2);
        }
        break;

      case 2:
        // 20 Ohm
        if (ohm > 21) {
          LCD_DISPLAY.drawString(">range", 0, 2);
        }
        else {
          LCD_DISPLAY.drawFloat(ohm, 2, 0, 2);
        }
        break;

      case 3:
        // 200 Ohm
        if (ohm > 210) {
          LCD_DISPLAY.drawString(">range", 0, 2);
        }
        else {
          LCD_DISPLAY.drawFloat(ohm, 2, 0, 2);
        }
        break;
    }
    LCD_DISPLAY.setTextPadding(0);
    draw20PixBitmap(60, 0, OmegaBitmap, 20, 20, TFT_WHITE);

    shortseek_previousohm = ohm;
  }
}

// Function to draw the bitmap on the screen
void draw20PixBitmap(int16_t x, int16_t y, const uint8_t bitmap[][20], int16_t w, int16_t h, uint16_t color) {
  for (int16_t j = 0; j < h; j++) {
    for (int16_t i = 0; i < w; i++) {
      if (bitmap[j][i] > 0) { // Assuming non-zero value means pixel should be drawn
        LCD_DISPLAY.drawPixel(x + i, y + j, color);
      }
    }
  }
}

// splash screen
void DisplaySplash(void ) {
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.drawString("Short Seeker", 80, 0);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.drawString("Jef Collin 2024", 80, 25);
  LCD_DISPLAY.drawString("V 1.0", 80, 45);
  // depending on startup mode, sleep or not
  if (shortseek_normalstart) {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Press R short: reset gauge", 0, 61);
    LCD_DISPLAY.drawString("Press R long: menu", 0, 71);
    LCD_DISPLAY.drawString("Press L short: range/sens", 0, 81);
    LCD_DISPLAY.drawString("Press L long: set 0", 0, 91);
    LCD_DISPLAY.drawString("Turn R: volume", 0, 101);
    LCD_DISPLAY.drawString("Turn L: range/sens", 0, 111);
  }
  else {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Going to sleep", 80, 85, 2);
    LCD_DISPLAY.drawString("Press knob R to wakeup", 80, 105, 2);
  }
}

// draw the menu
void DrawMenu() {

  if (shortseek_manualmode) {
    menuItems[1] = "Automatic mode";
  }
  else {
    menuItems[1] = "Manual mode";
  }

  if (shortseek_singleLED) {
    menuItems[2] = "LED bar";
  }
  else {
    menuItems[2] = "Single LED";
  }

  LCD_SPRITE2.fillSprite(TFT_BLACK);
  LCD_SPRITE2.setTextDatum(TL_DATUM);
  LCD_SPRITE2.setFreeFont(FSS9);
  LCD_SPRITE2.setTextSize(1);
  LCD_SPRITE2.setTextColor(TFT_WHITE, TFT_BLACK);
  for (int i = 0; i < 4; i++) {
    int menuIndex = menuOffset + i;
    if (menuIndex >= numMenuItems) break; // Avoid drawing outside menu array
    if (menuIndex == selectedMenu) {
      LCD_SPRITE2.fillRect(0, 0 + (i * 32), 160, 32, TFT_WHITE);
      LCD_SPRITE2.setTextColor(TFT_BLACK, TFT_WHITE); // Highlight selected line
    } else {
      LCD_SPRITE2.setTextColor(TFT_WHITE, TFT_BLACK);
    }
    LCD_SPRITE2.drawString(menuItems[i], 10, 9 + (i * 32), 1);
    ForceNewMenu = false;
  }
   LCD_SPRITE2.pushSprite(0, 0);
}

void GoToSleep (void) {
  // shut down sound
  DAC_Stop();
  // reset all relays
  // set current
  ShortseekSetCurrent(0);

  // set gain
  ShortseekSetGain(0);

  // shutdown the backlight
  digitalWrite(Backlight_control, HIGH);
  // lcd chip to sleep
  LCD_DISPLAY.writecommand(ST7735_SLPIN);
  // Set the wakeup source to the button pin (GPIO 34)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0); // 0 means wake up when the pin is LOW (button pressed)
  // Enter deep sleep mode
  esp_deep_sleep_start();
}

// set current source max current
void ShortseekSetCurrent(uint8_t current) {
  // reset all relays first
  digitalWrite(shortseek_relay_current1, LOW);
  digitalWrite(shortseek_relay_current2, LOW);
  switch (current) {
    case 0:
      // 1mA
      shortseek_current = 1;
      break;

    case 1:
      // 5mA
      digitalWrite(shortseek_relay_current1, HIGH);
      shortseek_current = 5;
      break;

    case 2:
      // 50mA
      digitalWrite(shortseek_relay_current1, HIGH);
      digitalWrite(shortseek_relay_current2, HIGH);
      shortseek_current = 50;
      break;
  }
}

// set amplifier gain
void ShortseekSetGain(uint8_t gain) {
  // reset all relays first
  digitalWrite(shortseek_relay_gain1, LOW);
  digitalWrite(shortseek_relay_gain2, LOW);
  switch (gain) {
    case 0:
      // x1
      shortseek_gain = 1;
      break;

    case 1:
      // x10
      digitalWrite(shortseek_relay_gain1, HIGH);
      shortseek_gain = 10;
      break;

    case 2:
      // x20
      digitalWrite(shortseek_relay_gain1, HIGH);
      digitalWrite(shortseek_relay_gain2, HIGH);
      shortseek_gain = 20;
      break;
  }
}

// set range
void ShortseekSetRange(uint8_t range) {
  switch (range) {
    case 0:
      // 200 mOhm
      ShortseekSetCurrent(2);
      ShortseekSetGain(2);
      break;

    case 1:
      // 2 Ohm
      ShortseekSetCurrent(1);
      ShortseekSetGain(2);
      break;

    case 2:
      // 20 Ohm
      ShortseekSetCurrent(0);
      ShortseekSetGain(1);
      break;

    case 3:
      // 200 Ohm
      ShortseekSetCurrent(0);
      ShortseekSetGain(0);
      break;

  }
  // allow relays to settle
  delay(200);
}

// get adc voltage for 1 channel
float ShortseekReadVoltage(void) {

  int16_t ADC_CH0;
  float Voltage_CH0;
  float f = ADS.toVoltage(1);  // voltage factor
  ADC_CH0 = ADS.readADC(0);

  // convert to microvolts, round and convert back to volts
  Voltage_CH0 = round(ADC_CH0 * f * 1000000);

  Voltage_CH0 = Voltage_CH0 / 1000000;

  return Voltage_CH0;
}

// calculate resistor based on voltage
float ShortseekCalculateOhm(float measured_voltage)
{
  float calc_volt;
  float calc_ohm;

  switch (shortseek_range) {
    case 0:
      // 200 mOhm
      calc_volt = (measured_voltage - shortseek_zero_range_0) / 20;
      calc_ohm = calc_volt / 0.05;
      calc_ohm = round((calc_ohm * 10000)) / 10000;
      if (calc_ohm > 0.2) {
        calc_ohm =  0.211;
      }
      break;

    case 1:
      // 2 Ohm
      calc_volt = (measured_voltage - shortseek_zero_range_1) / 20;
      calc_ohm = calc_volt / 0.005;
      calc_ohm = round((calc_ohm * 1000)) / 1000;
      if (calc_ohm > 2.0) {
        calc_ohm =  2.11;
      }
      break;

    case 2:
      // 20 Ohm
      calc_volt = (measured_voltage - shortseek_zero_range_2) / 10;
      calc_ohm = calc_volt / 0.001;
      calc_ohm = round((calc_ohm * 100)) / 100;
      if (calc_ohm > 20) {
        calc_ohm =  21.1;
      }
      break;

    case 3:
      // 200 Ohm
      calc_ohm = (measured_voltage - shortseek_zero_range_3) / 0.001;
      calc_ohm = round((calc_ohm * 10)) / 10;
      if (calc_ohm > 200) {
        calc_ohm =  211;
      }
      break;
  }
  if (calc_ohm < 0) {
    calc_ohm = 0;
  }
  return calc_ohm;
}

// volume scaling
void ShortseekSetVolume(uint8_t volume) {
  // check if previously muted
  if (volume > 0 and shortseek_mute) {
    DAC_Start();
    shortseek_mute = false;
    BuildScreen(1);
  }

  switch (volume) {
    case 0:
      // mute
      shortseek_mute = true;
      BuildScreen(1);
      DAC_Stop();
      break;

    case 1:
      DAC_SetScale(3);
      break;

    case 2:
      DAC_SetScale(2);
      break;

    case 3:
      DAC_SetScale(1);
      break;

    case 4:
      DAC_SetScale(0);
      break;

  }
}

float ShortseekSetSensitivity(float sens, uint8_t range) {
  float window = 0;

  switch (range) {
    case 0:
      // 200 mOhm
      window = ((100.0 - sens) * 0.001);
      break;

    case 1:
      // 2 Ohm
      window = ((100.0 - sens) * 0.01);
      break;

    case 2:
      // 20 Ohm
      window = ((100.0 - sens) * 0.1);
      break;

    case 3:
      // 200 Ohm
      window = (100.0 - sens);
      break;
  }
  return window;
}

uint8_t ShortseekCalcGauge(float ohm, float reference, float window, uint8_t range) {
  uint8_t gauge = 0;

  // gauge 0-16
  // 3x5 values
  // 0 = too low
  // 16 = too high
  // step size between windows
  float windows_step = window / 15;

  float window8 = reference - (windows_step / 2);

  float window7 = window8 - windows_step;
  float window6 = window7 - windows_step;
  float window5 = window6 - windows_step;
  float window4 = window5 - windows_step;
  float window3 = window4 - windows_step;
  float window2 = window3 - windows_step;
  float window1 = window2 - windows_step;
  float window9 = window8 + windows_step;
  float window10 = window9 + windows_step;
  float window11 = window10 + windows_step;
  float window12 = window11 + windows_step;
  float window13 = window12 + windows_step;
  float window14 = window13 + windows_step;
  float window15 = window14 + windows_step;

  gauge = 0;

  float ohmref = ohm;

  switch (range) {
    case 0:
      // 200 mOhm
      if (ohm > 0.200) {
        ohmref =  0.211;
      }
      break;

    case 1:
      // 2 Ohm
      if (ohm > 2.0) {
        ohmref =  2.11;
      }
      break;

    case 2:
      // 20 Ohm
      if (ohm > 20) {
        ohmref =  21.1;
      }
      break;

    case 3:
      // 200 Ohm
      if (ohm > 200) {
        ohmref =  211;
      }
      break;
  }

  if (ohmref < window1) {
    // below lower limit
    gauge = 16;
  }
  else {
    if (ohmref > window15 + windows_step) {
      // above higher limit
      gauge = 0;
    }
    else {
      if (ohmref >= window1 and ohmref <= window2) {
        gauge = 15;
      }
      else {
        if (ohmref > window2 and ohmref <= window3) {
          gauge = 14;
        }
        else {
          if (ohmref > window3 and ohmref <= window4) {
            gauge = 13;
          }
          else {
            if (ohmref > window4 and ohmref <= window5) {
              gauge = 12;
            }
            else {
              if (ohmref > window5 and ohmref <= window6) {
                gauge = 11;
              }
              else {
                if (ohmref > window6 and ohmref <= window7) {
                  gauge = 10;
                }
                else {
                  if (ohmref > window7 and ohmref <= window8) {
                    gauge = 9;
                  }
                  else {
                    if (ohmref > window8 and ohmref <= window9) {
                      gauge = 8;
                    }
                    else {
                      if (ohmref > window9 and ohmref <= window10) {
                        gauge = 7;
                      }
                      else {
                        if (ohmref > window10 and ohmref <= window11) {
                          gauge = 6;
                        }
                        else {
                          if (ohmref > window11 and ohmref <= window12) {
                            gauge = 5;
                          }
                          else {
                            if (ohmref > window12 and ohmref <= window13) {
                              gauge = 4;
                            }
                            else {
                              if (ohmref > window13 and ohmref <= window14) {
                                gauge = 3;
                              }
                              else {
                                if (ohmref > window14 and ohmref <= window15) {
                                  gauge = 2;
                                }
                                else {
                                  if (ohmref > window15 and ohmref <= window15 + windows_step) {
                                    gauge = 1;
                                  }
                                }
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  return (gauge);
}


// start dac
void DAC_Start(void) {
  // Enable tone generator common to both channels
  SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);
  // Enable / connect tone tone generator on / to this channel
  SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN1_M);
  // Invert MSB, otherwise part of waveform will have inverted
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV1, 2, SENS_DAC_INV1_S);
  // lowest volume
  DAC_SetScale(3);
  // enable channel
  dac_output_enable(DAC_CHANNEL_1);
  shortseek_dac_running = true;
}

// stop dac output
void DAC_Stop(void) {
  CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN1_M);
  dac_output_disable(DAC_CHANNEL_1);
  shortseek_dac_running = false;
}

// set dac frequency
void DAC_SetFrequency(double frequency)
{
  // f = s * 127 / v
  double f, delta, delta_min = 999999999.0;
  uint16_t divi = 0, step = 1, s;
  uint8_t clk_8m_div = 0; // 0 bis 7
  for (uint8_t div = 1; div < 9; div++)
  {
    s = round(frequency * div / 127);
    if ((s > 0) && ((div == 1) || (s < 1024)))
    {
      f = 127 * s / div;
      delta = abs(f - frequency);
      if (delta < delta_min)
      {
        step = s;
        divi = div - 1;
        delta_min = delta;
      }
    }
  }
  frequency = 127 * step / (divi + 1);
  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_DIV_SEL, divi);
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_FSTEP, step, SENS_SW_FSTEP_S);
}

// set volume 0-3 max to min
void DAC_SetScale(int scale) {
  /*
     Scale output of a DAC channel using two bit pattern:
     - 00: no scale
     - 01: scale to 1/2
     - 10: scale to 1/4
     - 11: scale to 1/8
  */
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_SCALE1, scale, SENS_DAC_SCALE1_S);
}

// calculate tone frequency based on position within ohm window
unsigned long ShortseekCalcAudio(float ohm, float reference, float window, uint8_t range, double frequency_low, double frequency_high) {
  double frequency = 0;
  float ohmref = ohm;

  switch (range) {
    case 0:
      // 200 mOhm
      if (ohm > 0.200) {
        ohmref =  0.211;
      }
      break;

    case 1:
      // 2 Ohm
      if (ohm > 2.0) {
        ohmref =  2.11;
      }
      break;

    case 2:
      // 20 Ohm
      if (ohm > 20) {
        ohmref =  21.1;
      }
      break;

    case 3:
      // 200 Ohm
      if (ohm > 200) {
        ohmref =  211;
      }
      break;
  }

  // limits for ohms value
  float limit_low = reference - (window / 2);
  float limit_high = reference + (window / 2);

  if (ohmref < limit_low) {
    // out of range
    frequency = frequency_high;
    shortseek_out_of_range = true;
  }
  else {
    if (ohmref > limit_high) {
      // out of range
      frequency = frequency_low;
      shortseek_out_of_range = true;
    }
    else {
      // calculate frequency, add percentage of audio range to lowest freq, percentage is reversed relative ohm value within window
      frequency = ((frequency_high - frequency_low) * (1 - (ohmref - limit_low) / window)) + frequency_low;
      shortseek_out_of_range = false;
    }
  }
  return frequency;
}

// set current values as zero reference, probes must be shorted
void ShortseekSetZero(float* zero_range_0, float* zero_range_1, float* zero_range_2, float* zero_range_3) {

  float voltage = 0;

  uint8_t stabilise_loop = 2;
  uint8_t stabilise_avg = 5;

  ShortseekSetRange(0);
  for (uint8_t x = 1; x <= stabilise_loop; x++) {
    voltage = ShortseekReadVoltage();
  }
  voltage = 0;
  for (uint8_t x = 1; x <= stabilise_avg; x++) {
    voltage += ShortseekReadVoltage();
  }
  voltage = voltage / stabilise_avg;

  *zero_range_0 = voltage;

  ShortseekSetRange(1);
  for (uint8_t x = 1; x <= stabilise_loop; x++) {
    voltage = ShortseekReadVoltage();
  }
  voltage = 0;
  for (uint8_t x = 1; x <= stabilise_avg; x++) {
    voltage += ShortseekReadVoltage();
  }
  voltage = voltage / stabilise_avg;

  *zero_range_1 = voltage;

  ShortseekSetRange(2);
  for (uint8_t x = 1; x <= stabilise_loop; x++) {
    voltage = ShortseekReadVoltage();
  }
  voltage = 0;
  for (uint8_t x = 1; x <= stabilise_avg; x++) {
    voltage += ShortseekReadVoltage();
  }
  voltage = voltage / stabilise_avg;

  *zero_range_2 = voltage;

  ShortseekSetRange(3);
  for (uint8_t x = 1; x <= stabilise_loop; x++) {
    voltage = ShortseekReadVoltage();
  }
  voltage = 0;
  for (uint8_t x = 1; x <= stabilise_avg; x++) {
    voltage += ShortseekReadVoltage();
  }
  voltage = voltage / stabilise_avg;

  *zero_range_3 = voltage;
}

// check if out of range
boolean ShortseekCheckRange(float ohm, uint8_t range) {
  boolean InRange = false;
  switch (range) {
    case 0:
      // 200 mOhm
      if (ohm <= 0.200) {
        InRange = true;
      }
      break;

    case 1:
      // 2 Ohm
      if (ohm <= 2.0) {
        InRange = true;
      }
      break;

    case 2:
      // 20 Ohm
      if (ohm <= 20) {
        InRange = true;
      }
      break;

    case 3:
      // 200 Ohm
      if (ohm <= 200) {
        InRange = true;
      }
      break;
  }
  return InRange;
}
