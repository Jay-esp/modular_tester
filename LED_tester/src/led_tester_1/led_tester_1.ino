// Modular tester
// LED tester
// Jef Collin 2024


// revisions
// 1.0 first release

// calibration, resistor 56 ohm 5w, V measure with multimeter, I just resistor



// todo



#include <TFT_eSPI.h>
#include <SPI.h>
#include "Free_Fonts.h"
#include "esp_sleep.h"
#include <Wire.h>
#include "ADS1X15.h"
#include "DAC8560.h"
#include <Preferences.h>


// use alps or other decoder
#define Use_Alps_Encoder false

TFT_eSPI LCD_DISPLAY = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

TFT_eSprite LCD_SPRITE1 = TFT_eSprite(&LCD_DISPLAY);
TFT_eSprite LCD_SPRITE2 = TFT_eSprite(&LCD_DISPLAY);

// SW SPI pins select, data, clock
DAC8560 mydac(4, 19, 17);

// LCD
#define Backlight_control 13

Preferences preferences;

// set V and calculated R and P
float LED_R_voltage = 5;
float LED_R_series = 0;
float LED_R_wattage = 0;

float LED_current = 0;
float LED_voltage = 0;

// calibration voltage measured
float LED_voltage_calibration = 0;
// calibration sense
float LED_calibration_factor_ch0 = 1;
// voltage divider for battery voltage, default value, will be calibrated later
float LED_calibration_factor_ch1 = 6.95;
float LED_calibration_factor_ch2 = 6.95;

boolean LED_shutoff = true;

uint8_t LED_shutdown_counter = 0;

uint8_t LED_set_current = 0;


// calibration values per mA 1-150
uint16_t LED_calibration[155];
// pointer to cal value
uint8_t LED_calibration_index;

// dac set value for selected current
uint16_t LED_calibration_dac_value = 0;

// target sense voltage
float LED_calibration_target = 0;

unsigned long LED_updatetimer;

// set to measured value
float LED_sense_resistor = 5.01;

// trigger measurement
boolean LED_manual_trigger = false;

// raw voltages
float LED_adc_ch0 = 0;
float LED_adc_ch1 = 0;
float LED_adc_ch2 = 0;
float LED_adc_ch3 = 0;

// calibration mode
boolean LED_calibrate_mode = false;
uint8_t LED_calibrate_sequence = 0;

// Array of E12 base values
const float E12_BASE[] = {1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2};
const int E12_BASE_COUNT = 12;

// sleep or startup
boolean LED_normalstart = false;

// Menu variables
const int numMenuItems = 3;

// Menu variables
const char* menuItems[] = {"Return", "Go to sleep", "Calibrate"};
int selectedMenu = 0;
int menuOffset = 0;

boolean LED_autostart = false;

uint8_t ScreenMode = 0;


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
char Encoder_1_Pin1 = 34;
char Encoder_1_Pin2 = 27;
char Encoder_1_Key = 36;

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

 // Serial.begin(115200);

  // setup encoder pins
  pinMode(Encoder_1_Pin1, INPUT);
  pinMode(Encoder_1_Pin2, INPUT);
  pinMode(Encoder_1_Key, INPUT);

  pinMode(Encoder_2_Pin1, INPUT);
  pinMode(Encoder_2_Pin2, INPUT);
  pinMode(Encoder_2_Key, INPUT);

  // setup i2c interface
  Wire.begin();

  SPI.begin();

  mydac.begin();
  delay(100);
  mydac.setValue(0);
  // shutdown
  mydac.setPowerDownMode(DAC8560_POWERDOWN_1K);

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
    LED_normalstart = true;
  }

  DisplaySplash();

  if (LED_normalstart) {
    // wait until key is no longer pressed
    while (digitalRead(Encoder_1_Key) == 0) {}
  }

  if (LED_normalstart) {
    delay(4000);
  }
  else {
    delay(2500);
  }

  LCD_DISPLAY.fillScreen(TFT_BLACK);

  // goto sleep
  if (!LED_normalstart) {
    GoToSleep();
  }

  // adc setup
  ADS.begin();
  ADS.setGain(2);      //
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

  LCD_SPRITE1.createSprite(70, 128);

  LCD_SPRITE2.createSprite(160, 128);

  // Initialize preferences and load calibration data
  loadCalibration();

  BuildScreen(0);

  update_set_current();

  update_voltage_current();

  // force first read
  LED_updatetimer = millis() - 1000;

}

void loop() {

  if (LED_calibrate_mode) {
    // calibration mode

    // 0 main, offset trimmer calibration

    // key 1 short next, long abort
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
        BuildScreen(110);
        delay(2000);
        GoToSleep();
      }
      else {
        // short press next step
        if (LED_calibrate_sequence < 4) {
          LED_calibrate_sequence++;
        }

        switch (LED_calibrate_sequence) {
          case 1:
            // battery voltage calibration
            BuildScreen(101);
            // dac is still turned off so no need to do anything to disable the load
            break;

          case 2:
            // battery voltage calibration
            BuildScreen(102);
            // dac is still turned off so no need to do anything to disable the load
            break;

          case 3:
            // current calibration
            // dac power on
            mydac.setPowerDownMode(DAC8560_POWERDOWN_NORMAL);
            mydac.setValue(0);
            BuildScreen(103);
            delay(200);
            mydac.setValue(0);
            delay(200);
            LED_calibration_dac_value = 0;
            LED_calibration[0] = 0;

            // calibration values for 1-150mA are 16 bit DAC settings
            for (LED_calibration_index = 1; LED_calibration_index <= 150; LED_calibration_index++) {
              update_calibration_current1();
              // sense voltage
              LED_calibration_target = ((float) LED_calibration_index / 1000) * LED_sense_resistor;
              LED_adc_ch0 = 0;
              while (LED_adc_ch0 < LED_calibration_target and LED_calibration_dac_value < 65500) {
                mydac.setValue(LED_calibration_dac_value);
                LED_adc_ch0 = LED_getvoltage(0);
                LED_adc_ch0 = LED_adc_ch0 + LED_getvoltage(0);
                LED_adc_ch0 = LED_adc_ch0 / 2;
                LED_calibration_dac_value++;
              }

              if (LED_calibration_dac_value > 0 and LED_calibration_dac_value < 65000) {
                // store for current
                LED_calibration[LED_calibration_index] = LED_calibration_dac_value - 1;
                //Serial.println(LED_calibration_index);
                //Serial.println(LED_calibration[LED_calibration_index]);
              }
              else {
                LED_calibration[LED_calibration_index] = 0;
                // fail
                //Serial.println("Fail");
                //Serial.println(LED_calibration_index);
              }
            }

            // Save updated calibration data
            saveCalibration();
            BuildScreen(120);
            delay(5000);
            GoToSleep();
            break;

        }
        LED_updatetimer = millis();
      }
    }

    if (EncoderCounter1 != 0) {
      if (EncoderCounter1 > 0) {
        switch (LED_calibrate_sequence) {
          case 1:
            // battery voltage calibration
            if (LED_calibration_factor_ch2 < 8) {
              LED_calibration_factor_ch2 = LED_calibration_factor_ch2 + 0.001;
            }
            break;

          case 2:
            // battery voltage calibration
            if (LED_calibration_factor_ch1 < 8) {
              LED_calibration_factor_ch1 = LED_calibration_factor_ch1 + 0.001;
            }
            break;

          case 3:
            // current calibration
            break;

        }

        EncoderCounter1--;
      }
      else {
        switch (LED_calibrate_sequence) {
          case 1:
            // battery voltage calibration
            if (LED_calibration_factor_ch2 > 5) {
              LED_calibration_factor_ch2 = LED_calibration_factor_ch2 - 0.001;
            }
            break;

          case 2:
            // battery voltage calibration
            if (LED_calibration_factor_ch1 > 5) {
              LED_calibration_factor_ch1 = LED_calibration_factor_ch1 - 0.001;
            }
            break;

          case 3:
            // current calibration
            break;

        }
        EncoderCounter1++;
      }
    }

    // actions per mode
    switch (LED_calibrate_sequence) {
      case 0:
        break;

      case 1:
        if (millis() - LED_updatetimer > 100) {
          LED_adc_ch2 = LED_getvoltage(2);
          LED_voltage_calibration = LED_adc_ch2 * LED_calibration_factor_ch2;
          update_calibration_voltage1();
          LED_updatetimer = millis();
        }
        break;


      case 2:
        if (millis() - LED_updatetimer > 100) {
          LED_adc_ch1 = LED_getvoltage(1);
          LED_voltage_calibration = LED_adc_ch1 * LED_calibration_factor_ch1;
          update_calibration_voltage1();
          LED_updatetimer = millis();
        }
        break;

    }
  }
  else {

    // normal mode

    if (ScreenMode == 0 or ScreenMode == 1) {
      // key 1 short mode, long menu
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
          ScreenMode = 10;
          selectedMenu = 0;
          menuOffset = 0;
          EncoderCounter1 = 0;
          DrawMenu();
          while (digitalRead(Encoder_1_Key) == 0) {}
          //little debounce
          delay(200);
        }
        else {
          // short press, toggle mode
          if (ScreenMode == 0) {
            ScreenMode = 1;
            BuildScreen(ScreenMode);
            update_all_screen1();

          }
          else {
            ScreenMode = 0;
            BuildScreen(ScreenMode);
            update_set_current();
            update_voltage_current();
          }
        }
      }

      // adjust current
      if (EncoderCounter1 != 0) {
        if (EncoderCounter1 > 0) {
          if (LED_set_current < 150) {
            LED_set_current++;
            LED_setcurrent(LED_set_current);
            if (ScreenMode == 0) {
              update_set_current();
            }
            else {
            }
          }
          EncoderCounter1--;
        }
        else {
          if (LED_set_current > 0) {
            LED_set_current--;
            LED_setcurrent(LED_set_current);
            if (ScreenMode == 0) {
              update_set_current();
            }
            else {
            }
          }
          EncoderCounter1++;
        }
      }
    }

    // adjust voltage for R calculation
    if (ScreenMode == 1) {
      if (EncoderCounter2 != 0) {
        if (EncoderCounter2 > 0) {
          if (LED_R_voltage < 30.0) {
            LED_R_voltage = LED_R_voltage + 0.1;
            update_all_screen1();
          }
          EncoderCounter2--;
        }
        else {
          // strange thing to do but needed for float rounding errors
          if (LED_R_voltage >= 2.1) {
            LED_R_voltage = LED_R_voltage - 0.1;
            update_all_screen1();
          }
          EncoderCounter2++;
        }
      }
    }


    switch (ScreenMode) {
      case 0:
        // running mode
        break;
      case 1:
        break;

      case 10:
        // menu mode
        // check for key pressed
        if (digitalRead(Encoder_1_Key) == 0) {
          switch (selectedMenu) {
            case 0:
              // return
              // rebuild all
              ScreenMode = 0;
              BuildScreen(ScreenMode);
              update_set_current();
              update_voltage_current();
              break;

            case 1:
              // sleep
              LCD_DISPLAY.fillScreen(TFT_BLACK);
              // wait until key is no longer pressed
              while (digitalRead(Encoder_1_Key) == 0) {}
              // debounce to avoid early wakeup
              delay(100);
              GoToSleep();
              break;

            case 2:
              // go to calibrate mode
              // set dac to 0
              mydac.setValue(0);
              LED_calibrate_mode = true;
              LED_calibrate_sequence = 0;
              BuildScreen(100);
              // turn the dac off for offset check and voltage calibration
              mydac.setPowerDownMode(DAC8560_POWERDOWN_1K);
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

    if (ScreenMode == 0 or ScreenMode == 1) {
      if (millis() - LED_updatetimer > 100) {
        LED_adc_ch0 = LED_getvoltage(0);
        LED_adc_ch1 = LED_getvoltage(1);
        LED_adc_ch2 = LED_getvoltage(2);

        LED_adc_ch1 = LED_adc_ch1 * LED_calibration_factor_ch1;
        LED_adc_ch2 = LED_adc_ch2 * LED_calibration_factor_ch2;

        LED_voltage = LED_adc_ch2 - LED_adc_ch1;

        LED_current = (LED_adc_ch0 * 1000) / LED_sense_resistor;
        LED_current = round(LED_current);

        if (ScreenMode == 0) {
          update_voltage_current();
        }
        else {
          calc_series_resistor();
          update_all_screen1();
        }

        // check voltage for open circuit, allow ca 7v max
        if (LED_adc_ch1 < 4) {
          // check consecutive disconnects
          if (LED_shutdown_counter > 4) {
            if (!LED_shutoff) {
              LED_set_current = 0;
              LED_setcurrent(LED_set_current);
              if (ScreenMode == 0) {
                update_set_current();
              }
              else {
                calc_series_resistor();
                update_all_screen1();
              }
            }
          }
          else {
            LED_shutdown_counter++;
          }
        }
        else {
          // reset counter
          LED_shutdown_counter = 0;
        }
        LED_updatetimer = millis();
      }
    }

  }

}







void calc_series_resistor(void) {
  if (LED_voltage <= 9 and LED_R_voltage > LED_voltage and LED_current > 0) {
    LED_R_series = (LED_R_voltage - LED_voltage) / (LED_current / 1000);
    float closest = findClosestE12(LED_R_series);
    if (closest != -1) {
      LED_R_series = closest;
    }
    else {
      LED_R_series = 0;
    }
    LED_R_wattage = (LED_R_voltage - LED_voltage) * (LED_current);
    LED_R_wattage = round(LED_R_wattage);
  }
  else {
    LED_R_series = 0;
    LED_R_wattage = 0;
  }
}


// build screen
void BuildScreen(uint8_t updatemode) {
  if (updatemode == 0) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TR_DATUM);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.drawString("mA", 159, 5);
    LCD_DISPLAY.drawString("mA", 159, 56, 4);
    LCD_DISPLAY.drawString("V", 159, 108, 4);
  }

  if (updatemode == 1) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.drawString("V+", 0, 0);
    LCD_DISPLAY.drawString("I L", 0, 25);
    LCD_DISPLAY.drawString("V L", 0, 50);
    LCD_DISPLAY.drawString("R", 0, 75);
    LCD_DISPLAY.drawString("P", 0, 100);
    LCD_DISPLAY.setTextDatum(TR_DATUM);
    LCD_DISPLAY.drawString("V", 159, 0);
    LCD_DISPLAY.drawString("mA", 159, 25);
    LCD_DISPLAY.drawString("V", 159, 50);
    LCD_DISPLAY.drawString("Ohm", 159, 75);
    LCD_DISPLAY.drawString("mW", 159, 100);
  }


  // calibration main screen
  if (updatemode == 100) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Calibration step 1", 0, 0);
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Offset check", 0, 40);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.drawString("Press R knob short: next", 0, 100);
    LCD_DISPLAY.drawString("Press R knob long: abort", 0, 115);
  }

  if (updatemode == 101) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Calibration step 2", 0, 0);
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("LED anode voltage", 0, 25);
    LCD_DISPLAY.drawString("V", 145, 66, 4);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.drawString("Press R knob short: next", 0, 100);
    LCD_DISPLAY.drawString("Press R knob long: abort", 0, 115);
  }

  if (updatemode == 102) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Calibration step 3", 0, 0);
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("LED cathode voltage", 0, 25);
    LCD_DISPLAY.drawString("V", 145, 66, 4);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.drawString("Press R knob short: next", 0, 100);
    LCD_DISPLAY.drawString("Press R knob long: abort", 0, 115);
  }

  if (updatemode == 103) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Calibration step 3", 0, 0);
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Current", 0, 25);
    LCD_DISPLAY.drawString("mA", 120, 66, 4);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.drawString("Wait...", 0, 100);
  }

  if (updatemode == 110) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Calibration aborted", 0, 0);
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Going to sleep...", 0, 40);
  }

  if (updatemode == 120) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Calibration saved", 0, 0);
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Going to sleep...", 0, 40);
  }

}


void update_set_current(void) {
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(32);
  LCD_DISPLAY.drawFloat(LED_set_current, 0, 0, 5);
  LCD_DISPLAY.setTextPadding(0);
}

void update_voltage_current(void) {
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(110);
  LCD_DISPLAY.drawFloat(LED_current, 0, 0, 38, 6);
  // overflow open leads
  if (LED_voltage > 9) {
    LCD_DISPLAY.drawString("---", 0, 90, 6);
  }
  else {
    LCD_DISPLAY.drawFloat(LED_voltage, 2, 0, 90, 6);
  }
  LCD_DISPLAY.setTextPadding(0);
}

void update_all_screen1(void) {
  LCD_SPRITE1.fillSprite(TFT_BLACK);
  LCD_SPRITE1.setTextDatum(TL_DATUM);
  LCD_SPRITE1.setFreeFont(FSS9);
  LCD_SPRITE1.setTextSize(1);
  LCD_SPRITE1.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_SPRITE1.drawFloat(LED_R_voltage, 1, 0, 0);
  LCD_SPRITE1.drawFloat(LED_current, 0, 0, 25);

  // overflow open leads
  if (LED_voltage > 9) {
    LCD_SPRITE1.drawString("---", 0, 50);
  }
  else {
    LCD_SPRITE1.drawFloat(LED_voltage, 2, 0, 50);
  }

  LCD_SPRITE1.drawString(formatResistorValue(LED_R_series), 0, 75);
  LCD_SPRITE1.drawFloat(LED_R_wattage, 0, 0, 100);
  LCD_SPRITE1.pushSprite(40, 0);
}


void update_calibration_voltage1(void) {
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(140);
  LCD_DISPLAY.drawFloat(LED_voltage_calibration, 2, 0, 48, 6);
  LCD_DISPLAY.setTextPadding(0);
}

void update_calibration_current1(void) {
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(110);
  LCD_DISPLAY.drawFloat(LED_calibration_index, 0, 0, 48, 6);
  LCD_DISPLAY.setTextPadding(0);
}


// splash screen
void DisplaySplash(void ) {
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.drawString("LED Tester", 80, 0);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.drawString("Jef Collin 2024", 80, 25);
  LCD_DISPLAY.drawString("V 1.0", 80, 45);
  // depending on startup mode, sleep or not
  if (LED_normalstart) {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Press R short: mode", 0, 65);
    LCD_DISPLAY.drawString("Press R long: menu", 0, 78);
    //  LCD_DISPLAY.drawString("Press L short: fix/var", 0, 91);
    LCD_DISPLAY.drawString("Turn L: VCC", 0, 91);
    LCD_DISPLAY.drawString("Turn R: current ", 0, 104);
  }
  else {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Going to sleep", 80, 85, 2);
    LCD_DISPLAY.drawString("Press knob R to wakeup", 80, 105, 2);
  }
}

// draw the menu
void DrawMenu() {
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
  }
  LCD_SPRITE2.pushSprite(0, 0);
}

void GoToSleep (void) {
  mydac.setValue(0);
  mydac.setPowerDownMode(DAC8560_POWERDOWN_1K);

  // shutdown the backlight
  digitalWrite(Backlight_control, HIGH);
  // lcd chip to sleep
  LCD_DISPLAY.writecommand(ST7735_SLPIN);
  // Set the wakeup source to the button pin (GPIO 36)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_36, 0); // 0 means wake up when the pin is LOW (button pressed)
  // Enter deep sleep mode
  esp_deep_sleep_start();
}

// read adc
float LED_getvoltage(uint8_t channel) {
  float voltage = 0;
  float factor = 1;
  int16_t rawvoltage = 0;
  factor = ADS.toVoltage(1);
  rawvoltage = ADS.readADC(channel);
  voltage = rawvoltage * factor * 1;
  return voltage;
}

// set calibrated current
void LED_setcurrent(uint16_t current) {
  uint16_t dacValue = 0;
  if (current == 0) {
    if (!LED_shutoff) {
      mydac.setValue(0);
      mydac.setValue(0);
      mydac.setValue(0);
      mydac.setValue(0);
      mydac.setValue(0);
      mydac.setValue(0);
      mydac.setValue(0);
      mydac.setValue(0);
      mydac.setValue(0);
      mydac.setValue(0);
      mydac.setPowerDownMode(DAC8560_POWERDOWN_1K);
      LED_shutoff = true;
    }
  }
  else {
    if (current >  0 and current <= 150) {
      dacValue = LED_calibration[current];
    }
    if (LED_shutoff) {
      mydac.setValue(0);
      mydac.setPowerDownMode(DAC8560_POWERDOWN_NORMAL);
      mydac.setValue(0);
      mydac.setValue(0);
      LED_shutoff = false;
    }
    mydac.setValue(dacValue);
  }
}

// Function to save calibration data
void saveCalibration() {
  preferences.begin("calibration", false); // Open in read-write mode
  // Save the float value
  preferences.putFloat("calibFactorCh1", LED_calibration_factor_ch1);
  preferences.putFloat("calibFactorCh2", LED_calibration_factor_ch2);

  // Save the array of uint16_t values
  for (int i = 0; i <= 150; i++) {
    String key = String("calibArray") + String(i);
    preferences.putUShort(key.c_str(), LED_calibration[i]);
  }
  preferences.end(); // Close the preferences
}

// Function to load calibration data
void loadCalibration() {
  preferences.begin("calibration", true); // Open in read-only mode
  // Load the float value
  LED_calibration_factor_ch1 = preferences.getFloat("calibFactorCh1", 6.95f); // Default to 5.127 if not found
  LED_calibration_factor_ch2 = preferences.getFloat("calibFactorCh2", 6.95f); // Default to 5.127 if not found

  // Load the array of uint16_t values
  for (int i = 0; i <= 150; i++) {
    String key = String("calibArray") + String(i);
    LED_calibration[i] = preferences.getUShort(key.c_str(), 0); // Default to 0 if not found
  }
  preferences.end(); // Close the preferences
}










// Function to find the closest E12 resistor
float findClosestE12(float resistor) {
  if (resistor < 1.0 || resistor > 10000000.0) {
    Serial.println("Error: Resistor value out of range (1 ohm to 10M ohm).");
    return -1;
  }

  float decade = pow(10, floor(log10(resistor))); // Decade multiplier
  float normalizedResistor = resistor / decade;  // Normalize to 1.0 to 10.0 range

  float closestValue = E12_BASE[0];
  float minDifference = fabs(normalizedResistor - E12_BASE[0]);
  for (int i = 1; i < E12_BASE_COUNT; ++i) {
    float difference = fabs(normalizedResistor - E12_BASE[i]);
    if (difference < minDifference) {
      closestValue = E12_BASE[i];
      minDifference = difference;
    }
  }

  return closestValue * decade; // Rescale to original range
}

// Function to format resistor value
String formatResistorValue(float resistor) {
  resistor = round(resistor); // Explicit rounding to eliminate floating-point errors

  if (resistor < 1.0) {
    return String(resistor, 2) + "R"; // Below 1 ohm, keep 2 decimals
  } else if (resistor >= 1.0 && resistor < 1000.0) {
    return String((int)resistor) + "R"; // Up to 999 ohm, drop decimals
  } else if (resistor >= 1000.0 && resistor < 1000000.0) {
    int kiloValue = (int)(resistor / 1000.0);
    float fractionalValue = fmod(resistor, 1000.0) / 100.0; // Extract fractional kilo component
    if (fractionalValue > 0) {
      return String(kiloValue) + "K" + String((int)fractionalValue); // 1K2, 1K5, etc.
    } else {
      return String(kiloValue) + "K"; // Whole kilo values, e.g., 1K, 2K
    }
  } else if (resistor >= 1000000.0) {
    resistor /= 1000000.0;
    return String((int)resistor) + "M"; // 1M and above, no decimals
  }
  return "Error"; // Fallback case
}
