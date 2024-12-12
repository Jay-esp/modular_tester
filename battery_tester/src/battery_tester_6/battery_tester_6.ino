// Modular tester
// battery tester
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

// battery voltages
float battery_voltage_loaded = 0;
float battery_voltage_unloaded = 0;

// calibration voltage measured
float battery_voltage_calibration = 0;
// calibration sense
float battery_calibration_factor_ch0 = 1;
// voltage divider for battery voltage, default value, will be calibrated later
float battery_calibration_factor_ch1 = 6.95;

uint8_t battery_measure_current;

boolean battery_shutoff = true;

// calibration values per mA 1-200
uint16_t battery_calibration[205];
// pointer to cal value
uint8_t battery_calibration_index;

// dac set value for selected current
uint16_t battery_calibration_dac_value = 0;

// target sense voltage
float battery_calibration_target = 0;

unsigned long battery_updatetimer;

// set to measured value
float battery_sense_resistor = 5.01;

// trigger measurement
boolean battery_manual_trigger = false;

// raw voltages
float battery_adc_ch0 = 0;
float battery_adc_ch1 = 0;
float battery_adc_ch2 = 0;
float battery_adc_ch3 = 0;

// calibration mode
boolean battery_calibrate_mode = false;
uint8_t battery_calibrate_sequence = 0;

// sleep or startup
boolean battery_normalstart = false;

// Menu variables
const int numMenuItems = 4;

// Menu variables
const char* menuItems[] = {"Return", "Auto start", "Go to sleep", "Calibrate"};
int selectedMenu = 0;
int menuOffset = 0;

boolean battery_autostart = true;

uint8_t ScreenMode = 0;

// all batteries
uint8_t battery_selected = 0;
uint8_t battery_selections = 21;

uint16_t battery_voltages[] = {1500, 1500, 1500, 1500, 1500, 9000, 3000, 3000, 3000, 3000, 3000, 3000, 1500, 1500, 1500, 1550, 1550, 12000, 1500, 6000, 6000};
uint16_t battery_currents[] = {100, 50, 20, 150, 150, 10, 100, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 5, 100, 1, 100};

// Define an array of battery type names
const char* battery_types[] = {
  "AA",
  "AAA",
  "AAAA",
  "C",
  "D",
  "9V / LR22",
  "CR123A",
  "CR1632",
  "CR2016",
  "CR2025",
  "CR2032",
  "CR2450",
  "LR41",
  "LR43",
  "LR44",
  "SR44",
  "SRW626SW",
  "A23",
  "N",
  "PX28",
  "2CR5"
};



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

  Serial.begin(115200);

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
    battery_normalstart = true;
  }

  DisplaySplash();

  if (battery_normalstart) {
    // wait until key is no longer pressed
    while (digitalRead(Encoder_1_Key) == 0) {}
  }

  if (battery_normalstart) {
    delay(4000);
  }
  else {
    delay(2500);
  }

  LCD_DISPLAY.fillScreen(TFT_BLACK);

  // goto sleep
  if (!battery_normalstart) {
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

  LCD_SPRITE1.createSprite(140, 40);

  LCD_SPRITE2.createSprite(160, 128);

  // Initialize preferences and load calibration data
  loadCalibration();

  BuildScreen(0);
  update_type();
  update_voltages();
  update_auto_mode(1);
  // force first read
  battery_updatetimer = millis() - 1000;

}

void loop() {

  if (battery_calibrate_mode) {
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
        if (battery_calibrate_sequence < 4) {
          battery_calibrate_sequence++;
        }

        switch (battery_calibrate_sequence) {
          case 1:
            // battery voltage calibration
            BuildScreen(101);
            // dac is still turned off so no need to do anything to disable the load
            break;

          case 2:
            // current calibration
            // dac power on
            mydac.setPowerDownMode(DAC8560_POWERDOWN_NORMAL);
            BuildScreen(102);
            delay(200);
            mydac.setValue(0);
            delay(200);
            battery_calibration_dac_value = 0;
            battery_calibration[0] = 0;

            // calibration values for 1-200mA are 16 bit DAC settings
            for (battery_calibration_index = 1; battery_calibration_index <= 200; battery_calibration_index++) {
              update_calibration_current1();
              // sense voltage
              battery_calibration_target = ((float) battery_calibration_index / 1000) * battery_sense_resistor;

              battery_adc_ch0 = 0;

              while (battery_adc_ch0 < battery_calibration_target and battery_calibration_dac_value < 65500) {
                mydac.setValue(battery_calibration_dac_value);

                battery_adc_ch0 = battery_getvoltage(0);
                battery_adc_ch0 = battery_adc_ch0 + battery_getvoltage(0);
                battery_adc_ch0 = battery_adc_ch0 / 2;

                battery_calibration_dac_value++;

              }

              if (battery_calibration_dac_value > 0 and battery_calibration_dac_value < 65000) {
                // store for current
                battery_calibration[battery_calibration_index] = battery_calibration_dac_value - 1;
              }
              else {
                battery_calibration[battery_calibration_index] = 0;
                // fail
                //                Serial.println("Fail");
                //                Serial.println(battery_calibration_index);
              }
            }

            // Save updated calibration data
            saveCalibration();
            BuildScreen(120);
            delay(5000);
            GoToSleep();
            break;

        }
        battery_updatetimer = millis();
      }
    }

    if (EncoderCounter1 != 0) {
      if (EncoderCounter1 > 0) {
        switch (battery_calibrate_sequence) {
          case 1:
            // battery voltage calibration
            if (battery_calibration_factor_ch1 < 9) {
              battery_calibration_factor_ch1 = battery_calibration_factor_ch1 + 0.001;
            }
            break;

          case 2:
            // current calibration
            break;

        }


        EncoderCounter1--;
      }
      else {
        switch (battery_calibrate_sequence) {
          case 1:
            // battery voltage calibration
            if (battery_calibration_factor_ch1 > 4) {
              battery_calibration_factor_ch1 = battery_calibration_factor_ch1 - 0.001;
            }
            break;

          case 2:
            // current calibration
            break;

        }
        EncoderCounter1++;
      }
    }

    // actions per mode
    switch (battery_calibrate_sequence) {
      case 0:
        break;

      case 1:
        if (millis() - battery_updatetimer > 100) {
          battery_adc_ch1 = battery_getvoltage(1);
          battery_voltage_calibration = battery_adc_ch1 * battery_calibration_factor_ch1;
          update_calibration_voltage1();
          battery_updatetimer = millis();
        }
        break;
    }
  }
  else {

    // normal mode



    if (EncoderCounter2 != 0) {
      if (EncoderCounter2 > 0) {
        if (ScreenMode == 0) {
          mydac.setValue(0);
          battery_measure_current = 0;
          ScreenMode = 2;
          BuildScreen(ScreenMode);
          update_bat_current();
          battery_updatetimer = millis();

        }
        else {
          if (ScreenMode == 2) {

          }
        }
        EncoderCounter2--;
      }
      else {
        if (ScreenMode == 0) {

        }
        else {
          if (ScreenMode == 2) {
            ScreenMode = 0;
            BuildScreen(ScreenMode);
            update_type();
            battery_voltage_loaded = 0;
            battery_voltage_unloaded = 0;
            update_voltages();
            if (battery_autostart) {
              update_auto_mode(1);
            }
            else {
              update_auto_mode(0);
            }
          }
        }
        EncoderCounter2++;
      }
    }





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
            battery_manual_trigger = true;
          }
        }

        if (EncoderCounter1 != 0) {
          if (EncoderCounter1 > 0) {
            if (battery_selected < (battery_selections - 1)) {
              battery_selected++;
              update_type();
              if (battery_voltage_loaded != 0 or battery_voltage_unloaded != 0) {
                battery_voltage_loaded = 0;
                battery_voltage_unloaded = 0;
                update_voltages();
              }
            }
            EncoderCounter1--;
          }
          else {
            if (battery_selected > 0) {
              battery_selected--;
              update_type();
              if (battery_voltage_loaded != 0 or battery_voltage_unloaded != 0) {
                battery_voltage_loaded = 0;
                battery_voltage_unloaded = 0;
                update_voltages();
              }
            }
            EncoderCounter1++;
          }
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
              update_type();
              battery_voltage_loaded = 0;
              battery_voltage_unloaded = 0;
              update_voltages();
              if (battery_autostart) {
                update_auto_mode(1);
              }
              else {
                update_auto_mode(0);
              }
              break;

            case 1:
              // toggle manual/automatic
              battery_autostart = !battery_autostart;
              DrawMenu();
              break;

            case 2:
              // sleep
              LCD_DISPLAY.fillScreen(TFT_BLACK);
              // wait until key is no longer pressed
              while (digitalRead(Encoder_1_Key) == 0) {}
              // debounce to avoid early wakeup
              delay(100);
              GoToSleep();
              break;

            case 3:
              // go to calibrate mode
              // set dac to 0
              mydac.setValue(0);
              battery_calibrate_mode = true;
              battery_calibrate_sequence = 0;
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




      case 2:
        // voltage measure mode

        if (EncoderCounter1 != 0) {
          if (EncoderCounter1 > 0) {
            if (battery_measure_current < 200) {
              battery_measure_current++;
              battery_setcurrent(battery_measure_current);
              update_bat_current();
            }
            EncoderCounter1--;
          }
          else {
            if (battery_measure_current > 0) {
              battery_measure_current--;
              if (battery_measure_current == 0) {
                mydac.setValue(0);
              }
              else {
                battery_setcurrent(battery_measure_current);
              }
              update_bat_current();
            }
            EncoderCounter1++;
          }
        }
        break;

    }

    if (ScreenMode == 0) {
      if (battery_autostart and !battery_manual_trigger) {
        mydac.setValue(0);
        delay(100);
        battery_adc_ch1 = battery_getvoltage(1);
        battery_voltage_unloaded = battery_adc_ch1 * battery_calibration_factor_ch1;
        if (battery_voltage_unloaded > 0.5) {
          delay(100);
          battery_adc_ch1 = battery_getvoltage(1);
          battery_voltage_unloaded = battery_adc_ch1 * battery_calibration_factor_ch1;
          if (battery_voltage_unloaded > 0.5) {
            battery_manual_trigger = true;
            update_auto_mode(2);
            // add a delay to stabilise and contact changes
            delay(100);
          }
        }
      }

      if (battery_manual_trigger) {
        battery_voltage_unloaded = 0;
        battery_voltage_loaded = 0;
        update_voltages();
        // check for negative voltage
        battery_adc_ch1 = battery_getvoltage(1);
        battery_voltage_unloaded = battery_adc_ch1 * battery_calibration_factor_ch1;
        if (battery_voltage_unloaded < 0) {
          // negative voltage
          // abort
          battery_manual_trigger = false;
        }
        else {
          if (!battery_autostart) {
            update_auto_mode(4);
          }
          // disable current
          mydac.setValue(0);
          delay(200);
          battery_adc_ch0 = battery_getvoltage(0);
          battery_adc_ch1 = battery_getvoltage(1);
          battery_adc_ch2 = battery_getvoltage(2);
          battery_adc_ch3 = battery_getvoltage(3);
          battery_voltage_unloaded = battery_adc_ch1 * battery_calibration_factor_ch1;
          battery_setcurrent(battery_currents[battery_selected]);
          delay(2000);
          battery_adc_ch1 = battery_getvoltage(1);
          battery_voltage_loaded = battery_adc_ch1 * battery_calibration_factor_ch1;
          mydac.setValue(0);
          update_voltages();
          battery_manual_trigger = false;
          if (battery_autostart) {
            update_auto_mode(3);
            while (battery_voltage_unloaded  > 0.5) {
              battery_adc_ch1 = battery_getvoltage(1);
              battery_voltage_unloaded = battery_adc_ch1 * battery_calibration_factor_ch1;
            }
            delay(300);
            // reset just in case
            EncoderCounter1 = 0;
            EncoderCounter2 = 0;
            update_auto_mode(1);
          }
          else {
            update_auto_mode(0);
          }
        }
      }
    }
    else {
      if (ScreenMode == 2) {
        if (millis() - battery_updatetimer > 100) {
          battery_adc_ch1 = battery_getvoltage(1);
          battery_voltage_unloaded = battery_adc_ch1 * battery_calibration_factor_ch1;
          // check for negative voltage
          if (battery_voltage_unloaded < 0) {
            battery_voltage_unloaded = 0;
          }
          update_bat_voltage();
          battery_updatetimer = millis();
        }
      }
    }
  }


}

// build screen
void BuildScreen(uint8_t updatemode) {
  if (updatemode == 0) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.drawString("V", 145, 66, 4);
    LCD_DISPLAY.drawString("V", 145, 108, 4);
  }


  if (updatemode == 2) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.drawString("Battery Voltage", 2, 0);
    LCD_DISPLAY.drawString("V", 145, 66, 4);

    LCD_DISPLAY.drawString("mA", 40, 114);

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
    LCD_DISPLAY.drawString("Bat voltage", 0, 25);
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
    LCD_DISPLAY.drawString("Current", 0, 25);
    LCD_DISPLAY.drawString("mA", 120, 66, 4);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.drawString("Wait...", 0, 100);
  }

  if (updatemode == 103) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Calibration step 4", 0, 0);
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Sense voltage", 0, 25);
    LCD_DISPLAY.drawString("V", 145, 66, 4);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.drawString("Press R knob short: next", 0, 100);
    LCD_DISPLAY.drawString("Press R knob long: abort", 0, 115);
  }

  if (updatemode == 104) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Calibration step 5", 0, 0);
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("I test", 0, 25);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.drawString("Press R knob short: next", 0, 100);
    LCD_DISPLAY.drawString("Press R knob long: abort", 0, 115);
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


void update_type(void) {
  LCD_SPRITE1.fillSprite(TFT_BLACK);
  LCD_SPRITE1.setTextDatum(TL_DATUM);
  LCD_SPRITE1.setFreeFont(FSS12);
  LCD_SPRITE1.setTextSize(1);
  LCD_SPRITE1.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_SPRITE1.drawString(battery_types[battery_selected], 0, 2);
  LCD_SPRITE1.setFreeFont(FSS9);
  LCD_SPRITE1.setTextColor(TFT_GREEN, TFT_BLACK);
  float b_voltage = (float) battery_voltages[battery_selected] / 1000;
  LCD_SPRITE1.drawFloat(b_voltage, 2, 0, 26);
  LCD_SPRITE1.drawNumber(battery_currents[battery_selected], 70, 26);
  LCD_SPRITE1.drawString("V", 45, 26);
  LCD_SPRITE1.drawString("mA", 110, 26);
  LCD_SPRITE1.pushSprite(0, 0);
}

void update_voltages(void) {
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(140);
  LCD_DISPLAY.drawFloat(battery_voltage_unloaded, 2, 0, 48, 6);
  LCD_DISPLAY.drawFloat(battery_voltage_loaded, 2, 0, 90, 6);
  LCD_DISPLAY.setTextPadding(0);
}

void update_bat_voltage(void) {
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(140);
  LCD_DISPLAY.drawFloat(battery_voltage_unloaded, 2, 0, 48, 6);
  LCD_DISPLAY.setTextPadding(0);
}


void update_bat_current(void) {
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(32);
  LCD_DISPLAY.drawFloat(battery_measure_current, 0, 0, 114);
  LCD_DISPLAY.setTextPadding(0);
}





void update_calibration_voltage1(void) {
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(140);
  LCD_DISPLAY.drawFloat(battery_voltage_calibration, 2, 0, 48, 6);
  LCD_DISPLAY.setTextPadding(0);
}

void update_calibration_current1(void) {
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(110);
  LCD_DISPLAY.drawFloat(battery_calibration_index, 0, 0, 48, 6);
  LCD_DISPLAY.setTextPadding(0);
}

void update_auto_mode(uint8_t automode) {
  // 0 manual blue
  // 1 auto waiting for trigger blue
  // 2 auto measuring green
  // 3 auto done orange
  // 4 manual measuring green

  LCD_DISPLAY.setTextDatum(TR_DATUM);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setTextPadding(20);
  switch (automode) {
    case 0:
      LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
      LCD_DISPLAY.drawString("M", 159, 0);
      break;

    case 1:
      LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
      LCD_DISPLAY.drawString("A", 159, 0);
      break;

    case 2:
      LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
      LCD_DISPLAY.drawString("A", 159, 0);
      break;

    case 3:
      LCD_DISPLAY.setTextColor(TFT_ORANGE, TFT_BLACK);
      LCD_DISPLAY.drawString("A", 159, 0);
      break;

    case 4:
      LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
      LCD_DISPLAY.drawString("M", 159, 0);
      break;

  }
  LCD_DISPLAY.setTextPadding(0);
}

// splash screen
void DisplaySplash(void ) {
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.drawString("Battery Tester", 80, 0);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.drawString("Jef Collin 2024", 80, 25);
  LCD_DISPLAY.drawString("V 1.0", 80, 45);
  // depending on startup mode, sleep or not
  if (battery_normalstart) {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Press R short: test", 0, 71);
    LCD_DISPLAY.drawString("Press R long: menu", 0, 86);
    LCD_DISPLAY.drawString("Turn L: mode", 0, 101);
    LCD_DISPLAY.drawString("Turn R: type/current ", 0, 116);
  }
  else {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Going to sleep", 80, 85, 2);
    LCD_DISPLAY.drawString("Press knob R to wakeup", 80, 105, 2);
  }
}

// draw the menu
void DrawMenu() {
  if (battery_autostart) {
    menuItems[1] = "Manual start";
  }
  else {
    menuItems[1] = "Auto start";
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
float battery_getvoltage(uint8_t channel) {
  float voltage = 0;
  float factor = 1;
  int16_t rawvoltage = 0;
  factor = ADS.toVoltage(1);
  rawvoltage = ADS.readADC(channel);
  voltage = rawvoltage * factor * 1;
  return voltage;
}


// set calibrated current
void battery_setcurrent(uint16_t current) {
  uint16_t dacValue = 0;
  if (current == 0) {
    if (!battery_shutoff) {
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
      battery_shutoff = true;
    }
  }
  else {
    if (current >  0 and current <= 200) {
      dacValue = battery_calibration[current];
    }
    if (battery_shutoff) {
      mydac.setValue(0);
      mydac.setPowerDownMode(DAC8560_POWERDOWN_NORMAL);
      mydac.setValue(0);
      mydac.setValue(0);
      battery_shutoff = false;
    }
    mydac.setValue(dacValue);
  }
}


// Function to save calibration data
void saveCalibration() {
  preferences.begin("calibration", false); // Open in read-write mode
  // Save the float value
  preferences.putFloat("calibFactorCh1", battery_calibration_factor_ch1);
  // Save the array of uint16_t values
  for (int i = 0; i < 205; i++) {
    String key = String("calibArray") + String(i);
    preferences.putUShort(key.c_str(), battery_calibration[i]);
  }
  preferences.end(); // Close the preferences
}

// Function to load calibration data
void loadCalibration() {
  preferences.begin("calibration", true); // Open in read-only mode
  // Load the float value
  battery_calibration_factor_ch1 = preferences.getFloat("calibFactorCh1", 6.95f); // Default to 6.95 if not found
  // Load the array of uint16_t values
  for (int i = 0; i < 205; i++) {
    String key = String("calibArray") + String(i);
    battery_calibration[i] = preferences.getUShort(key.c_str(), 0); // Default to 0 if not found
  }
  preferences.end(); // Close the preferences
}
