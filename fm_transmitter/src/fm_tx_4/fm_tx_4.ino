// Modular tester
// FM transmitter module
// Jef Collin 2025


// revisions
// 1.0 first release



// todo



#include <TFT_eSPI.h>
#include <SPI.h>
#include "Free_Fonts.h"
#include "esp_sleep.h"
#include <Wire.h>
// use this instead of the adafruit library (otherwise distorted audio)
#include "si4713.h"

#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include <driver/i2s.h>
#include "soc/syscon_reg.h"
#include "driver/adc.h"
#include <driver/dac_oneshot.h>
#include <driver/dac_cosine.h>


#include "DSEG7ClassicMini_Regular15pt7b.h"



// use alps or other decoder
#define Use_Alps_Encoder false

TFT_eSPI LCD_DISPLAY = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

TFT_eSprite LCD_SPRITE2 = TFT_eSprite(&LCD_DISPLAY);

//  interface pins

#define fm_power1 17
#define fm_power2 16
#define fm_reset 5

// I2C
// 0x63 fm si4713

// LCD
#define Backlight_control 13

// sleep or startup
boolean fm_normalstart = false;

SI4713 fm_tx;

// fm generator
// 8000 to 10800 / 50khz steps
uint32_t fm_frequency = 10000;

// increment step size 1 = 100KHz
byte fm_stepsize = 1;

// 88 to 115 dBuV, 0 = off
// pass index to list not value
byte fm_power = 13;

// 50 for europe
// pass index to list not value
uint8_t fm_preemphasis = 0;

// mode freq setting, step setting, power setting
uint8_t fm_mode1 = 0;

// toggle sine or bluetooth mode
boolean fm_bt_on = false;

// modulation
uint8_t fm_sinemode = 0;

boolean fm_overmodulated;
boolean fm_prev_overmodulated = false;

int8_t fm_inputlevel;

unsigned long loop_timer;



// sprintf buffer for various convertions
char printbuf[30];
String strbuf;

// handle for the cosine mode on dac 2
dac_cosine_handle_t dac_chan_handle;

// Menu variables
const int numMenuItems = 3;

// Menu variables
const char* menuItems[] = {"Return", "Preemp 50uS", "Go to sleep"};
int selectedMenu = 0;
int menuOffset = 0;

byte ScreenMode = 0;
boolean ForceNewMenu = true;

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

// rotary encode io pins
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

  Wire.begin();

  // setup encoder pins
  pinMode(Encoder_1_Pin1, INPUT);
  pinMode(Encoder_1_Pin2, INPUT);
  pinMode(Encoder_1_Key, INPUT);

  pinMode(Encoder_2_Pin1, INPUT);
  pinMode(Encoder_2_Pin2, INPUT);
  pinMode(Encoder_2_Key, INPUT);

  // turn on fm module
  pinMode(fm_power1, OUTPUT);
  digitalWrite(fm_power1, HIGH);

  // turn off bt module
  pinMode(fm_power2, OUTPUT);
  digitalWrite(fm_power2, LOW);

  pinMode(Backlight_control, OUTPUT);

  // turn off backlight
  digitalWrite(Backlight_control, HIGH);

  // initialise and reset fm tx
  pinMode(fm_reset, OUTPUT);
  digitalWrite(fm_reset, HIGH);
  delay(200);
  digitalWrite(fm_reset, LOW);
  delay(200);
  digitalWrite(fm_reset, HIGH);

  // Setup the LCD
  LCD_DISPLAY.init();
  LCD_DISPLAY.setRotation(3);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  delay(100);
  // turn on backlight
  digitalWrite(Backlight_control, LOW);

  // check for key pressed, enable normal start otherwise go to sleep
  if (digitalRead(Encoder_1_Key) == 0) {
    fm_normalstart = true;
  }

  DisplaySplash();

  if (fm_normalstart) {
    // wait until key is no longer pressed
    while (digitalRead(Encoder_1_Key) == 0) {}
  }

  if (fm_normalstart) {
    delay(4000);
  }
  else {
    delay(2500);
  }

  LCD_DISPLAY.fillScreen(TFT_BLACK);

  // goto sleep
  if (!fm_normalstart) {
    GoToSleep();
  }

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

  BuildScreen(ScreenMode);

  update_frequency();
  update_mhz();
  update_step();
  update_power();
  update_modulation();

  //  byte error, address;
  //  int nDevices;
  //  while (1) {
  //    Serial.println("Scanning...");
  //
  //    nDevices = 0;
  //    for (address = 1; address < 127; address++ )
  //    {
  //      // The i2c_scanner uses the return value of
  //      // the Write.endTransmisstion to see if
  //      // a device did acknowledge to the address.
  //      Wire.beginTransmission(address);
  //      error = Wire.endTransmission();
  //
  //      if (error == 0)
  //      {
  //        Serial.print("I2C device found at address 0x");
  //        if (address < 16)
  //          Serial.print("0");
  //        Serial.print(address, HEX);
  //        Serial.println("  !");
  //
  //        nDevices++;
  //      }
  //      else if (error == 4)
  //      {
  //        Serial.print("Unknown error at address 0x");
  //        if (address < 16)
  //          Serial.print("0");
  //        Serial.println(address, HEX);
  //      }
  //    }
  //    Serial.println("Scanning end...");
  //  }
  //  delay(1000);


  fm_tx.Init(fm_reset, 32768, 0x63);             // RST pin: 11 (use -1 when using external supervisor), Crystal: 32.768kHz, I2C address: 0x63

  tx_set_power();
  tx_set_frequency();

  fm_tx.MPX_Enable(1);                     // 1 = Enable MPX Stereocoder
  fm_tx.MPX_Deviation(675);                // Set MPX pilot deviation to 6.75kHz
  fm_tx.MPX_Freq(19000);                   // Set MPX pilot to 19000Hz (19kHz)
  fm_tx.Audio_Deviation(7500);             // Set Audio deviation to 75.00kHz
  fm_tx.Audio_Mute(0);                     // Unmute audio
  tx_set_preemphasis();
  fm_tx.Audio_Limiter(0);                  // Disable Audio limiter
  fm_tx.Audio_AGC(1);                      // Enable Audio Dynamic Range Control
  fm_tx.Audio_Comp_Threshold(-40);         // Set Audio Dynamic Range Threshold to -40dBFS (max = 0dBFS)
  fm_tx.Audio_Comp_Attack(0);              // Set Audio Dynamic Range Attack time to 0.5mS (max value = 9 -> 5.0mS, stepsize is 0.5mS)
  fm_tx.Audio_Comp_Release(4);             // Set Audio Dynamic Range Release time to 1S (see p43 of application notes for other values)
  fm_tx.Audio_Comp_Gain(15);               // Set Audio Dynamic Range Gain to 15dB (max=20dB)
  fm_tx.Audio_Limiter_Release(102);        // Set Audio Limiter Release time to 5.01mS (see p44 of application notes for other values)
  fm_tx.RDS_PI(0x803A);                    // Set RDS PI code to 803A
  //  fm_tx.RDS_AF(fm_frequency);              // Set RDS AF to 90.8MHz (use 0 to disable AF)
  fm_tx.RDS_PTY(10);                       // Set RDS PTY to Pop Music (see list of codes at https://www.electronics-notes.com/articles/audio-video/broadcast-audio/rds-radio-data-system-pty-codes.php)
  fm_tx.RDS_Deviation(200);                // Set RDS deviation to 2.00kHz
  fm_tx.RDS_COMP(0);                       // Set RDS Compressed code to not Compressed
  fm_tx.RDS_ART(0);                        // Set RDS Artificial Head code to Not artificial head
  fm_tx.RDS_MS(1);                         // Set RDS Mono/Stereo code to stereo
  fm_tx.RDS_TP(1);                         // Enable RDS Traffic Program
  fm_tx.RDS_TA(0);                         // Disable RDS Traffic Announcement
  fm_tx.RDS_MUSP(1);                       // Set RDS Music/Speech selection to Music
  fm_tx.RDS_PS("Jef Collin", 0);               // Set PS Message (max 8 characters) and position number in carousel
  fm_tx.RDS_PS("RDS test", 1);
  fm_tx.RDS_PS("ABCDEFG", 2);
  fm_tx.RDS_PS("012345", 3);
  fm_tx.RDS_PSCOUNT(4, 10);                // Number of PS Messages in carousel(4), (min 1, max 12),  and carousel speed (10) (min 1, max 255);
  fm_tx.RDS_RT("RDS test message"); // RDS Radiotext message, up to 32 characters
  fm_tx.RDS_Enable(1);                     // 1 = Enable RDS encoder, 0 = Disable RDS encoder
  fm_tx.GPO(0, 0, 0);                      // Set GPO outputs 1,2 and 3 to low.

  // do this again, this seems to activate the tx
  tx_set_frequency();

  loop_timer = millis();
}

void loop() {

  if (ScreenMode == 0) {

    // key 1 short , long menu
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
        // short press

        if (fm_mode1 == 0) {
          fm_mode1 = 1;
        }
        else {
          if (fm_mode1 == 1) {
            fm_mode1 = 2;
          }
          else {
            if (fm_mode1 == 2) {
              fm_mode1 = 0;
            }
          }
        }
        update_mhz();
        update_step();
        update_power();
      }
    }

    // key 2 short
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
        // toggle sine / bt
        fm_bt_on = !fm_bt_on;
        update_modulation();
        // power to bt module
        if (fm_bt_on) {
          digitalWrite(fm_power2, HIGH);
          fm_sinemode = 0;
        }
        else {
          digitalWrite(fm_power2, LOW);
        }
        set_sine();
        while (digitalRead(Encoder_2_Key) == 0) {}
        //little debounce
        delay(200);
      }
      else {
        // short press

      }
    }

    if (EncoderCounter1 != 0) {
      if (EncoderCounter1 > 0) {
        switch (fm_mode1) {
          case 0:
            switch (fm_stepsize) {
              case 0:
                fm_frequency = fm_frequency + 5;
                break;
              case 1:
                fm_frequency = fm_frequency + 10;
                break;
              case 2:
                fm_frequency = fm_frequency + 50;
                break;
              case 3:
                fm_frequency = fm_frequency + 100;
                break;
            }
            if (fm_frequency > 10800) {
              fm_frequency = 10800;
            }
            update_frequency();
            tx_set_frequency();
            break;

          case 1:
            if (fm_stepsize < 3) {
              fm_stepsize++;
              update_step();
            }
            break;

          case 2:
            if (fm_power < 13) {
              fm_power++;
              update_power();
              tx_set_power();
            }
            break;
        }
        EncoderCounter1--;
      }
      else {
        switch (fm_mode1) {
          case 0:
            switch (fm_stepsize) {
              case 0:
                if (fm_frequency >= 8005) {
                  fm_frequency = fm_frequency - 5;
                }
                break;
              case 1:
                if (fm_frequency >= 8010) {
                  fm_frequency = fm_frequency - 10;
                }
                break;
              case 2:
                if (fm_frequency >= 8050) {
                  fm_frequency = fm_frequency - 50;
                }
                break;
              case 3:
                if (fm_frequency >= 8100) {
                  fm_frequency = fm_frequency - 100;
                }
                break;
            }
            if (fm_frequency < 8000) {
              fm_frequency = 8000;
            }
            update_frequency();
            tx_set_frequency();
            break;

          case 1:
            if (fm_stepsize > 0) {
              fm_stepsize--;
              update_step();
            }
            break;

          case 2:
            if (fm_power > 0) {
              fm_power--;
              update_power();
              tx_set_power();
            }
            break;

        }
        EncoderCounter1++;
      }
    }

    if (EncoderCounter2 != 0) {
      if (EncoderCounter2 > 0) {
        if (fm_bt_on) {
          EncoderCounter2 = 0;
        }
        else {
          if (fm_sinemode < 2) {
            fm_sinemode++;
            update_modulation();
            set_sine();
          }
          EncoderCounter2--;
        }
      }
      else {
        if (fm_bt_on) {
          EncoderCounter2 = 0;
        }
        else {
          if (fm_sinemode > 0) {
            fm_sinemode--;
            update_modulation();
            set_sine();
          }
          EncoderCounter2++;
        }
      }
    }
  }

  if (ScreenMode == 10) {
    // menu mode
    // check for key pressed
    if (digitalRead(Encoder_1_Key) == 0) {
      switch (selectedMenu) {
        case 0:
          // return
          // rebuild all
          ScreenMode = 0;
          BuildScreen(ScreenMode);
          update_mhz();
          update_frequency();
          update_step();
          update_power();
          update_modulation();
          break;

        case 1:
          if (fm_preemphasis == 0) {
            fm_preemphasis = 1;
          }
          else {
            fm_preemphasis = 0;
          }
          tx_set_preemphasis();
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
  }

  if (millis() - loop_timer > 100 and ScreenMode == 0 and EncoderCounter1 == 0 and EncoderCounter2 == 0) {
    fm_tx.ASQ(fm_overmodulated, fm_inputlevel);             // Receive overmodulation indicator and audio level
    update_overmodulation();
    loop_timer = millis();
  }

}

// build screen
void BuildScreen(byte updatemode) {
  LCD_DISPLAY.fillScreen(TFT_BLACK);

}

void update_mhz(void) {
  LCD_DISPLAY.setTextDatum(TR_DATUM);
  LCD_DISPLAY.setFreeFont(FSS9);
  if (fm_mode1 == 0) {
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  }
  else {
    LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  }
  LCD_DISPLAY.drawString("MHz", 159, 63);
}

void update_frequency(void) {
  strbuf = "     " + String(fm_frequency);
  strbuf =  strbuf.substring(strbuf.length() - 5);
  strbuf =  strbuf.substring(0, 3) + "." +  strbuf.substring(3, 5);
  strbuf.toCharArray(printbuf, 30);
  LCD_DISPLAY.setFreeFont(&DSEG7ClassicMini_Regular15pt7b);
  LCD_DISPLAY.setTextDatum(TR_DATUM);
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(114);
  LCD_DISPLAY.drawString(printbuf, 114, 48);
  LCD_DISPLAY.setTextPadding(0);
}

void update_step(void) {
  LCD_DISPLAY.setFreeFont(GLCD);
  LCD_DISPLAY.setTextDatum(TL_DATUM);

  if (fm_mode1 == 1) {
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  }
  else {
    LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  }

  LCD_DISPLAY.setTextPadding(79);
  switch (fm_stepsize) {
    case 0:
      LCD_DISPLAY.drawString("Stp 50KHz", 0, 115);
      break;

    case 1:
      LCD_DISPLAY.drawString("Stp 100KHz", 0, 115);
      break;

    case 2:
      LCD_DISPLAY.drawString("Stp 500KHz", 0, 115);
      break;

    case 3:
      LCD_DISPLAY.drawString("Stp 1MHz", 0, 115);
      break;

  }
  LCD_DISPLAY.setTextPadding(0);
}

void update_power(void) {
  LCD_DISPLAY.setFreeFont(GLCD);
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  if (fm_mode1 == 2) {
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  }
  else {
    LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  }

  LCD_DISPLAY.setTextPadding(79);
  switch (fm_power) {
    case 0:
      LCD_DISPLAY.drawString("Pwr 88 dBuV", 80, 115);
      break;

    case 1:
      LCD_DISPLAY.drawString("Pwr 90 dBuV", 80, 115);
      break;

    case 2:
      LCD_DISPLAY.drawString("Pwr 92 dBuV", 80, 115);
      break;

    case 3:
      LCD_DISPLAY.drawString("Pwr 94 dBuV", 80, 115);
      break;

    case 4:
      LCD_DISPLAY.drawString("Pwr 96 dBuV", 80, 115);
      break;

    case 5:
      LCD_DISPLAY.drawString("Pwr 98 dBuV", 80, 115);
      break;

    case 6:
      LCD_DISPLAY.drawString("Pwr 100 dBuV", 80, 115);
      break;

    case 7:
      LCD_DISPLAY.drawString("Pwr 102 dBuV", 80, 115);
      break;

    case 8:
      LCD_DISPLAY.drawString("Pwr 104 dBuV", 80, 115);
      break;

    case 9:
      LCD_DISPLAY.drawString("Pwr 106 dBuV", 80, 115);
      break;

    case 10:
      LCD_DISPLAY.drawString("Pwr 108 dBuV", 80, 115);
      break;

    case 11:
      LCD_DISPLAY.drawString("Pwr 110 dBuV", 80, 115);
      break;

    case 12:
      LCD_DISPLAY.drawString("Pwr 112 dBuV", 80, 115);
      break;

    case 13:
      LCD_DISPLAY.drawString("Pwr 115 dBuV", 80, 115);
      break;

  }
  LCD_DISPLAY.setTextPadding(0);
}

void update_modulation (void) {
  LCD_DISPLAY.setFreeFont(GLCD);
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  LCD_DISPLAY.setTextPadding(159);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  if (fm_bt_on) {
    LCD_DISPLAY.drawString("Bluetooth", 0, 0);
  }
  else {
    switch (fm_sinemode) {
      case 0:
        LCD_DISPLAY.drawString("Modulation off", 0, 0);
        break;

      case 1:
        LCD_DISPLAY.drawString("Modulation 1KHz", 0, 0);
        break;

      case 2:
        LCD_DISPLAY.drawString("Modulation 400Hz", 0, 0);
        break;

    }
  }
  LCD_DISPLAY.setTextPadding(0);
}

void update_overmodulation (void) {
  if (fm_overmodulated != fm_prev_overmodulated) {
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TR_DATUM);
    LCD_DISPLAY.setTextPadding(30);
    LCD_DISPLAY.setTextColor(TFT_RED, TFT_BLACK);
    if (fm_overmodulated) {
      LCD_DISPLAY.drawString("OVR", 159, 0);
    }
    else {
      LCD_DISPLAY.drawString("   ", 159, 0);
    }
    LCD_DISPLAY.setTextPadding(0);
  }
  fm_prev_overmodulated = fm_overmodulated;
}

// splash screen
void DisplaySplash(void ) {
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.drawString("FM transmitter", 80, 0);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.drawString("Jef Collin 2025", 80, 25);
  LCD_DISPLAY.drawString("V 1.0", 80, 45);
  // depending on startup mode, sleep or not
  if (fm_normalstart) {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Press L long: sine/bt", 0, 70);
    LCD_DISPLAY.drawString("Press R short: mode", 0, 82);
    LCD_DISPLAY.drawString("Press R long: menu", 0, 94);
    LCD_DISPLAY.drawString("Turn L: modulation", 0, 106);
    LCD_DISPLAY.drawString("Turn R: freq/stp/pwr", 0, 118);
  }
  else {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Going to sleep", 80, 85, 2);
    LCD_DISPLAY.drawString("Press knob R to wakeup", 80, 105, 2);
  }
}

// draw the menu
void DrawMenu() {

  if (fm_preemphasis == 0) {
    menuItems[1] = "Preemp 75uS";
  }
  else {
    menuItems[1] = "Preemp 50uS";
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
  DAC_Stop();
  // turn off bt module
  digitalWrite(fm_power2, LOW);

  // turn off fm module
  digitalWrite(fm_power1, LOW);

  // shutdown the backlight
  digitalWrite(Backlight_control, HIGH);
  // lcd chip to sleep
  LCD_DISPLAY.writecommand(ST7735_SLPIN);
  // Set the wakeup source to the button pin (GPIO 36)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_36, 0); // 0 means wake up when the pin is LOW (button pressed)
  // Enter deep sleep mode
  esp_deep_sleep_start();
}

void set_sine(void) {
  if (fm_bt_on) {
    DAC_Stop();
  }
  else {
    switch (fm_sinemode) {
      case 0:
        DAC_Stop();
        break;

      case 1:
        DAC_Stop();
        DAC_Start(1050);
        break;

      case 2:
        DAC_Stop();
        DAC_Start(400);
        break;
    }
  }
}

// setup and start DAC 2
// dac 1 has documented artifacts (spikes) and dac 2 does not
void DAC_Start( uint32_t freq ) {
  dac_cosine_config_t dac_cos_cfg = {
    .chan_id = DAC_CHAN_1, // GPIO26 pin on ESP32
    .freq_hz = freq,       // DAC frequency in Hz
    .clk_src = DAC_COSINE_CLK_SRC_DEFAULT, // RC_FAST

    // DAC_COSINE_ATTEN_DEFAULT    = 0x0,      /*!< No attenuation to the DAC cosine wave amplitude. Default. */
    //    DAC_COSINE_ATTEN_DB_0       = 0x0,      /*!< Original amplitude of the DAC cosine wave, equals to DAC_COSINE_ATTEN_DEFAULT */
    //    DAC_COSINE_ATTEN_DB_6       = 0x1,      /*!< 1/2 amplitude of the DAC cosine wave */
    //    DAC_COSINE_ATTEN_DB_12      = 0x2,      /*!< 1/4 amplitude of the DAC cosine wave */
    //    DAC_COSINE_ATTEN_DB_18      = 0x3,      /*!< 1/8 amplitude of the DAC cosine wave */

    .atten = DAC_COSINE_ATTEN_DB_18, //  amplitude
    .phase = DAC_COSINE_PHASE_0, // phase value (0 or 180)
    .offset = 0,           // offset value -128 ~ +127
    .flags = { .force_set_freq = true }
  };

  // Configure the DAC-CW channel
  dac_cosine_new_channel( &dac_cos_cfg, &dac_chan_handle );
  // Start te DAC-CW Generator on DAC channel
  dac_cosine_start( dac_chan_handle );
}

// stop dac if running
void DAC_Stop(void) {
  if (dac_chan_handle != NULL ) {
    dac_cosine_stop( dac_chan_handle );
    dac_cosine_del_channel( dac_chan_handle );
  }
}

void tx_set_frequency(void) {
  fm_tx.Freq(fm_frequency);
  fm_tx.RDS_AF(fm_frequency);
}

void tx_set_power(void) {
  uint8_t fm_powercode = 88;
  switch (fm_power) {
    case 0:
      fm_powercode = 88;
      break;

    case 1:
      fm_powercode = 90;
      break;

    case 2:
      fm_powercode = 92;
      break;

    case 3:
      fm_powercode = 94;
      break;

    case 4:
      fm_powercode = 96;
      break;

    case 5:
      fm_powercode = 98;
      break;

    case 6:
      fm_powercode = 100;
      break;

    case 7:
      fm_powercode = 102;
      break;

    case 8:
      fm_powercode = 104;
      break;

    case 9:
      fm_powercode = 106;
      break;

    case 10:
      fm_powercode = 108;
      break;

    case 11:
      fm_powercode = 110;
      break;

    case 12:
      fm_powercode = 112;
      break;

    case 13:
      fm_powercode = 115;
      break;
  }

  fm_tx.Output(fm_powercode, 4);

}

void tx_set_preemphasis(void) {
  switch (fm_preemphasis) {
    case 0:
      fm_tx.Audio_PreEmphasis(50);
      break;

    case 1:
      fm_tx.Audio_PreEmphasis(75);
      break;
  }
}
