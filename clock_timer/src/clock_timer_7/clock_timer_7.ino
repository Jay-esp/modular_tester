// Modular tester
// clock / timer module
// Jef Collin 2024


// revisions
// 1.0 first release




// todo




#include <TFT_eSPI.h>
#include <SPI.h>
#include "Free_Fonts.h"
#include "esp_timer.h"
#include <Wire.h>
#include "esp_sleep.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <DNSServer.h>
#include <Preferences.h>

// Include font files
#include "DSEG7ClassicMini_Regular15pt7b.h"
#include "DSEG7ClassicMini_Regular20pt7b.h"

// use alps or other decoder
#define Use_Alps_Encoder false

TFT_eSPI LCD_DISPLAY = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

TFT_eSprite LCD_SPRITE1 = TFT_eSprite(&LCD_DISPLAY);

TFT_eSprite LCD_SPRITE2 = TFT_eSprite(&LCD_DISPLAY);

// io pins
#define clock_beeper 4
#define clock_button 26
#define clock_rtc_interrupt 25

// LCD
#define Backlight_control 13

// RTC I2C
#define DS3231_I2C_ADDRESS 0x68
// note eeprom on RTC board is at x57 , not used in this project

// to read the date and time from RTC into
byte RTC_second, RTC_minute, RTC_hour, RTC_dayOfWeek, RTC_dayOfMonth, RTC_month, RTC_year;

// previous time
byte P_RTC_second, P_RTC_minute, P_RTC_hour;

// previous date
byte P_RTC_dayOfWeek, P_RTC_dayOfMonth, P_RTC_month, P_RTC_year;

char weekDay[7][10] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
char monthName[12][10] = {
  "January", "February", "March", "April", "May", "June",
  "July", "August", "September", "October", "November", "December"
};

// sleep or startup
boolean clock_normalstart = false;

// are we in setup mode (access point)
boolean clock_setup_mode = false;

boolean clock_update_flag = false;
boolean clock_update_date_flag = true;

boolean clock_force_update = false;

uint8_t clock_timer_minutes = 0;
uint8_t clock_timer_seconds = 0;

boolean clock_timer_running = false;

uint8_t clock_countdown_minutes = 1;
uint8_t clock_countdown_seconds = 0;

boolean clock_autoset_countdown = true;

boolean clock_beep_enabled = true;

boolean clock_do_beep = false;

// fail counter for problems with ntp access
byte ntp_error = 0;

// flag if ntp is synced
byte ntp_insync = 0;

// index value for NTP pool
byte ntp_index = 0;

// NTP settings
byte packetBuffer[48]; //buffer to hold incoming & outgoing packets

// to avoid DNS conflicts use a fixed IP address to the NTP server
// setup an array of possible NTP server
IPAddress timeServer[7] = {IPAddress(157, 193, 40, 37) , IPAddress(24, 56, 178, 140) , IPAddress(129, 6, 15, 29), IPAddress(129, 6, 15, 28), IPAddress(193, 79, 237, 14), IPAddress(129, 6, 15, 30), IPAddress(132, 163, 4, 101) };

// NTP Server pool:
static const char ntpServerName[] = "us.pool.ntp.org";
//static const char ntpServerName[] = "time.nist.gov";
//static const char ntpServerName[] = "time-a.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-b.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-c.timefreq.bldrdoc.gov";

// assign UDP (used for NTP)
WiFiUDP  Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

DNSServer dnsServer;

WiFiServer server(80);

// wifi setup parameters
String su_ssid = "";
String su_pw = "";
String su_ip = "";
String su_gtw = "";
String su_sub = "";
String su_prim = "";
String su_sec = "";

Preferences preferences;

// Menu variables

const int numMenuItems = 4;
const char* menuItems[] = {"Return", "Beep On", "Autoset cdn on", "Go to sleep"};
int selectedMenu = 0;
int menuOffset = 0;

// screen mode
// 0 main clock
// 1 timer
// 2 countdown
// 3 setup countdown min
// 4 setup countdown sec
// 20.. setup
byte ScreenMode = 0;

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

unsigned char Encoder_State1 = R_START;

// rotary encode io pins
char Encoder_Pin1 = 34;
char Encoder_Pin2 = 27;
char Encoder_Key_Pin = 35;

// track rotary encoder changes
int EncoderCounter = 0;

unsigned long timer_button;

boolean Encoder_Key_Long_Press = false;

// interrupt routine for rotary encoder
void IRAM_ATTR isr1() {
  unsigned char pinstate = (digitalRead(Encoder_Pin2) << 1) | digitalRead(Encoder_Pin1);
  Encoder_State1 = ttable[Encoder_State1 & 0xf][pinstate];
  unsigned char result = Encoder_State1 & 0x30;
  if (result == DIR_CW) {
    if (EncoderCounter < 30) {
      EncoderCounter++;
    }
  } else if (result == DIR_CCW) {
    if (EncoderCounter > -30) {
      EncoderCounter--;
    }
  }
}

// 1 second interrupt by rtc
void IRAM_ATTR rtc_interrupt_handler() {
  // timer interrupt, trigger update
  clock_update_flag = true;
}

void setup() {

  // enable only for debugging
  // Serial.begin(115200);

  Wire.begin(); // Initialize I2C communication

  // setup encoder pins
  pinMode(Encoder_Pin1, INPUT);
  pinMode(Encoder_Pin2, INPUT);
  pinMode(Encoder_Key_Pin, INPUT);

  pinMode(clock_button, INPUT);

  pinMode(clock_rtc_interrupt, INPUT);

  pinMode(Backlight_control, OUTPUT);

  digitalWrite(Backlight_control, HIGH);

  // beeper off
  pinMode(clock_beeper, OUTPUT);
  digitalWrite(clock_beeper, LOW);

  // Setup the LCD
  LCD_DISPLAY.init();
  LCD_DISPLAY.setRotation(3);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  delay(100);
  // turn on backlight
  digitalWrite(Backlight_control, LOW);

  // check for key pressed, enable normal start otherwise go to sleep
  if (digitalRead(Encoder_Key_Pin) == 0) {
    clock_normalstart = true;
  }

  // check for button pressed, wifi setup mode
  if (digitalRead(clock_button) == 0) {
    clock_setup_mode = true;
  }

  loadWiFiParameters(su_ssid, su_pw, su_ip, su_gtw, su_sub, su_prim, su_sec);

  // configure access point with captive portal to setup wifi parameters if both buttons pressed during power on
  if (clock_normalstart and clock_setup_mode) {
    ScreenMode = 20;
    BuildScreen(ScreenMode);
    // wifi setup
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Modular Tester Clock/Timer Setup");
    delay(150);
    WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
    dnsServer.start(53, "*", IPAddress(192, 168, 1, 1));
    server.begin();
  }
  else {
    DisplaySplash();

    if (clock_normalstart or clock_setup_mode) {
      // wait until key is no longer pressed
      while (digitalRead(Encoder_Key_Pin) == 0) {}
    }

    delay(2500);

    // get current state to start otherwise encoder might not react to first click
    unsigned char temppinstate = (digitalRead(Encoder_Pin2) << 1) | digitalRead(Encoder_Pin1);
    Encoder_State1 = ttable[Encoder_State1 & 0xf][temppinstate];

    // setup encoder interrupts
    attachInterrupt(Encoder_Pin1, isr1, CHANGE);
    attachInterrupt(Encoder_Pin2, isr1, CHANGE);

    // goto sleep
    if (!clock_normalstart) {
      GoToSleep();
    }

    configerWiFi();
    delay(150);

    Udp.begin(localPort);
    delay(150);

    // sync every xx hours (in seconds)
    // set just in case but other sync option is used
    setSyncInterval(3600);

    // set NTP subroutine as external sync source
    setSyncProvider(getNtpTime);

    // call the time library to initiate process and update from the NTP server once
    now();

    // previous time
    P_RTC_second = 0;
    P_RTC_minute = 0;
    P_RTC_hour = 0;

    // previous date
    P_RTC_dayOfWeek = 0;
    P_RTC_dayOfMonth = 0;
    P_RTC_month = 0;
    P_RTC_year = 0;

    readDS3231time(&RTC_second, &RTC_minute, &RTC_hour, &RTC_dayOfWeek, &RTC_dayOfMonth, &RTC_month, &RTC_year);

    // main screen sprite
    LCD_SPRITE1.createSprite(160, 41);

    LCD_SPRITE2.createSprite(160, 128);


    attachInterrupt(digitalPinToInterrupt(clock_rtc_interrupt), rtc_interrupt_handler, FALLING);

    DS3231_OneSecondInterrupt();

    LCD_DISPLAY.fillScreen(TFT_BLACK);

    BuildScreen(ScreenMode);

    LCD_DISPLAY.setTextSize(1);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TC_DATUM);
    LCD_DISPLAY.drawString("...ntp connect...", 80, 10);

  }
}

void loop() {

  if (clock_setup_mode) {
    // AP mode for setup
    clock_setup_routine();
    // exit just in case we do not want to setup the wifi
    if (digitalRead(Encoder_Key_Pin) == 0) {
      GoToSleep();
    }
  }
  else {
    // normal running mode
    // if we have NTP failures, cycle through array of possible NTP IPs
    if (ntp_error > 0) {
      //  Serial.println("Fail ntp");
      // point index to next NTP server in list
      // 0 is reserved for automatic ip, others for fixed ip list fallback
      ntp_index += 1;
      if (ntp_index > 7) {
        // cycle again
        ntp_index = 0;
      }
      // try again, time library function is not used but RTC is updated
      getNtpTime();
    }

    // force update from 1 second interrupt
    if (clock_update_flag) {
      readDS3231time(&RTC_second, &RTC_minute, &RTC_hour, &RTC_dayOfWeek, &RTC_dayOfMonth, &RTC_month, &RTC_year);
      // check if hour is 4, force update clock from NTP
      // time is choosen to catch summertime/wintertime transit
      if (RTC_hour == 4) {
        // only if not already synced
        if (ntp_insync == 0) {
          getNtpTime();
          ntp_insync = 1;
        }
      }
      else {
        // if different hour, reset sync flag
        ntp_insync = 0;
      }
      switch (ScreenMode) {
        case 0:
          // normal clock
          UpdateTime0();

          if ((RTC_dayOfWeek != P_RTC_dayOfWeek) or (RTC_dayOfMonth != P_RTC_dayOfMonth) or (RTC_month != P_RTC_month) or (RTC_year != P_RTC_year) or clock_update_date_flag) {
            UpdateDate();
            clock_update_date_flag = false;
          }
          break;

        case 1:
          // timer
          // time + 1 second
          if (clock_timer_running) {
            if (clock_timer_minutes <= 59 and clock_timer_seconds <= 59) {
              clock_timer_seconds++;
              if (clock_timer_seconds == 60) {
                clock_timer_seconds = 0;
                clock_timer_minutes++;
              }
            }
          }
          UpdateTime1();
          break;

        case 2:
          // countdown
          // time - 1 second
          if (clock_timer_running) {
            if (clock_timer_minutes > 0 or clock_timer_seconds > 0) {
              if (clock_timer_seconds == 0) {
                if (clock_timer_minutes > 0) {
                  clock_timer_seconds = 59;
                  clock_timer_minutes--;
                }
              }
              else {
                clock_timer_seconds--;
              }
              UpdateTime1();
            }
            if (clock_timer_minutes == 0 and clock_timer_seconds == 0) {
              clock_timer_running = false;
              eraseClock(149, 10);
              if (clock_beep_enabled) {
                tone(clock_beeper, 2000, 100);
                delay(200);
                tone(clock_beeper, 2000, 100);
                delay(200);
                tone(clock_beeper, 2000, 100);
              }
              if (clock_autoset_countdown) {
                clock_timer_minutes = clock_countdown_minutes;
                clock_timer_seconds = clock_countdown_seconds;
                UpdateTime1();
              }
            }
          }
          //UpdateTime1();
          break;

        case 3:
          // countdown setup min

          break;

        case 4:
          // countdown setup sec

          break;

      }
      clock_update_flag = false;
    }

    // encoder button pressed
    if (ScreenMode < 10 and digitalRead(Encoder_Key_Pin) == 0) {
      // key short toggle countdown setup mode, long menu
      timer_button = millis();
      Encoder_Key_Long_Press = false;
      // wait until key is no longer pressed or time expired
      while (digitalRead(Encoder_Key_Pin) == 0) {
        if (millis() - timer_button > 1000) {
          Encoder_Key_Long_Press = true;
          break;
        }
      }
      if (Encoder_Key_Long_Press) {
        // long press
        ScreenMode = 10;
        selectedMenu = 0;
        menuOffset = 0;
        DrawMenu();
        while (digitalRead(Encoder_Key_Pin) == 0) {}
        //little debounce
        delay(200);
      }
      else {
        // short press
        if (ScreenMode == 2) {
          // countdown mode
          clock_timer_running = false;
          // go to time setup mode
          ScreenMode = 3;
          BuildScreen(ScreenMode);
          UpdateTime2();
        }
        else {
          if (ScreenMode == 3) {
            // in countdown setup mode min
            ScreenMode = 4;
            // back to countdown mode
            BuildScreen(ScreenMode);
            UpdateTime3();
          }
          else {
            if (ScreenMode == 4) {
              // in countdown setup mode sec
              ScreenMode = 2;
              // back to countdown mode
              BuildScreen(ScreenMode);
              // set value
              clock_timer_minutes = clock_countdown_minutes;
              clock_timer_seconds = clock_countdown_seconds;
              UpdateTime1();
            }
          }
        }
      }
    }

    // button pressed in timer or countdown mode
    if ((ScreenMode == 1 or ScreenMode == 2) and digitalRead(clock_button) == 0) {
      // key short start / stop, long reset
      timer_button = millis();
      Encoder_Key_Long_Press = false;
      // wait until key is no longer pressed or time expired
      while (digitalRead(clock_button) == 0) {
        if (millis() - timer_button > 1000) {
          Encoder_Key_Long_Press = true;
          break;
        }
      }
      if (Encoder_Key_Long_Press) {
        // long press
        if (!clock_timer_running) {
          if (ScreenMode == 1) {
            clock_timer_minutes = 0;
            clock_timer_seconds = 0;
            UpdateTime1();
          }
          if (ScreenMode == 2) {
            clock_timer_minutes = clock_countdown_minutes;
            clock_timer_seconds = clock_countdown_seconds;
            UpdateTime1();
          }
          while (digitalRead(clock_button) == 0) {
          }
          delay(250);
        }
        else {
          clock_timer_running = false;
          eraseClock(149, 10);
          while (digitalRead(clock_button) == 0) {
          }
          delay(100);
        }
      }
      else {
        // short press
        if (ScreenMode == 2 and clock_timer_minutes == 0 and clock_timer_seconds == 0) {
          // countdown is not set
          tone(clock_beeper, 2000, 100);
        }
        else {
          // toggle timer
          clock_timer_running = !clock_timer_running;

          // if timer starts, reset seconds to restart second interrupt timing, resync to ntp later
          if (clock_timer_running) {
            resetRTCSeconds();
          }
          // update icon
          if (clock_timer_running) {
            drawClock(149, 10);
          }
          else {
            eraseClock(149, 10);
          }
        }
        delay(250);
      }
    }

    switch (ScreenMode) {
      case 0:
        // clock mode

        break;

      case 1:
        // timer mode

        break;

      case 2:
        // countdown mode

        break;

      case 10:
        //menu mode
        // check for key pressed
        if (digitalRead(Encoder_Key_Pin) == 0) {
          switch (selectedMenu) {
            case 0:
              // return
              ScreenMode = 0;
              BuildScreen(ScreenMode);
              clock_update_flag = true;
              clock_update_date_flag = true;
              // resync just in case
              getNtpTime();
              break;

            case 1:
              clock_beep_enabled = !clock_beep_enabled;
              DrawMenu();
              break;

            case 2:
              clock_autoset_countdown = !clock_autoset_countdown;
              DrawMenu();
              break;

            case 3:
              // sleep
              LCD_DISPLAY.fillScreen(TFT_BLACK);
              // wait until key is no longer pressed
              while (digitalRead(Encoder_Key_Pin) == 0) {}

              // debounce to avoid early wakeup
              delay(100);
              GoToSleep();
              break;

          }
          // wait until key is no longer pressed
          while (digitalRead(Encoder_Key_Pin) == 0) {}
          // debounce
          delay(100);
        }
        // scroll menu
        if (EncoderCounter != 0) {
          if (EncoderCounter > 0) {
            if (selectedMenu < numMenuItems - 1) {
              selectedMenu++;
              if (selectedMenu >= menuOffset + 4) {
                menuOffset++;
              } else if (selectedMenu < menuOffset) {
                menuOffset--;
              }
              DrawMenu();
            }
            EncoderCounter--;
          }
          if (EncoderCounter < 0) {
            if (selectedMenu > 0) {
              selectedMenu--;
              if (selectedMenu >= menuOffset + 4) {
                menuOffset++;
              } else if (selectedMenu < menuOffset) {
                menuOffset--;
              }
              DrawMenu();
            }
            EncoderCounter++;
          }
        }
        break;
    }
  }

  // scroll screens
  if (ScreenMode < 3 and EncoderCounter != 0) {
    clock_force_update = false;
    if (EncoderCounter > 0) {
      if (ScreenMode < 2) {
        ScreenMode++;
        clock_force_update = true;
        EncoderCounter--;
      }
      else {
        EncoderCounter = 0;
      }
    }
    else {
      if (ScreenMode > 0) {
        ScreenMode--;
        clock_force_update = true;
        // if we return to normal clock, sync from ntp again since we reset the rtc
        if (ScreenMode == 0) {
          getNtpTime();
        }
        EncoderCounter++;
      }
      else {
        EncoderCounter = 0;
      }
    }
    if (clock_force_update) {
      BuildScreen(ScreenMode);
      if (ScreenMode == 1) {
        // switch to timer mode
        clock_timer_minutes = 0;
        clock_timer_seconds = 0;
        clock_timer_running = false;
        UpdateTime1();
      }
      else {
        if (ScreenMode == 2) {
          // switch to countdown mode
          clock_timer_minutes = clock_countdown_minutes;
          clock_timer_seconds = clock_countdown_seconds;
          clock_timer_running = false;
          UpdateTime1();
        }
      }
      // force rebuild
      clock_update_flag = true;
      clock_update_date_flag = true;
    }
  }

  // countdown time setup mode min
  if (ScreenMode == 3 and EncoderCounter != 0) {
    if (EncoderCounter > 0) {
      if (clock_countdown_minutes < 59 or (clock_countdown_minutes == 59 and clock_countdown_seconds == 0)) {
        clock_countdown_minutes++;
        EncoderCounter--;
      }
      else {
        EncoderCounter = 0;
      }
    }
    else {
      if (clock_countdown_minutes > 0) {
        clock_countdown_minutes--;
        EncoderCounter++;
      }
      else {
        EncoderCounter = 0;
      }
    }
    UpdateTime2();
  }

  // countdown time setup mode sec
  if (ScreenMode == 4 and EncoderCounter != 0) {
    if (EncoderCounter > 0) {
      if (clock_countdown_minutes <= 59 and clock_countdown_seconds <= 59) {
        clock_countdown_seconds++;
        if (clock_countdown_seconds == 60) {
          clock_countdown_seconds = 0;
          clock_countdown_minutes++;
        }
        EncoderCounter--;
      }
      else {
        EncoderCounter = 0;
      }
    }
    else {
      if (clock_countdown_minutes > 0 or clock_countdown_seconds > 0) {
        if (clock_countdown_seconds == 0) {
          if (clock_countdown_minutes > 0) {
            clock_countdown_seconds = 59;
            clock_countdown_minutes--;
          }
        }
        else {
          clock_countdown_seconds--;
        }
        EncoderCounter++;
      }
      else {
        EncoderCounter = 0;
      }
    }
    UpdateTime3();
  }


}

// splash screen
void DisplaySplash(void ) {
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.drawString("Clock / Timer", 80, 0);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.drawString("Jef Collin 2024", 80, 25);
  LCD_DISPLAY.drawString("V 1.0", 80, 45);
  // depending on startup mode, sleep or not
  if (clock_normalstart) {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Turn knob: mode", 80, 65, 2);
    LCD_DISPLAY.drawString("Btn S: start/stop, L: reset", 80, 85, 2);
    LCD_DISPLAY.drawString("Knob S: set, L: menu", 80, 105, 2);
  }
  else {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Going to sleep", 80, 65, 2);
    LCD_DISPLAY.drawString("Button+knob setup WiFi", 80, 85, 2);
    LCD_DISPLAY.drawString("Press knob to wakeup", 80, 105, 2);
  }
}

void BuildScreen(uint8_t ScreenNumber) {
  LCD_DISPLAY.fillScreen(TFT_BLACK);

  if (ScreenNumber == 0) {
    // handled by time & date update

  }

  if (ScreenNumber == 1) {
    // timer
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
    LCD_DISPLAY.drawString("Timer", 0, 0);
  }

  if (ScreenNumber == 2) {
    // timer
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
    LCD_DISPLAY.drawString("Countdown", 0, 0);
  }

  if (ScreenNumber == 3 or ScreenNumber == 4) {
    // timer
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
    LCD_DISPLAY.drawString("Countdown setup", 0, 0);
  }

  // wifi setup mode
  if (ScreenNumber == 20) {
    LCD_DISPLAY.setTextSize(1);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TC_DATUM);
    LCD_DISPLAY.drawString("WiFi setup", 80, 10);
    LCD_DISPLAY.drawString("Access Point active", 80, 55);
    LCD_DISPLAY.drawString("Press knob to sleep", 80, 90);
  }

  // wifi setup mode done
  if (ScreenNumber == 21) {
    LCD_DISPLAY.setTextSize(1);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TC_DATUM);
    LCD_DISPLAY.drawString("WiFi setup", 80, 10);
    LCD_DISPLAY.drawString("Completed", 80, 60);
    LCD_DISPLAY.drawString("Rebooting", 80, 90);
  }

  // wifi setup failed
  if (ScreenNumber == 22) {
    LCD_DISPLAY.setTextSize(1);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextColor(TFT_RED, TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TC_DATUM);
    LCD_DISPLAY.drawString("WiFi setup", 80, 10);
    LCD_DISPLAY.drawString("Failed!", 80, 55);
    LCD_DISPLAY.drawString("Going to sleep", 80, 90);
  }

}

// time update
void UpdateTime0(void) {
  LCD_SPRITE1.fillSprite(TFT_BLACK);
  LCD_SPRITE1.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_SPRITE1.setTextDatum(TL_DATUM);
  LCD_SPRITE1.setFreeFont(&DSEG7ClassicMini_Regular15pt7b);
  LCD_SPRITE1.setCursor(1, 35);  // Set cursor position
  // Format and print time (leading zero for single digits)
  if (RTC_hour < 10) LCD_SPRITE1.print('0');
  LCD_SPRITE1.print(RTC_hour);
  LCD_SPRITE1.print(':');
  if (RTC_minute < 10) LCD_SPRITE1.print('0');
  LCD_SPRITE1.print(RTC_minute);
  LCD_SPRITE1.print(':');
  if (RTC_second < 10) LCD_SPRITE1.print('0');
  LCD_SPRITE1.print(RTC_second);
  LCD_SPRITE1.pushSprite(0, 0);
  P_RTC_second = RTC_second;
  P_RTC_minute = RTC_minute;
  P_RTC_hour = RTC_hour;
}

// time update timer
void UpdateTime1(void) {
  LCD_SPRITE1.fillSprite(TFT_BLACK);
  LCD_SPRITE1.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_SPRITE1.setTextDatum(TL_DATUM);
  LCD_SPRITE1.setFreeFont(&DSEG7ClassicMini_Regular20pt7b);
  LCD_SPRITE1.setCursor(12, 39);
  // Set cursor position
  // Format and print time (leading zero for single digits)
  if (clock_timer_minutes < 10) LCD_SPRITE1.print('0');
  LCD_SPRITE1.print(clock_timer_minutes);
  LCD_SPRITE1.print(':');
  if (clock_timer_seconds < 10) LCD_SPRITE1.print('0');
  LCD_SPRITE1.print(clock_timer_seconds);
  LCD_SPRITE1.pushSprite(0, 45);
}

// time update timer settings countdown min
void UpdateTime2(void) {
  LCD_SPRITE1.fillSprite(TFT_BLACK);
  LCD_SPRITE1.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_SPRITE1.setTextDatum(TL_DATUM);
  LCD_SPRITE1.setFreeFont(&DSEG7ClassicMini_Regular20pt7b);
  LCD_SPRITE1.setCursor(12, 39);
  // Set cursor position
  // Format and print time (leading zero for single digits)
  if (clock_countdown_minutes < 10) LCD_SPRITE1.print('0');
  LCD_SPRITE1.print(clock_countdown_minutes);
  LCD_SPRITE1.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_SPRITE1.print(':');
  if (clock_countdown_seconds < 10) LCD_SPRITE1.print('0');
  LCD_SPRITE1.print(clock_countdown_seconds);
  LCD_SPRITE1.pushSprite(0, 45);
}

// time update timer settings countdown sec
void UpdateTime3(void) {
  LCD_SPRITE1.fillSprite(TFT_BLACK);
  LCD_SPRITE1.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_SPRITE1.setTextDatum(TL_DATUM);
  LCD_SPRITE1.setFreeFont(&DSEG7ClassicMini_Regular20pt7b);
  LCD_SPRITE1.setCursor(12, 39);
  // Set cursor position
  // Format and print time (leading zero for single digits)
  if (clock_countdown_minutes < 10) LCD_SPRITE1.print('0');
  LCD_SPRITE1.print(clock_countdown_minutes);
  LCD_SPRITE1.print(':');
  LCD_SPRITE1.setTextColor(TFT_GREEN, TFT_BLACK);
  if (clock_countdown_seconds < 10) LCD_SPRITE1.print('0');
  LCD_SPRITE1.print(clock_countdown_seconds);
  LCD_SPRITE1.pushSprite(0, 45);
}

// date update
void UpdateDate(void) {
  // Clear previous date
  LCD_DISPLAY.fillRect(0, 69, 159, 52, TFT_BLACK);
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.drawString(weekDay[RTC_dayOfWeek - 1], 80, 70);
  char dateString[30];  // Buffer to hold the formatted date string
  // Format the date as "dd-Month-yyyy"
  sprintf(dateString, "%02d-%s-%04d", RTC_dayOfMonth, monthName[RTC_month - 1], 2000 + RTC_year);
  LCD_DISPLAY.drawString(dateString, 80, 105);
  P_RTC_dayOfWeek = RTC_dayOfWeek;
  P_RTC_dayOfMonth = RTC_dayOfMonth;
  P_RTC_month = RTC_month;
  P_RTC_year = RTC_year;
}

// draw the menu
void DrawMenu(void) {
  if (clock_beep_enabled) {
    menuItems[1] = "Beep off";
  }
  else {
    menuItems[1] = "Beep on";
  }

  if (clock_autoset_countdown) {
    menuItems[2] = "Autoset cdn off";
  }
  else {
    menuItems[2] = "Autoset cdn on";
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
  // shutdown the backlight
  digitalWrite(Backlight_control, HIGH);
  // lcd chip to sleep
  LCD_DISPLAY.writecommand(ST7735_SLPIN);
  // Set the wakeup source to the button pin (GPIO 35)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0); // 0 means wake up when the pin is LOW (button pressed)
  // Enter deep sleep mode
  esp_deep_sleep_start();
}

// Load Wi-Fi parameters function
void loadWiFiParameters(String & ssid, String & password, String & ip, String & gateway, String & submask, String & primDNS, String & secDNS) {
  preferences.begin("wifi", false);  // Open preferences with namespace "wifi"
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
  ip = preferences.getString("ip", "192.168.1.1");  // Default IP
  gateway = preferences.getString("gateway", "192.168.1.1");  // Default gateway
  submask = preferences.getString("submask", "255.255.255.0");  // Default subnet mask
  primDNS = preferences.getString("primDNS", "8.8.8.8");  // Default primary DNS
  secDNS = preferences.getString("secDNS", "8.8.4.4");  // Default secondary DNS
  preferences.end();
}

// Save Wi-Fi parameters function
void saveWiFiParameters(const String & ssid, const String & password, const String & ip, const String & gateway, const String & submask, const String & primDNS, const String & secDNS) {
  preferences.begin("wifi", false);  // Open preferences with namespace "wifi"
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  preferences.putString("ip", ip);
  preferences.putString("gateway", gateway);
  preferences.putString("submask", submask);
  preferences.putString("primDNS", primDNS);
  preferences.putString("secDNS", secDNS);
  preferences.end();
}

// extract input from captive portal
String extractParam(const String & request, const String & param) {
  int startIndex = request.indexOf(param + "=");
  if (startIndex == -1) return "";  // Parameter not found
  startIndex += param.length() + 1;  // Move to the start of the parameter value
  int endIndex = request.indexOf("&", startIndex);  // Find the end of the parameter value
  if (endIndex == -1) endIndex = request.length();  // Handle last parameter case
  return request.substring(startIndex, endIndex);  // Return the parameter value
}

// wifi setup
void configerWiFi(void) {
  // Create IPAddress objects using the constructor
  IPAddress ipAddr;
  IPAddress gatewayAddr;
  IPAddress submaskAddr;
  IPAddress primDNSAddr;
  IPAddress secDNSAddr;

  ipAddr.fromString(su_ip);  // Convert string to IPAddress
  gatewayAddr.fromString(su_gtw);
  submaskAddr.fromString(su_sub);
  primDNSAddr.fromString(su_prim);
  secDNSAddr.fromString(su_sec);

  // Configure Wi-Fi
  WiFi.config(ipAddr, gatewayAddr, submaskAddr, primDNSAddr, secDNSAddr);
  WiFi.begin(su_ssid.c_str(), su_pw.c_str());

  unsigned long wifitimer = millis();

  while ((WiFi.status() != WL_CONNECTED) and millis() - wifitimer < 5000) {
    // Serial.println("wifi wait..");
    delay(200);
  }

  // loop if connection failed
  if (WiFi.status() != WL_CONNECTED) {
    BuildScreen(22);
    delay(10000);
    GoToSleep();
  }
}

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ( (val / 10 * 16) + (val % 10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val / 16 * 10) + (val % 16) );
}

// subroutine called by the time functions to get the NTP time
// and set the RTC
// can be called directly to sync
time_t getNtpTime()
{
  // if we are not on the main clock screen we need to ignore ntp updates since it messes up the timer functions
  if (ScreenMode != 0) {
    ntp_index = 0;
    return 0;
  }

  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  if (ntp_index == 0) {
    // get a random server from the pool
    WiFi.hostByName(ntpServerName, ntpServerIP);
  }
  else {
    // fallback to hardcoded IP
    // use indexed NTP IP address
    ntpServerIP = timeServer[ntp_index - 1];
  }
  // Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);

  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= 48) {
      Udp.read(packetBuffer, 48);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      //correct for 1970 and timezone, +1 = central european time
      unsigned long epoch = secsSince1900 - 2208988800UL + 1 * SECS_PER_HOUR;
      // correct for summertime
      epoch = epoch + dstOffset(epoch);
      // setup time elements variable
      tmElements_t te;
      // break down epoch time to elements
      breakTime(epoch, te);
      // write current NTP time to RTC
      setDS3231time(te.Second, te.Minute, te.Hour, te.Wday, te.Day, te.Month, te.Year - 30);
      // clear UDP buffers
      Udp.flush();
      // reset fail counter
      ntp_error = 0;

      return epoch;
    }
  }
  // we failed to get an NTP time
  // increased successive fails
  ntp_error += 1;
  // reset if overflow
  if (ntp_error > 250) {
    ntp_error = 1;
  }
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress & address) // ip direct
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, 48);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, 48);
  Udp.endPacket();
}

// correction for summertime
int dstOffset (unsigned long unixTime)
{
  //Receives unix epoch time and returns seconds of offset for local DST
  // EU: last sunday of march until last sunday of october
  //Get epoch times @ "http://www.epochconverter.com/" for testing
  //DST update wont be reflected until the next time sync
  time_t t = unixTime;
  tmElements_t te;
  te.Year = year(t) - 1970;
  te.Month = 3;
  te.Day = 31;
  te.Hour = 0;
  te.Minute = 0;
  te.Second = 0;
  time_t dstStart, dstEnd, current;
  dstStart = makeTime(te);
  dstStart = previousSunday(dstStart); // get last sunday of month
  dstStart += 2 * SECS_PER_HOUR; //2AM ntp time
  te.Month = 10;
  dstEnd = makeTime(te);
  dstEnd = previousSunday(dstEnd); // get last sunday of month
  dstEnd += 2 * SECS_PER_HOUR; //2AM ntp time
  if (t >= dstStart && t < dstEnd) return (3600); //Add back in one hours worth of seconds - DST in effect
  else return (0); //NonDST
}

// set RTC
void setDS3231time(byte RTC_second, byte RTC_minute, byte RTC_hour, byte RTC_dayOfWeek, byte RTC_dayOfMonth, byte RTC_month, byte RTC_year)
{
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(RTC_second)); // set seconds
  Wire.write(decToBcd(RTC_minute)); // set minutes
  Wire.write(decToBcd(RTC_hour)); // set hours
  Wire.write(decToBcd(RTC_dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(RTC_dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(RTC_month)); // set month
  Wire.write(decToBcd(RTC_year)); // set year (0 to 99)
  Wire.endTransmission();
}

// read RTC
void readDS3231time(byte * RTC_second, byte * RTC_minute, byte * RTC_hour, byte * RTC_dayOfWeek, byte * RTC_dayOfMonth, byte * RTC_month, byte * RTC_year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *RTC_second = bcdToDec(Wire.read() & 0x7f);
  *RTC_minute = bcdToDec(Wire.read());
  *RTC_hour = bcdToDec(Wire.read() & 0x3f);
  *RTC_dayOfWeek = bcdToDec(Wire.read());
  *RTC_dayOfMonth = bcdToDec(Wire.read());
  *RTC_month = bcdToDec(Wire.read());
  *RTC_year = bcdToDec(Wire.read());
}

void writeDS3231(byte reg, byte value) {
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte readDS3231(byte reg) {
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(DS3231_I2C_ADDRESS, 1);
  return Wire.read();
}

// reset the seconds of the RTC for timer functions
void resetRTCSeconds() {
  writeDS3231(0, 0);
}

// setup 1 second interrupt
void DS3231_OneSecondInterrupt(void) {
  // Set the DS3231 to output a 1Hz square wave on the INT/SQW pin
  byte controlReg = readDS3231(0x0E);
  controlReg &= 0b11111100;  // Clear RS1 and RS2 bits (set to 1Hz)
  controlReg &= ~(1 << 2);   // Clear INTCN bit (enable square wave output)
  writeDS3231(0x0E, controlReg);
}

// wifi setup in access point mode
void clock_setup_routine(void) {
  dnsServer.processNextRequest();
  WiFiClient client = server.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
    // Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    String requestBody = "";                // to hold the POST data

    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        //Serial.write(c);                    // print it out the serial monitor
        requestBody += c;
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {

            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:

            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            client.println("<!DOCTYPE html>\n<html>\n<head>\n<meta charset='utf-8'>\n<title>Clock / Timer setup page</title>");
            client.println(F("<meta name='viewport' content='width=device-width, initial-scale=1'>"));
            client.println(F("<meta name='generator' content='Google Web Designer 7.1.0.1122'>"));

            client.println(F("<style> body { font-family: Arial, Helvetica, sans-serif; } </style>"));

            client.println(F("<style type='text/css' id='gwd-text-style'> p {margin: 0px;} h1 {margin: 0px;} h2 {margin: 0px;} h3 {margin: 0px;}</style>"));
            client.println(F("<style type='text/css'> html, body {width: 100%;height: 100%;margin: 0px;} body {background-color: transparent;transform: perspective(1400px) matrix3d(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);transform-style: preserve-3d;}"));

            client.println(F(".gwd-button-15xv {position:absolute;width:200px;height:32px;left:10px;top:480px;background-image:none;background-color:#619ee8;font-size:18px;}"));
            client.println(F(".gwd-button-10o7 {top:530px;}"));

            client.println(F(".g-l-15wo{position:absolute;width:200px;height:20px;left:10px;top:20px;}"));
            client.println(F(".g-l-e974{position:absolute;width:200px;height:20px;left:10px;top:50px;}"));
            client.println(F(".g-l-1sdk{position:absolute;width:200px;height:20px;left:10px;top:110px;}"));
            client.println(F(".g-l-1nvc{position:absolute;width:200px;height:20px;left:10px;top:170px;}"));
            client.println(F(".g-l-12bu{position:absolute;width:200px;height:20px;left:10px;top:230px;}"));
            client.println(F(".g-l-j505{position:absolute;width:200px;height:20px;left:10px;top:290px;}"));
            client.println(F(".g-l-1e8f{position:absolute;width:200px;height:20px;left:10px;top:350px;}"));
            client.println(F(".g-l-1jjm{position:absolute;width:200px;height:20px;left:10px;top:410px;}"));
            client.println(F(".g-i-txx9{position:absolute;width:200px;height:20px;left:10px;top:75px;}"));
            client.println(F(".g-i-dnss{position:absolute;width:200px;height:20px;left:10px;top:135px;}"));
            client.println(F(".g-i-zyp8{position:absolute;width:200px;height:20px;left:10px;top:195px;}"));
            client.println(F(".g-i-1mfb{position:absolute;width:200px;height:20px;left:10px;top:255px;}"));
            client.println(F(".g-i-d2c5{position:absolute;width:200px;height:20px;left:10px;top:315px;}"));
            client.println(F(".g-i-1ic8{position:absolute;width:200px;height:20px;left:10px;top:375px;}"));
            client.println(F(".g-i-1kpt{position:absolute;width:200px;height:20px;left:10px;top:435px;}"));

            client.println(F("</style>"));
            client.println(F("</head>"));
            client.println(F("<body class='htmlNoPages'>"));
            client.println(F("<label id='label_1' class='g-l-15wo'>Setup Clock / Timer</label>"));
            client.println(F("<label id='label_2' class='g-l-e974'>WiFi SSID</label>"));
            client.println(F("<label id='label_3' class='g-l-1sdk'>WiFi Password</label>"));
            client.println(F("<label id='label_4' class='g-l-1nvc'>Fixed IP address</label>"));
            client.println(F("<label id='label_5' class='g-l-12bu'>Gateway</label>"));
            client.println(F("<label id='label_6' class='g-l-j505'>Submask</label>"));
            client.println(F("<label id='label_7' class='g-l-1e8f'>Primary DNS</label>"));
            client.println(F("<label id='label_8' class='g-l-1jjm'>Secundary DNS</label>"));

            client.print(F("<form action='/setup' method=get>"));
            client.print("<input type='text' id='text_1' name='su_ssid' class='g-i-txx9' value='");
            client.print(su_ssid);
            client.print("'>");
            client.print("<input type='text' id='text_2' name='su_pw' class='g-i-dnss' value='");
            client.print(su_pw);
            client.print("'>");
            client.print("<input type='text' id='text_3' name='su_ip' class='g-i-zyp8' value='");
            client.print(su_ip);
            client.print("'>");
            client.print("<input type='text' id='text_4' name='su_gtw' class='g-i-1mfb' value='");
            client.print(su_gtw);
            client.print("'>");
            client.print("<input type='text' id='text_5' name='su_sub' class='g-i-d2c5' value='");
            client.print(su_sub);
            client.print("'>");
            client.print("<input type='text' id='text_6' name='su_prim' class='g-i-1ic8' value='");
            client.print(su_prim);
            client.print("'>");
            client.print("<input type='text' id='text_7' name='su_sec' class='g-i-1kpt' value='");
            client.print(su_sec);
            client.print("'>");

            client.print(F("<button type='submit' id='button_9' class='gwd-button-15xv' name='su_save' value=1>Save & Reboot</button></form>"));

            client.println(F("<a href = \"/B\"><button id='button_7' class='gwd-button-15xv gwd-button-10o7'>Cancel & Reboot</button></a>"));

            client.println(F("</body>"));
            client.println(F("</html>"));
            // The HTTP response ends with another blank line:
            client.println();
            client.flush();

            if (requestBody.indexOf("GET /setup?") >= 0) {
              // Extract the parameters from the request
              su_ssid = extractParam(requestBody, "su_ssid");
              su_pw = extractParam(requestBody, "su_pw");
              su_ip = extractParam(requestBody, "su_ip");
              su_gtw = extractParam(requestBody, "su_gtw");
              su_sub = extractParam(requestBody, "su_sub");
              su_prim = extractParam(requestBody, "su_prim");
              su_sec = extractParam(requestBody, "su_sec");

              // Save the parameters
              saveWiFiParameters(su_ssid, su_pw, su_ip, su_gtw, su_sub, su_prim, su_sec);

              BuildScreen(21);
              delay(1000);
              ESP.restart();  // Restart the ESP32
            }

            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else {
          if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }
        }
        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /B")) {
          // reboot
          ESP.restart();
        }
      }
    }
    requestBody = "";
    // close the connection:
    client.stop();
    // Serial.println("Client Disconnected.");
  }
}

// draw the clock icon
void drawClock(int x, int y) {
  // Draw the outer circle for the clock
  LCD_DISPLAY.drawCircle(x, y, 10, TFT_GREEN);   // Outer circle (clock face)

  // Draw fixed hour hand for 4 o'clock (angle = 120 degrees)
  LCD_DISPLAY.drawLine(x, y, x + 2, y + 4, TFT_GREEN);   // Short hour hand (4 o'clock)

  // Draw fixed minute hand for 12 o'clock (angle = 0 degrees)
  LCD_DISPLAY.drawLine(x, y, x, y - 6, TFT_GREEN);       // Longer minute hand (12 o'clock)
}

// remove the clock icon
void eraseClock(int x, int y) {
  // Erase the outer circle for the clock
  LCD_DISPLAY.drawCircle(x, y, 10, TFT_BLACK);   // Outer circle (erased)

  // Erase the hour hand (draw in background color)
  LCD_DISPLAY.drawLine(x, y, x + 2, y + 4, TFT_BLACK);   // Short hour hand (erased)

  // Erase the minute hand (draw in background color)
  LCD_DISPLAY.drawLine(x, y, x, y - 6, TFT_BLACK);       // Longer minute hand (erased)
}
