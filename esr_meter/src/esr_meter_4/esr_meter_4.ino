// Modular tester
// ESR meter module
// based on Bob Parker's design published in Electronics Australia & Silicon Chip Australia
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

// external storage
#define EEPROM_I2C_ADDRESS 0x50

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

// esr io pins
#define ESR_50ma 32
#define ESR_5ma 25
#define ESR_05ma 26
#define ESR_ramp_ctrl 33
#define ESR_pulse 35
#define ESR_short 27

// LCD
#define Backlight_control 13

// esr current setting
uint8_t esr_current = 0;
// state machine status
uint8_t esr_state = 0;
// counted pulses
uint16_t esr_counter = 0;
// keep counter
uint16_t esr_previouscounter = 9999;
// offset values for each current
uint16_t esr_offset1 = 71;
uint16_t esr_offset2 = 72;
uint16_t esr_offset3 = 76;
// temporary values from eeprom
uint16_t esr_storedoffset1 = 0;
uint16_t esr_storedoffset2 = 0;
uint16_t esr_storedoffset3 = 0;
// rebuild screen or not
//boolean esr_rebuild = true;
// message or error number
uint8_t esr_message = 0;
// flag for message timing
boolean esr_messageflag = false;
// timer for message
unsigned long esr_messagetimer;

// force redisplay
boolean esr_forcedisplay = true;

// sleep or startup
boolean esr_normalstart = false;

// Menu variables
const char* menuItems[] = {"Return", "Go to sleep", "Reset calibration"};
int selectedMenu = 0;
byte ScreenMode = 0;

// Define a handle for the timer
esp_timer_handle_t periodic_timer;

// Interrupt Service Routine (ISR) for the timer
void IRAM_ATTR onTimer(void* arg) {
  // ramp not started
  if (esr_state == 0) {
    // start ramp
    digitalWrite(ESR_ramp_ctrl, LOW);
    // reset counter
    esr_counter = 0;
    esr_state = 1;
  }
  delay_us(2);
  // remove current source short
  digitalWrite(ESR_short, LOW);
  // enable selected current source
  switch (esr_current) {
    case 1:
      digitalWrite(ESR_05ma, LOW);
      break;
    case 2:
      digitalWrite(ESR_5ma, LOW);
      break;
    case 3:
      digitalWrite(ESR_50ma, LOW);
      break;
  }
  // timer ca 8uS total
  // allow pulse to settle
  delay_us(5);
  // sample comparator output
  if (digitalRead(ESR_pulse) == 1 and esr_state == 1) {
    // pulse higher than ramp
    esr_counter++;
    if (esr_counter > 300) {
      // time out
      digitalWrite(ESR_ramp_ctrl, HIGH);
      esr_state = 2;
    }
  }
  else {
    // no more pulses
    if (esr_state == 1) {
      // start cap discharge
      digitalWrite(ESR_ramp_ctrl, HIGH);
      esr_state = 2;
    }
  }
  delay_us(2);
  // shut down current souce
  switch (esr_current) {
    case 1:
      digitalWrite(ESR_05ma, HIGH);
      break;
    case 2:
      digitalWrite(ESR_5ma, HIGH);
      break;
    case 3:
      digitalWrite(ESR_50ma, HIGH);
      break;
  }
  // short source
  digitalWrite(ESR_short, HIGH);
}

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
char Encoder_Pin1 = 36;
char Encoder_Pin2 = 39;
char Encoder_Key_Pin = 34;

// track rotary encoder changes
int EncoderCounter = 0;

unsigned long timer_encoderbutton;

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

void setup() {

  // enable only for debugging, seems to disturb the measurements
  //   Serial.begin(115200);

  Wire.begin(); // Initialize I2C communication

  // setup encoder pins
  pinMode(Encoder_Pin1, INPUT);
  pinMode(Encoder_Pin2, INPUT);
  pinMode(Encoder_Key_Pin, INPUT);

  pinMode(ESR_50ma, OUTPUT);
  pinMode(ESR_5ma, OUTPUT);
  pinMode(ESR_05ma, OUTPUT);
  pinMode(ESR_ramp_ctrl, OUTPUT);
  pinMode(ESR_pulse, INPUT);
  pinMode(ESR_short, OUTPUT);
  pinMode(Backlight_control, OUTPUT);

  // no current
  esr_set_current();

  // turn off backlight
  digitalWrite(Backlight_control, HIGH);

  // discharge ref cap
  digitalWrite(ESR_ramp_ctrl, HIGH);

  // Setup the LCD
  LCD_DISPLAY.init();
  LCD_DISPLAY.setRotation(3);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  delay(100);
  // turn on backlight
  digitalWrite(Backlight_control, LOW);

  // check for key pressed, enable normal start otherwise go to sleep
  if (digitalRead(Encoder_Key_Pin) == 0) {
    esr_normalstart = true;
  }

  DisplaySplash();

  if (esr_normalstart) {
    // wait until key is no longer pressed
    while (digitalRead(Encoder_Key_Pin) == 0) {}
  }

  // get stored calibration values
  esr_storedoffset1 = readEEPROM_uint16(0x00);
  esr_storedoffset2 = readEEPROM_uint16(0x02);
  esr_storedoffset3 = readEEPROM_uint16(0x04);
  if (esr_storedoffset1 > 0 and esr_storedoffset1 < 200) {
    esr_offset1 = esr_storedoffset1;
  }
  if (esr_storedoffset2 > 0 and esr_storedoffset2 < 200) {
    esr_offset2 = esr_storedoffset2;
  }
  if (esr_storedoffset3 > 0 and esr_storedoffset3 < 200) {
    esr_offset3 = esr_storedoffset3;
  }

  delay(2500);
  LCD_DISPLAY.fillScreen(TFT_BLACK);

  // get current state to start otherwise encoder might not react to first click
  unsigned char temppinstate = (digitalRead(Encoder_Pin2) << 1) | digitalRead(Encoder_Pin1);
  Encoder_State1 = ttable[Encoder_State1 & 0xf][temppinstate];

  // setup encoder interrupts
  attachInterrupt(Encoder_Pin1, isr1, CHANGE);
  attachInterrupt(Encoder_Pin2, isr1, CHANGE);

  // goto sleep
  if (!esr_normalstart) {
    GoToSleep();
  }

  // start with highest current
  esr_current = 3;
  esr_set_current();
  digitalWrite(ESR_short, LOW);

  LCD_SPRITE2.createSprite(160, 128);

  BuildScreen(ScreenMode);

  // Create a timer configuration
  const esp_timer_create_args_t periodic_timer_args = {
    .callback = &onTimer,        // Attach ISR function
    .name = "periodic_timer"     // Name of the timer
  };

  // Create the timer
  esp_timer_create(&periodic_timer_args, &periodic_timer);

  // Start the timer with a period of 500 microseconds (500000 nanoseconds)
  esp_timer_start_periodic(periodic_timer, 500); // 500 microseconds

}

void loop() {

  if ((ScreenMode < 7) and digitalRead(Encoder_Key_Pin) == 0) {
    // key short toggle mute, long menu
    timer_encoderbutton = millis();
    Encoder_Key_Long_Press = false;
    // wait until key is no longer pressed or time expired
    while (digitalRead(Encoder_Key_Pin) == 0) {
      if (millis() - timer_encoderbutton > 1000) {
        Encoder_Key_Long_Press = true;
        break;
      }
    }
    if (Encoder_Key_Long_Press) {
      // long press
      ScreenMode = 7;
      selectedMenu = 0;
      DrawMenu();
      while (digitalRead(Encoder_Key_Pin) == 0) {}
      //little debounce
      delay(200);
    }
    else {
      // short press
      if (ScreenMode == 0) {
        esr_reset();
      }
    }
  }

  // scroll through tables or back to measurement
  if (ScreenMode < 7 and EncoderCounter != 0) {
    if (EncoderCounter > 0) {
      if (ScreenMode < 6) {
        ScreenMode++;
        BuildScreen(ScreenMode);
        EncoderCounter--;
      }
      else {
        EncoderCounter = 0;
      }
    }
    else {
      if (ScreenMode > 0) {
        ScreenMode--;
        BuildScreen(ScreenMode);
        EncoderCounter++;
      }
      else {
        EncoderCounter = 0;
      }
    }
    if (ScreenMode == 0) {
      // force display of value
      esr_forcedisplay = true;
    }
  }

  switch (ScreenMode) {
    case 0:
      // measurement mode
      // check for any errors or messages
      if (esr_message != 0) {
        // force rebuild, error message will be displayed
        // force display of value
        esr_forcedisplay = true;
        BuildScreen(ScreenMode);
      }

      // esr meter
      if (esr_state == 2) {
        // measurement is done
        // autoranging
        esr_offset();

        switch (esr_current) {
          case 1:
            // 0.5ma 0-99 ohm
            if (esr_counter > 100) {
              // overflow
              esr_counter = 999;
              esr_show();
              esr_state = 0;
            }
            else {
              if (esr_counter < 10) {
                // switch range
                esr_current = 2;
                esr_set_current();
                esr_state = 0;
              }
              else {
                esr_show();
                esr_state = 0;
              }
            }
            break;

          case 2:
            // 5ma 0-9.9 ohm
            if (esr_counter > 100) {
              // overflow
              esr_current = 1;
              esr_set_current();
              esr_state = 0;
            }
            else {
              if (esr_counter < 10) {
                // switch range
                esr_current = 3;
                esr_set_current();
                esr_state = 0;
              }
              else {
                esr_show();
                esr_state = 0;
              }
            }
            break;

          case 3:
            // 50ma 0-0.99 ohm
            if (esr_counter > 100) {
              // overflow
              esr_current = 2;
              esr_set_current();
              esr_state = 0;
            }
            else {
              esr_show();
              esr_state = 0;
            }
            break;
        }
      }
      else {
        delay(30);
      }
      // check if we have an active message that needs to be cleared
      if (esr_messageflag) {
        // check if timer has elapsed
        if (millis() - esr_messagetimer > 2000) {
          // blank out message
          LCD_DISPLAY.fillRect(0, 0, 159, 15, TFT_BLACK);
          // clear flag
          esr_messageflag = false;
        }
      }
      break;

    case 7:
      // menu mode
      // check for key pressed
      if (digitalRead(Encoder_Key_Pin) == 0) {
        switch (selectedMenu) {
          case 0:
            // return
            ScreenMode = 0;
            BuildScreen(ScreenMode);
            // force display of value
            esr_forcedisplay = true;
            break;

          case 1:
            // sleep
            LCD_DISPLAY.fillScreen(TFT_BLACK);
            // wait until key is no longer pressed
            while (digitalRead(Encoder_Key_Pin) == 0) {}
            // stop timer
            esp_timer_stop(periodic_timer);
            // set to default state
            digitalWrite(ESR_ramp_ctrl, HIGH);
            esr_current = 0;
            esr_set_current();
            digitalWrite(ESR_short, LOW);
            // debounce to avoid early wakeup
            delay(100);
            GoToSleep();
            break;

          case 2:
            // reset values to 0
            esr_offset1 = 0;
            esr_offset2 = 0;
            esr_offset3 = 0;
            SaveCalibration();
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
          if (selectedMenu < 2) {
            selectedMenu++;
            DrawMenu();
          }
          EncoderCounter--;
        }
        if (EncoderCounter < 0) {
          if (selectedMenu > 0) {
            selectedMenu--;
            DrawMenu();
          }
          EncoderCounter++;
        }
      }
      break;
  }
}

// splash screen
void DisplaySplash(void ) {
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.drawString("ESR meter", 80, 0);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.drawString("Jef Collin 2024", 80, 25);
  LCD_DISPLAY.drawString("V 1.0", 80, 45);
  // depending on startup mode, sleep or not
  if (esr_normalstart) {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Turn knob: tables", 80, 65, 2);
    LCD_DISPLAY.drawString("Press short: set 0", 80, 85, 2);
    LCD_DISPLAY.drawString("Press long: menu", 80, 105, 2);
  }
  else {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Going to sleep", 80, 85, 2);
    LCD_DISPLAY.drawString("Press knob to wakeup", 80, 105, 2);
  }
}

void BuildScreen(uint8_t ScreenNumber) {
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  if (ScreenNumber == 0) {
    if (esr_message != 0) {
      switch (esr_message) {
        case 1:
          LCD_DISPLAY.setTextDatum(TC_DATUM);
          LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
          LCD_DISPLAY.drawString("Calibration saved", 80, 0, 2);
          break;
        case 2:
          LCD_DISPLAY.setTextDatum(TC_DATUM);
          LCD_DISPLAY.setTextColor(TFT_RED, TFT_BLACK);
          LCD_DISPLAY.drawString("Calibration error", 80, 0, 2);
          break;
      }
      // reset message
      esr_message = 0;
      // flag that we have a message active
      esr_messageflag = true;
      // start the timer
      esr_messagetimer = millis();
    }
    LCD_DISPLAY.setTextDatum(TC_DATUM);
    LCD_DISPLAY.setTextPadding(0);
    draw20PixBitmap(70, 90, OmegaBitmap, 20, 20, TFT_WHITE);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);

  }

  if (ScreenNumber >= 1 and ScreenNumber <= 6) {
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Cap", 0, 0);
  }
  if (ScreenNumber == 1 or ScreenNumber == 2 or ScreenNumber == 3) {
    LCD_DISPLAY.drawString("1uF", 0, 12);
    LCD_DISPLAY.drawString("2.2uF", 0, 24);
    LCD_DISPLAY.drawString("3.3uF", 0, 36);
    LCD_DISPLAY.drawString("4.7uF", 0, 48);
    LCD_DISPLAY.drawString("10uF", 0, 60);
    LCD_DISPLAY.drawString("22uF", 0, 72);
    LCD_DISPLAY.drawString("33uF", 0, 84);
    LCD_DISPLAY.drawString("47uF", 0, 96);
    LCD_DISPLAY.drawString("100uF", 0, 108);
  }

  if (ScreenNumber == 4 or ScreenNumber == 5 or ScreenNumber == 6) {
    LCD_DISPLAY.drawString("220uF", 0, 12);
    LCD_DISPLAY.drawString("330uF", 0, 24);
    LCD_DISPLAY.drawString("470uF", 0, 36);
    LCD_DISPLAY.drawString("1000uF", 0, 48);
    LCD_DISPLAY.drawString("2200uF", 0, 60);
    LCD_DISPLAY.drawString("3300uF", 0, 72);
    LCD_DISPLAY.drawString("4700uF", 0, 84);
    LCD_DISPLAY.drawString("10000uF", 0, 96);
  }

  if (ScreenNumber == 1 or ScreenNumber == 4) {
    LCD_DISPLAY.drawString("10V", 45, 0);
    LCD_DISPLAY.drawString("16V", 75, 0);
    LCD_DISPLAY.drawString("25V", 105, 0);
    LCD_DISPLAY.drawString("35V", 135, 0);
  }

  if (ScreenNumber == 2 or ScreenNumber == 5) {
    LCD_DISPLAY.drawString("50V", 45, 0);
    LCD_DISPLAY.drawString("63V", 75, 0);
    LCD_DISPLAY.drawString("100V", 105, 0);
    LCD_DISPLAY.drawString("160V", 135, 0);
  }

  if (ScreenNumber == 3 or ScreenNumber == 6) {
    LCD_DISPLAY.drawString("250V", 45, 0);
    LCD_DISPLAY.drawString("350V", 75, 0);
    LCD_DISPLAY.drawString("450V", 105, 0);
  }

  if (ScreenNumber == 1) {
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.drawString("2.4", 135, 12);
    LCD_DISPLAY.drawString("2", 105, 24);
    LCD_DISPLAY.drawString("2.4", 135, 24);
    LCD_DISPLAY.drawString("2", 105, 36);
    LCD_DISPLAY.drawString("2.3", 135, 36);
    LCD_DISPLAY.drawString("2", 105, 48);
    LCD_DISPLAY.drawString("2.2", 135, 48);
    LCD_DISPLAY.drawString("8", 75, 60);
    LCD_DISPLAY.drawString("5.3", 105, 60);
    LCD_DISPLAY.drawString("2.2", 135, 60);
    LCD_DISPLAY.drawString("5.4", 45, 72);
    LCD_DISPLAY.drawString("3.6", 75, 72);
    LCD_DISPLAY.drawString("1.5", 105, 72);
    LCD_DISPLAY.drawString("1.5", 135, 72);
    LCD_DISPLAY.drawString("4.6", 45, 84);
    LCD_DISPLAY.drawString("2", 75, 84);
    LCD_DISPLAY.drawString("1.2", 105, 84);
    LCD_DISPLAY.drawString("1.2", 135, 84);
    LCD_DISPLAY.drawString("2.2", 45, 96);
    LCD_DISPLAY.drawString("1", 75, 96);
    LCD_DISPLAY.drawString("0.9", 105, 96);
    LCD_DISPLAY.drawString("0.7", 135, 96);
    LCD_DISPLAY.drawString("1.2", 45, 108);
    LCD_DISPLAY.drawString("0.7", 75, 108);
    LCD_DISPLAY.drawString("0.3", 105, 108);
    LCD_DISPLAY.drawString("0.3", 135, 108);
  }

  if (ScreenNumber == 2) {
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.drawString("4.5", 45, 12);
    LCD_DISPLAY.drawString("4.5", 75, 12);
    LCD_DISPLAY.drawString("8.5", 105, 12);
    LCD_DISPLAY.drawString("8.5", 135, 12);
    LCD_DISPLAY.drawString("4.5", 45, 24);
    LCD_DISPLAY.drawString("4.5", 75, 24);
    LCD_DISPLAY.drawString("2.3", 105, 24);
    LCD_DISPLAY.drawString("4", 135, 24);
    LCD_DISPLAY.drawString("4.7", 45, 36);
    LCD_DISPLAY.drawString("4.5", 75, 36);
    LCD_DISPLAY.drawString("2.2", 105, 36);
    LCD_DISPLAY.drawString("3.1", 135, 36);
    LCD_DISPLAY.drawString("3", 45, 48);
    LCD_DISPLAY.drawString("3.8", 75, 48);
    LCD_DISPLAY.drawString("2", 105, 48);
    LCD_DISPLAY.drawString("3", 135, 48);
    LCD_DISPLAY.drawString("1.6", 45, 60);
    LCD_DISPLAY.drawString("1.9", 75, 60);
    LCD_DISPLAY.drawString("2", 105, 60);
    LCD_DISPLAY.drawString("1.2", 135, 60);
    LCD_DISPLAY.drawString("0.8", 45, 72);
    LCD_DISPLAY.drawString("0.9", 75, 72);
    LCD_DISPLAY.drawString("1.5", 105, 72);
    LCD_DISPLAY.drawString("1.1", 135, 72);
    LCD_DISPLAY.drawString("0.6", 45, 84);
    LCD_DISPLAY.drawString("0.8", 75, 84);
    LCD_DISPLAY.drawString("1.2", 105, 84);
    LCD_DISPLAY.drawString("1", 135, 84);
    LCD_DISPLAY.drawString("0.5", 45, 96);
    LCD_DISPLAY.drawString("0.6", 75, 96);
    LCD_DISPLAY.drawString("0.7", 105, 96);
    LCD_DISPLAY.drawString("0.5", 135, 96);
    LCD_DISPLAY.drawString("0.3", 45, 108);
    LCD_DISPLAY.drawString("0.4", 75, 108);
    LCD_DISPLAY.drawString("0.15", 105, 108);
    LCD_DISPLAY.drawString("0.3", 135, 108);
  }

  if (ScreenNumber == 3) {
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.drawString("8.7", 45, 12);
    LCD_DISPLAY.drawString("8.5", 75, 12);
    LCD_DISPLAY.drawString("3.6", 105, 12);
    LCD_DISPLAY.drawString("6.1", 45, 24);
    LCD_DISPLAY.drawString("4.2", 75, 24);
    LCD_DISPLAY.drawString("3.6", 105, 24);
    LCD_DISPLAY.drawString("4.6", 45, 36);
    LCD_DISPLAY.drawString("1.6", 75, 36);
    LCD_DISPLAY.drawString("3.3", 105, 36);
    LCD_DISPLAY.drawString("3.5", 45, 48);
    LCD_DISPLAY.drawString("1.6", 75, 48);
    LCD_DISPLAY.drawString("5.65", 105, 48);
    LCD_DISPLAY.drawString("1.4", 45, 60);
    LCD_DISPLAY.drawString("1.2", 75, 60);
    LCD_DISPLAY.drawString("6.5", 105, 60);
    LCD_DISPLAY.drawString("0.7", 45, 72);
    LCD_DISPLAY.drawString("1.1", 75, 72);
    LCD_DISPLAY.drawString("1.5", 105, 72);
    LCD_DISPLAY.drawString("0.5", 45, 84);
    LCD_DISPLAY.drawString("1.1", 75, 84);
    LCD_DISPLAY.drawString("0.4", 45, 96);
    LCD_DISPLAY.drawString("1.1", 75, 96);
    LCD_DISPLAY.drawString("0.2", 45, 108);
  }

  if (ScreenNumber == 4) {
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.drawString("0.6", 45, 12);
    LCD_DISPLAY.drawString("0.3", 75, 12);
    LCD_DISPLAY.drawString("0.25", 105, 12);
    LCD_DISPLAY.drawString("0.2", 135, 12);
    LCD_DISPLAY.drawString("0.24", 45, 24);
    LCD_DISPLAY.drawString("0.2", 75, 24);
    LCD_DISPLAY.drawString("0.25", 105, 24);
    LCD_DISPLAY.drawString("0.1", 135, 24);
    LCD_DISPLAY.drawString("0.24", 45, 36);
    LCD_DISPLAY.drawString("0.18", 75, 36);
    LCD_DISPLAY.drawString("0.12", 105, 36);
    LCD_DISPLAY.drawString("0.1", 135, 36);
    LCD_DISPLAY.drawString("0.12", 45, 48);
    LCD_DISPLAY.drawString("0.15", 75, 48);
    LCD_DISPLAY.drawString("0.08", 105, 48);
    LCD_DISPLAY.drawString("0.1", 135, 48);
    LCD_DISPLAY.drawString("0.09", 45, 60);
    LCD_DISPLAY.drawString("0.07", 75, 60);
    LCD_DISPLAY.drawString("0.06", 105, 60);
    LCD_DISPLAY.drawString("0.05", 135, 60);
    LCD_DISPLAY.drawString("0.09", 45, 72);
    LCD_DISPLAY.drawString("0.07", 75, 72);
    LCD_DISPLAY.drawString("0.06", 105, 72);
    LCD_DISPLAY.drawString("0.05", 135, 72);
    LCD_DISPLAY.drawString("0.04", 45, 84);
    LCD_DISPLAY.drawString("0.03", 75, 84);
    LCD_DISPLAY.drawString("0.03", 105, 84);
    LCD_DISPLAY.drawString("0.02", 135, 84);
    LCD_DISPLAY.drawString("0.02", 45, 96);
    LCD_DISPLAY.drawString("0.02", 75, 96);
    LCD_DISPLAY.drawString("0.01", 105, 96);
    LCD_DISPLAY.drawString("0.01", 135, 96);
  }

  if (ScreenNumber == 5) {
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.drawString("0.2", 45, 12);
    LCD_DISPLAY.drawString("0.1", 75, 12);
    LCD_DISPLAY.drawString("0.1", 105, 12);
    LCD_DISPLAY.drawString("0.2", 135, 12);
    LCD_DISPLAY.drawString("0.2", 45, 24);
    LCD_DISPLAY.drawString("0.1", 75, 24);
    LCD_DISPLAY.drawString("0.1", 105, 24);
    LCD_DISPLAY.drawString("0.2", 135, 24);
    LCD_DISPLAY.drawString("0.1", 45, 36);
    LCD_DISPLAY.drawString("0.1", 75, 36);
    LCD_DISPLAY.drawString("0.1", 105, 36);
    LCD_DISPLAY.drawString("0.1", 135, 36);
    LCD_DISPLAY.drawString("0.1", 45, 48);
    LCD_DISPLAY.drawString("0.1", 75, 48);
    LCD_DISPLAY.drawString("0.1", 105, 48);
    LCD_DISPLAY.drawString("0.1", 135, 48);
    LCD_DISPLAY.drawString("0.05", 45, 60);
    LCD_DISPLAY.drawString("0.04", 75, 60);
    LCD_DISPLAY.drawString("0.04", 105, 60);
    LCD_DISPLAY.drawString("0.03", 135, 60);
    LCD_DISPLAY.drawString("0.05", 45, 72);
    LCD_DISPLAY.drawString("0.04", 75, 72);
    LCD_DISPLAY.drawString("0.04", 105, 72);
    LCD_DISPLAY.drawString("0.03", 135, 72);
    LCD_DISPLAY.drawString("0.02", 45, 84);
    LCD_DISPLAY.drawString("0.02", 75, 84);
    LCD_DISPLAY.drawString("0.02", 105, 84);
    LCD_DISPLAY.drawString("0.02", 135, 84);
    LCD_DISPLAY.drawString("0.01", 45, 96);
    LCD_DISPLAY.drawString("0.01", 75, 96);
    LCD_DISPLAY.drawString("0.01", 105, 96);
    LCD_DISPLAY.drawString("0.01", 135, 96);
  }
  if (ScreenNumber == 6) {
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.drawString("0.2", 45, 12);
    LCD_DISPLAY.drawString("0.2", 45, 24);
    LCD_DISPLAY.drawString("0.2", 45, 36);
    LCD_DISPLAY.drawString("0.15", 45, 48);
    LCD_DISPLAY.drawString("0.1", 45, 60);
    LCD_DISPLAY.drawString("0.03", 45, 72);
    LCD_DISPLAY.drawString("0.01", 45, 84);
    LCD_DISPLAY.drawString("0.01", 45, 96);
  }
}

// set constant current source
void esr_set_current(void) {
  // disable all first
  digitalWrite(ESR_50ma, HIGH);
  digitalWrite(ESR_5ma, HIGH);
  digitalWrite(ESR_05ma, HIGH);
  switch (esr_current) {
    case 0:
      // no current
      break;
    case 1:
      digitalWrite(ESR_05ma, LOW);
      break;
    case 2:
      digitalWrite(ESR_5ma, LOW);
      break;
    case 3:
      digitalWrite(ESR_50ma, LOW);
      break;
  }
}

// subtract offset from measured value
void esr_offset(void) {
  switch (esr_current) {
    case 1:
      // 0.5ma
      if (esr_counter >= esr_offset1) {
        esr_counter = esr_counter - esr_offset1;
      }
      else {
        esr_counter = 0;
      }
      break;
    case 2:
      // 5ma
      if (esr_counter >= esr_offset2) {
        esr_counter = esr_counter - esr_offset2;
      }
      else {
        esr_counter = 0;
      }
      break;
    case 3:
      // 50ma
      if (esr_counter >= esr_offset3) {
        esr_counter = esr_counter - esr_offset3;
      }
      else {
        esr_counter = 0;
      }
      break;
  }
}

// show esr
void esr_show(void) {
  // only if measurement result is different from previous value or forced display
  if (esr_counter != esr_previouscounter or esr_forcedisplay) {
    // overflow
    LCD_DISPLAY.setTextPadding(160);
    if (esr_counter == 999) {
      LCD_DISPLAY.drawString("---", 80, 30, 7);
    }
    else {
      float esr_val = 0;
      // calculate actual cap voltage
      // 10mv/count / gain 20
      esr_val = (float) esr_counter * 0.0005;
      switch (esr_current) {
        case 1:
          // 0.5ma
          esr_val = esr_val / 0.0005;
          LCD_DISPLAY.drawFloat(esr_val, 0, 80, 30, 7);
          break;
        case 2:
          // 5ma
          esr_val = esr_val / 0.005;
          LCD_DISPLAY.drawFloat(esr_val, 1, 80, 30, 7);
          break;
        case 3:
          // 50ma
          esr_val = esr_val / 0.05;
          LCD_DISPLAY.drawFloat(esr_val, 2, 80, 30, 7);
          break;
      }
    }
  }
  // keep current value
  esr_previouscounter = esr_counter;
  // reset
  esr_forcedisplay = false;
}

// calibrate offset for each current
void esr_reset(void)
{
  esr_message = 1;
  long timeout = millis();
  // wait for current measurement to complete
  while (millis() - timeout < 200 or esr_state != 2) {
  }
  esr_current = 1;
  esr_set_current();
  timeout = millis();
  esr_state = 0;
  // wait for current measurement to complete
  while (millis() - timeout < 200 or esr_state != 2) {
  }
  timeout = millis();
  esr_state = 0;
  while (millis() - timeout < 100 or esr_state != 2) {
    if (esr_state == 2) {
      if (esr_counter < 200) {
        esr_offset1 = esr_counter;
      }
      else {
        esr_message = 2;
      }
    }
    else {
      esr_offset1 = 0;
    }
  }
  if (esr_offset1 == 0) {
    esr_message = 2;
  }

  esr_current = 2;
  esr_set_current();
  timeout = millis();
  esr_state = 0;
  // wait for current measurement to complete
  while (millis() - timeout < 200 or esr_state != 2) {
  }
  timeout = millis();
  esr_state = 0;
  while (millis() - timeout < 100 or esr_state != 2) {
    if (esr_state == 2) {
      if (esr_counter < 200) {
        esr_offset2 = esr_counter;
      }
      else {
        esr_message = 2;
      }
    }
    else {
      esr_offset2 = 0;
    }
  }
  if (esr_offset2 == 0) {
    esr_message = 2;
  }

  esr_current = 3;
  esr_set_current();
  timeout = millis();
  esr_state = 0;
  // wait for current measurement to complete
  while (millis() - timeout < 200 or esr_state != 2) {
  }
  timeout = millis();
  esr_state = 0;
  while (millis() - timeout < 100 or esr_state != 2) {
    if (esr_state == 2) {
      if (esr_counter < 200) {
        esr_offset3 = esr_counter;
      }
      else {
        esr_message = 2;
      }
    }
    else {
      esr_offset3 = 0;
    }
  }
  if (esr_offset3 == 0) {
    esr_message = 2;
  }

  esr_current = 1;
  esr_set_current();
  esr_state = 0;
  // only store if succesfull
  if (esr_message == 1) {
    // store in eeprom
    SaveCalibration();
  }
}

// Function to delay for a specified number of microseconds
void delay_us(uint32_t us) {
  int64_t start = esp_timer_get_time();
  while ((esp_timer_get_time() - start) < us) {
    // Wait until the required time has passed
  }
}

// draw the menu
void DrawMenu(void) {
  LCD_SPRITE2.fillSprite(TFT_BLACK);
  LCD_SPRITE2.setTextDatum(TL_DATUM);
  LCD_SPRITE2.setFreeFont(FSS9);
  LCD_SPRITE2.setTextSize(1);
  LCD_SPRITE2.setTextColor(TFT_WHITE, TFT_BLACK);
  for (int i = 0; i < 3; i++) {
    if (i == selectedMenu) {
      LCD_SPRITE2.fillRect(0, 15 + (i * 30), 160, 30, TFT_WHITE);
      LCD_SPRITE2.setTextColor(TFT_BLACK, TFT_WHITE); // Highlight selected line
    } else {
      LCD_SPRITE2.setTextColor(TFT_WHITE, TFT_BLACK);
    }
    LCD_SPRITE2.drawString(menuItems[i], 10, 21 + (i * 30), 1);
  }
  LCD_SPRITE2.pushSprite(0, 0);
}

void GoToSleep (void) {
  // shutdown the backlight
  digitalWrite(Backlight_control, HIGH);
  // lcd chip to sleep
  LCD_DISPLAY.writecommand(ST7735_SLPIN);
  // Set the wakeup source to the button pin (GPIO 34)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0); // 0 means wake up when the pin is LOW (button pressed)
  // Enter deep sleep mode
  esp_deep_sleep_start();
}

// save offset calibration to external eeprom
void SaveCalibration(void) {
  writeEEPROM_uint16(0x00, esr_offset1);
  writeEEPROM_uint16(0x02, esr_offset2);
  writeEEPROM_uint16(0x04, esr_offset3);

  //     Serial.println(esr_offset1);
  //     Serial.println(esr_offset2);
  //    Serial.println(esr_offset3);

}

// write to external eeprom
void writeEEPROM_uint16(uint16_t address, uint16_t data) {
  // Split the uint16_t data into two bytes
  uint8_t dataHigh = (data >> 8) & 0xFF;
  uint8_t dataLow = data & 0xFF;
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((int)(address >> 8)); // MSB of the address
  Wire.write((int)(address & 0xFF)); // LSB of the address
  Wire.write(dataHigh); // Write the high byte
  Wire.write(dataLow); // Write the low byte
  Wire.endTransmission();
  delay(5); // Write cycle time (5ms)
}

// read from external eeprom
uint16_t readEEPROM_uint16(uint16_t address) {
  uint16_t rdata = 0xFFFF; // Default value in case of error
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((int)(address >> 8)); // MSB of the address
  Wire.write((int)(address & 0xFF)); // LSB of the address
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_I2C_ADDRESS, 2); // Request 2 bytes
  if (Wire.available() == 2) {
    uint8_t dataHigh = Wire.read(); // Read the high byte
    uint8_t dataLow = Wire.read(); // Read the low byte
    rdata = (dataHigh << 8) | dataLow; // Combine the bytes into uint16_t
  }
  return rdata;
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
