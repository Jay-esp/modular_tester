// Modular tester
// ring tester
// Jef Collin 2024
// based on Bob Parker's design


// revisions
// 1.0 first release

// first version with scope
// problem with timing of sampling
// plan b is to remove the scope function in version 2 for now


// todo




#include <TFT_eSPI.h>
#include <SPI.h>
#include "Free_Fonts.h"
#include "esp_timer.h"
#include "esp_sleep.h"


// use alps or other decoder
#define Use_Alps_Encoder false

TFT_eSPI LCD_DISPLAY = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

//TFT_eSprite LCD_SPRITE1 = TFT_eSprite(&LCD_DISPLAY);

TFT_eSprite LCD_SPRITE2 = TFT_eSprite(&LCD_DISPLAY);

// io pins
#define ring_button 33
#define ring_pulse 25
#define ring_detect 26

// not used but set to input just in case
#define ring_scope 39


// LCD
#define Backlight_control 13


// sleep or startup
boolean ring_normalstart = false;

// enable counting interrupt
boolean ring_enable_count = false;

// actual rings
uint32_t ring_count = 0;


// Menu variables

const int numMenuItems = 2;
const char* menuItems[] = {"Return", "Go to sleep"};
int selectedMenu = 0;
int menuOffset = 0;

// screen mode
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
char Encoder_Pin1 = 32;
char Encoder_Pin2 = 35;
char Encoder_Key_Pin = 34;

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


// interrupt routine for ring counter
void IRAM_ATTR ring_int() {
  if (ring_enable_count) {
    ring_count++;
  }
}


void setup() {

  // enable only for debugging
  //Serial.begin(115200);

  // setup encoder pins
  pinMode(Encoder_Pin1, INPUT);
  pinMode(Encoder_Pin2, INPUT);
  pinMode(Encoder_Key_Pin, INPUT);

  pinMode(ring_button, INPUT);
  pinMode(ring_detect, INPUT);

  // not used for now so just make it input
  pinMode(ring_scope, INPUT);

  pinMode(ring_pulse, OUTPUT);
  digitalWrite(ring_pulse, LOW);

  pinMode(Backlight_control, OUTPUT);

  digitalWrite(Backlight_control, HIGH);

  // Setup the LCD
  LCD_DISPLAY.init();
  LCD_DISPLAY.setRotation(3);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  delay(100);
  // turn on backlight
  digitalWrite(Backlight_control, LOW);

  // check for key pressed, enable normal start otherwise go to sleep
  if (digitalRead(Encoder_Key_Pin) == 0) {
    ring_normalstart = true;
  }

  DisplaySplash();

  if (ring_normalstart) {
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
  if (!ring_normalstart) {
    GoToSleep();
  }


  LCD_SPRITE2.createSprite(160, 128);

  LCD_DISPLAY.fillScreen(TFT_BLACK);

  BuildScreen(ScreenMode);
  update_rings();

    attachInterrupt(ring_detect, ring_int, CHANGE);

}

void loop() {

  // button pressed
  if (digitalRead(ring_button) == 0) {
    ring_enable_count = false;
    ring_count = 0;

    digitalWrite(ring_pulse, LOW);
    delay(30);
    digitalWrite(ring_pulse, HIGH);
    ring_enable_count = true;

    delay(100);

    ring_enable_count = false;

    update_rings();

    while (digitalRead(ring_button) == 0) {
    }
    delay(10);
  }




  // encoder button pressed
  if (ScreenMode == 0 and digitalRead(Encoder_Key_Pin) == 0) {
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
      EncoderCounter = 0;
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

    }
  }





  switch (ScreenMode) {
    case 0:
      //

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
            ring_count = 0;
            update_rings();
            break;

          case 1:
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

// splash screen
void DisplaySplash(void ) {
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.drawString("Ring Tester", 80, 0);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.drawString("Jef Collin 2024", 80, 25);
  LCD_DISPLAY.drawString("V 1.0", 80, 45);
  // depending on startup mode, sleep or not
  if (ring_normalstart) {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Press R long: menu", 0, 71);
    LCD_DISPLAY.drawString("Button: measure", 0, 91);
  }
  else {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Going to sleep", 80, 65, 2);
    LCD_DISPLAY.drawString("Press knob to wakeup", 80, 105, 2);
  }
}

void BuildScreen(uint8_t ScreenNumber) {
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  if (ScreenNumber == 0) {
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
    LCD_DISPLAY.drawString("Rings", 0, 0);
  }


}


void update_rings(void) {
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(140);
  LCD_DISPLAY.drawFloat(ring_count, 0, 0, 48, 6);
  LCD_DISPLAY.setTextPadding(0);
}


// draw the menu
void DrawMenu(void) {
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
  // Set the wakeup source to the button pin (GPIO 34)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0); // 0 means wake up when the pin is LOW (button pressed)
  // Enter deep sleep mode
  esp_deep_sleep_start();
}
