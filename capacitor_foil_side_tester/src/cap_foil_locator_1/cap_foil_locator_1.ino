// Modular tester
// Capacitor foil locator
// Jef Collin 2024


// revisions
// 1.0 first release


// todo


#include <TFT_eSPI.h>
#include <SPI.h>
#include "Free_Fonts.h"
//#include "esp_timer.h"
//#include <Wire.h>
#include "esp_sleep.h"
#include "driver/adc.h"
#include "soc/syscon_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include <driver/i2s.h>

// use alps or other decoder
#define Use_Alps_Encoder false

TFT_eSPI LCD_DISPLAY = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

TFT_eSprite LCD_SPRITE2 = TFT_eSprite(&LCD_DISPLAY);

// io pins

#define capfoil_switch_polarity    25
#define capfoil_gain_1             26
#define capfoil_gain_2             27

// LCD
#define Backlight_Control 13

// mic analog signal sampling
#define capfoil_audio (ADC1_CHANNEL_0)

// sampling is allways stereo when using i2s so double the buffer size
#define I2S_DMA_BUF_LEN (1024)

boolean Capfoil_Polarity_Normal = true;
uint8_t Capfoil_Gain_Setting = 1;
boolean Capfoil_Scope_Is_On = false;
boolean Capfoil_Running = false;

unsigned long Capfoil_Switchover_Timer;
// ms switch time
uint16_t Capfoil_Switchover_Time = 500;

// sleep or startup
boolean Capfoil_Normalstart = false;

// Menu variables
const int numMenuItems = 3;
// Menu variables
const char* menuItems[] = {"Return", "Scope on", "Go to sleep"};
int selectedMenu = 0;
int menuOffset = 0;
uint8_t ScreenMode = 0;
boolean ForceNewMenu = true;

// for trigger routine
int Scope_Min;
int Scope_Max;

// create raw sample data and clear the buffer
uint16_t Scope_Buffer[I2S_DMA_BUF_LEN] = {0};

size_t Scope_Bytes_Read;

// decoded samples
uint16_t Scope_Samples[I2S_DMA_BUF_LEN];

// keep previous samples to overwrite with background color instead of clearing the screen to reduce flicker
uint16_t Scope_Previous_Samples_Normal[520];
uint16_t Scope_Previous_Samples_Reverse[520];

// trigger point in old samples
int Scope_Previous_Index_Normal = 0;
int Scope_Previous_Index_Reverse = 0;

boolean Scope_Previous_Only_Noise = false;

uint8_t Gauge_Value_Normal = 0;
uint8_t Gauge_Value_Reverse = 0;

long Gauge_Peak_Value = 0;
long Gauge_Smoothed_Peak_Value = 0;

// ignore the first x samples to avoid noise
// the sampling sequences are probably to close after each other, noise is visible in the first part but consistant in duration
// using a delay works but is not a good solution, it introduces lag in the scope display
uint16_t Scope_Ignore_Samples = 120;

// short delay when switching to scope screen to suppress noise
unsigned long Scope_Startup_Delay_timer;

unsigned long Scope_Timer;

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
char Encoder_Pin1 = 39;
char Encoder_Pin2 = 34;
char Encoder_Key_Pin = 35;

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

  // enable only for debugging
  // Serial.begin(115200);

  // setup encoder pins
  pinMode(Encoder_Pin1, INPUT);
  pinMode(Encoder_Pin2, INPUT);
  pinMode(Encoder_Key_Pin, INPUT);

  pinMode(capfoil_switch_polarity, OUTPUT);
  pinMode(capfoil_gain_1, OUTPUT);
  pinMode(capfoil_gain_2, OUTPUT);

  pinMode(Backlight_Control, OUTPUT);

  // set default states
  capfoil_set_polarity(Capfoil_Polarity_Normal);
  capfoil_set_gain(Capfoil_Gain_Setting);

  // turn off backlight
  digitalWrite(Backlight_Control, HIGH);

  // Setup the LCD
  LCD_DISPLAY.init();
  LCD_DISPLAY.setRotation(3);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  delay(100);
  // turn on backlight
  digitalWrite(Backlight_Control, LOW);



  // check for key pressed, enable normal start otherwise go to sleep
  if (digitalRead(Encoder_Key_Pin) == 0) {
    Capfoil_Normalstart = true;
  }

  DisplaySplash();

  if (Capfoil_Normalstart) {
    // wait until key is no longer pressed
    while (digitalRead(Encoder_Key_Pin) == 0) {}
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
  if (!Capfoil_Normalstart) {
    GoToSleep();
  }

  // initialize the I2S peripheral
  i2sInit();

  i2s_stop(I2S_NUM_0);

 LCD_SPRITE2.createSprite(160, 128);
 
  BuildScreen(ScreenMode);
  BuildScreen(10);


  Scope_Timer = millis();
  Capfoil_Switchover_Timer = millis();
}

void loop() {

  // main screen
  if ((ScreenMode == 0 or ScreenMode == 1) and digitalRead(Encoder_Key_Pin) == 0) {
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
      ScreenMode = 2;
      selectedMenu = 0;
      menuOffset = 0;
      EncoderCounter = 0;
      DrawMenu();
      while (digitalRead(Encoder_Key_Pin) == 0) {}
      //little debounce
      delay(200);
    }
    else {
      // short press toggle tester on/off
      Capfoil_Running = !Capfoil_Running;
      BuildScreen(ScreenMode);
      if (ScreenMode == 0) {
        BuildScreen(10);
      }
      else {
        BuildScreen(11);
      }
      // revert to normal if switched off
      if (!Capfoil_Running) {
        Capfoil_Polarity_Normal = true;
        capfoil_set_polarity(Capfoil_Polarity_Normal);
      }
      else {
        Capfoil_Switchover_Timer = millis();
      }
    }
  }

  switch (ScreenMode) {
    case 0:
      // measurement mode
      if (EncoderCounter != 0) {
        if (EncoderCounter > 0) {
          if (Capfoil_Gain_Setting < 3) {
            Capfoil_Gain_Setting++;
            capfoil_set_gain(Capfoil_Gain_Setting);
            BuildScreen(10);
            EncoderCounter--;
          }
          else {
            EncoderCounter = 0;
          }
        }
        else {
          if (Capfoil_Gain_Setting > 0) {
            Capfoil_Gain_Setting--;
            capfoil_set_gain(Capfoil_Gain_Setting);
            BuildScreen(10);
            EncoderCounter++;
          }
          else {
            EncoderCounter = 0;
          }
        }
      }
      break;

    case 1:
      // scope
      if (EncoderCounter != 0) {
        if (EncoderCounter > 0) {
          if (Capfoil_Gain_Setting < 3) {
            Capfoil_Gain_Setting++;
            capfoil_set_gain(Capfoil_Gain_Setting);
            BuildScreen(11);
            EncoderCounter--;
          }
          else {
            EncoderCounter = 0;
          }
        }
        else {
          if (Capfoil_Gain_Setting > 0) {
            Capfoil_Gain_Setting--;
            capfoil_set_gain(Capfoil_Gain_Setting);
            BuildScreen(11);
            EncoderCounter++;
          }
          else {
            EncoderCounter = 0;
          }
        }
      }
      break;

    case 2:
      // menu mode
      // check for key pressed
      if (digitalRead(Encoder_Key_Pin) == 0) {
        switch (selectedMenu) {
          case 0:
            // return
            if (Capfoil_Scope_Is_On) {
              ScreenMode = 1;
            }
            else {
              ScreenMode = 0;
            }

            BuildScreen(ScreenMode);
            // start the timer in case of scope display
            if (ScreenMode == 1) {
              BuildScreen(11);
              Scope_Startup_Delay_timer = millis();
              Scope_Timer = millis();
            }
            else {
              if (ScreenMode == 0) {
                BuildScreen(10);
              }
            }
            break;

          case 1:
            // toggle fullscreen scope
            Capfoil_Scope_Is_On = !Capfoil_Scope_Is_On;
            DrawMenu();
            break;

          case 2:
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

  // polarity switching
  if ((ScreenMode == 0 or ScreenMode == 1) and Capfoil_Running) {
    if (millis() - Capfoil_Switchover_Timer >= Capfoil_Switchover_Time) {
      if (ScreenMode == 1) {
        Scope_Sample();
        // small delay before initial display to suppress noise in initial measurement
        if (millis() - Scope_Startup_Delay_timer > 100) {
          Scope_Show();
        }
      }
      else {
        if (Capfoil_Polarity_Normal) {
          Gauge_Value_Normal = SampleGauge();
          ShowGauge(Gauge_Value_Normal, Capfoil_Polarity_Normal);
        }
        else {
          Gauge_Value_Reverse = SampleGauge();
          ShowGauge(Gauge_Value_Reverse, Capfoil_Polarity_Normal);
        }
        if (Gauge_Value_Normal == Gauge_Value_Reverse) {
          LCD_DISPLAY.fillRect(98, 40, 8, 8, TFT_BLACK);
          LCD_DISPLAY.fillRect(52, 40, 8, 8, TFT_BLACK);
        }
        else {
          if (Gauge_Value_Normal > Gauge_Value_Reverse) {
            LCD_DISPLAY.fillRect(98, 40, 8, 8, TFT_ORANGE);
            LCD_DISPLAY.fillRect(52, 40, 8, 8, TFT_BLACK);
          }
          else {
            LCD_DISPLAY.fillRect(98, 40, 8, 8, TFT_BLACK);
            LCD_DISPLAY.fillRect(52, 40, 8, 8, TFT_ORANGE);
          }
        }
      }
      // switchover at the end for stability
      Capfoil_Polarity_Normal = !Capfoil_Polarity_Normal;
      capfoil_set_polarity(Capfoil_Polarity_Normal);
      // reset timer
      Capfoil_Switchover_Timer = millis();
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
  LCD_DISPLAY.drawString("Cap Foil Check", 80, 0);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.drawString("Jef Collin 2024", 80, 25);
  LCD_DISPLAY.drawString("V 1.0", 80, 45);
  // depending on startup mode, sleep or not
  if (Capfoil_Normalstart) {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Turn knob: gain", 80, 65, 2);
    LCD_DISPLAY.drawString("Press short: on/off", 80, 85, 2);
    LCD_DISPLAY.drawString("Press long: menu", 80, 105, 2);
  }
  else {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Going to sleep", 80, 85, 2);
    LCD_DISPLAY.drawString("Press knob to wakeup", 80, 105, 2);
  }
}

// build screen
void BuildScreen(uint8_t updatemode) {
  if (updatemode == 0) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    // frames
    LCD_DISPLAY.drawLine(4, 4, 24, 4, TFT_LIGHTGREY);
    LCD_DISPLAY.drawLine(4, 4, 4, 123, TFT_LIGHTGREY);
    LCD_DISPLAY.drawLine(24, 4, 24, 123, TFT_LIGHTGREY);
    LCD_DISPLAY.drawLine(4, 123, 24, 123, TFT_LIGHTGREY);
    LCD_DISPLAY.drawLine(135, 4, 155, 4, TFT_LIGHTGREY);
    LCD_DISPLAY.drawLine(135, 4, 135, 123, TFT_LIGHTGREY);
    LCD_DISPLAY.drawLine(155, 4, 155, 123, TFT_LIGHTGREY);
    LCD_DISPLAY.drawLine(135, 123, 155, 123, TFT_LIGHTGREY);

    // cap
    LCD_DISPLAY.fillRect(67, 40, 8, 38, TFT_GREEN);
    LCD_DISPLAY.fillRect(83, 40, 8, 38, TFT_GREEN);
    LCD_DISPLAY.fillRect(52, 57, 15, 6, TFT_GREEN);
    LCD_DISPLAY.fillRect(91, 57, 15, 6, TFT_GREEN);
  }

  if (updatemode == 1) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    // scope axis
    LCD_DISPLAY.drawLine(25, 0, 25, 127, TFT_LIGHTGREY);
    LCD_DISPLAY.drawLine(26, 31, 159, 31, TFT_LIGHTGREY);
    LCD_DISPLAY.drawLine(26, 95, 159, 95, TFT_LIGHTGREY);

    // caps
    LCD_DISPLAY.drawLine(0, 31, 8, 31, TFT_GREEN);
    LCD_DISPLAY.drawLine(15, 31, 23, 31, TFT_GREEN);
    LCD_DISPLAY.drawLine(9, 26, 9, 36, TFT_GREEN);
    LCD_DISPLAY.drawLine(10, 26, 10, 36, TFT_GREEN);
    LCD_DISPLAY.drawLine(13, 26, 13, 36, TFT_GREEN);
    LCD_DISPLAY.drawLine(14, 26, 14, 36, TFT_GREEN);

    LCD_DISPLAY.fillRect(18, 26, 2, 2, TFT_RED);

    LCD_DISPLAY.drawLine(0, 95, 8, 95, TFT_GREEN);
    LCD_DISPLAY.drawLine(15, 95, 23, 95, TFT_GREEN);
    LCD_DISPLAY.drawLine(9, 90, 9, 100, TFT_GREEN);
    LCD_DISPLAY.drawLine(10, 90, 10, 100, TFT_GREEN);
    LCD_DISPLAY.drawLine(13, 90, 13, 100, TFT_GREEN);
    LCD_DISPLAY.drawLine(14, 90, 14, 100, TFT_GREEN);

    LCD_DISPLAY.fillRect(4, 90, 2, 2, TFT_RED);
  }

  // display gain setting
  if (updatemode == 10 or updatemode == 11) {
    String gain_text = "";
    switch (Capfoil_Gain_Setting) {
      case 0:
        // manual mode
        gain_text = "Man";
        break;

      case 1:
        // low gain
        gain_text = "Low";
        break;

      case 2:
        // medium gain
        gain_text = "Med";
        break;

      case 3:
        // high gain
        gain_text = "Hi ";
        break;
    }

    if (updatemode == 10) {
      // main screen
      LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
      LCD_DISPLAY.setFreeFont(GLCD);
      LCD_DISPLAY.setTextDatum(TC_DATUM);
      LCD_DISPLAY.drawString(gain_text, 80, 0);
    }

    if (updatemode == 11) {
      // scope screen
      LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
      LCD_DISPLAY.setFreeFont(GLCD);
      LCD_DISPLAY.setTextDatum(TL_DATUM);
      LCD_DISPLAY.drawString(gain_text, 0, 0);

    }
  }
}

// set gain
void capfoil_set_gain(uint8_t capfoil_gain) {
  // set to min gain first
  digitalWrite(capfoil_gain_1, LOW);
  digitalWrite(capfoil_gain_2, LOW);
  switch (capfoil_gain) {
    case 0:
      //0 = manual
      digitalWrite(capfoil_gain_1, HIGH);
      digitalWrite(capfoil_gain_2, HIGH);
      break;
    case 1:
      //1 = min
      break;
    case 2:
      //2 = medium
      digitalWrite(capfoil_gain_2, HIGH);
      break;
    case 3:
      //3 = max
      digitalWrite(capfoil_gain_1, HIGH);
      break;
  }
}

// switch cap polarity or not
void capfoil_set_polarity(boolean Capfoil_Pol) {
  digitalWrite(capfoil_switch_polarity, !Capfoil_Pol);
}

// draw the menu
void DrawMenu(void) {
  if (Capfoil_Scope_Is_On) {
    menuItems[1] = "Scope off";
  }
  else {
    menuItems[1] = "Scope on";
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
  // disable all relays
  capfoil_set_polarity(true);
  capfoil_set_gain(1);
  // shutdown the backlight
  digitalWrite(Backlight_Control, HIGH);
  // lcd chip to sleep
  LCD_DISPLAY.writecommand(ST7735_SLPIN);
  // Set the wakeup source to the button pin (GPIO 35)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0); // 0 means wake up when the pin is LOW (button pressed)
  // Enter deep sleep mode
  esp_deep_sleep_start();
}

// audio sampling dma over i2s
void Scope_Sample(void) {
  uint8_t Scope_Max_Height = 60;

  int Scope_Buffer_Pointer = 0;
  // reset min-max for trigger
  Scope_Min = 4096;
  Scope_Max = 0;

  // sum all for offset
  unsigned long Scope_Sum = 0;

  i2s_start(I2S_NUM_0);

  // fill buffer from DMA
  // only read what fits on the screen plus some extra for trigger
  i2s_read(I2S_NUM_0, &Scope_Buffer, sizeof(Scope_Buffer), &Scope_Bytes_Read, portMAX_DELAY);

  i2s_stop(I2S_NUM_0);

  // 16 bits, 4 high bits are the channel, and the data is inverted
  // both stereo channels are samples of left channel but shifted in time so we only use 1 in 2 samples
  for (int i = 0; i < Scope_Bytes_Read / 2; i = i + 2) {
    if ((Scope_Buffer[i] & 0xF000) >> 12 == 0) {
      // left channel
      Scope_Samples[Scope_Buffer_Pointer] = Scope_Buffer[i] & 0x0FFF;
      Scope_Sum += Scope_Samples[Scope_Buffer_Pointer];
      Scope_Buffer_Pointer++;
    }
  }

  // calculate offset (average)
  uint16_t Scope_Offset = Scope_Sum / 512;

  // scaling factor to fit to display height
  float ScalingFactor = (float)(Scope_Max_Height / 2) / 1850.0;

  // process the samples
  for (int i = 0; i < 512; i++) {
    // remove the offset scale to fit the display height
    int AdjustedSample = (Scope_Max_Height / 2) + (int)((Scope_Samples[i] - Scope_Offset) * ScalingFactor);
    // allow room for text
    if (AdjustedSample > Scope_Max_Height) {
      AdjustedSample = Scope_Max_Height;
    }
    if (AdjustedSample < 0) {
      AdjustedSample = 0;
    }
    Scope_Samples[i] = (int)AdjustedSample;
    if (Scope_Samples[i] > Scope_Max) {
      Scope_Max = Scope_Samples[i];
    }
    else {
      if (Scope_Samples[i] < Scope_Min) {
        Scope_Min = Scope_Samples[i];
      }
    }
  }
}

// scope
// half screen height
void Scope_Show(void) {

  uint16_t Position_X = 26;

  uint8_t Scope_Max_Width = 157;
  uint8_t Scope_End_Y = 0;
  // position based on polarity
  if (Capfoil_Polarity_Normal) {
    Scope_End_Y = 62;
  }
  else {
    Scope_End_Y = 126;
  }

  // calc threshold
  int Scope_Mid = (Scope_Min + Scope_Max) / 2;

  // skip a part due to noise
  // find where the signal starts
  int Scope_Index = -1;
  // test only the first samples so we still have something to display
  for (int i = 1 + Scope_Ignore_Samples; i < 100 + Scope_Ignore_Samples; i++) {
    if (Scope_Samples[i] >= Scope_Mid && Scope_Samples[i - 1] < Scope_Mid) {
      // threshold crossing detected
      Scope_Index = i;
      break;
    }
  }
  // fallback in case we did not find a trigger in the front
  if (Scope_Index == -1) {
    Scope_Index = 0 + Scope_Ignore_Samples;
  }

  // blank out previous plot with background color unless its the same
  for (uint16_t Scope_Buffer_Pointer = 0; Scope_Buffer_Pointer < Scope_Max_Width ; Scope_Buffer_Pointer++) {
    if (Capfoil_Polarity_Normal) {
      if (Scope_Previous_Samples_Normal[Scope_Previous_Index_Normal + Scope_Buffer_Pointer] != Scope_Samples[Scope_Index + Scope_Buffer_Pointer] or Scope_Previous_Samples_Normal[Scope_Previous_Index_Normal + Scope_Buffer_Pointer + 1] != Scope_Samples[Scope_Index + Scope_Buffer_Pointer + 1]) {
        LCD_DISPLAY.drawLine(Position_X, Scope_End_Y - Scope_Previous_Samples_Normal[Scope_Previous_Index_Normal + Scope_Buffer_Pointer], Position_X + 1, Scope_End_Y - Scope_Previous_Samples_Normal[Scope_Previous_Index_Normal + Scope_Buffer_Pointer + 1], TFT_BLACK);
      }
    }
    else {
      if (Scope_Previous_Samples_Reverse[Scope_Previous_Index_Reverse + Scope_Buffer_Pointer] != Scope_Samples[Scope_Index + Scope_Buffer_Pointer] or Scope_Previous_Samples_Reverse[Scope_Previous_Index_Reverse + Scope_Buffer_Pointer + 1] != Scope_Samples[Scope_Index + Scope_Buffer_Pointer + 1]) {
        LCD_DISPLAY.drawLine(Position_X, Scope_End_Y - Scope_Previous_Samples_Reverse[Scope_Previous_Index_Reverse + Scope_Buffer_Pointer], Position_X + 1, Scope_End_Y - Scope_Previous_Samples_Reverse[Scope_Previous_Index_Reverse + Scope_Buffer_Pointer + 1], TFT_BLACK);
      }
    }
    Position_X++;
  }
  // redraw axis
  LCD_DISPLAY.drawLine(26, 31, 159, 31, TFT_LIGHTGREY);
  LCD_DISPLAY.drawLine(26, 95, 159, 95, TFT_LIGHTGREY);

  // plot
  Position_X = 26;
  for (uint16_t Scope_Buffer_Pointer = 0; Scope_Buffer_Pointer < Scope_Max_Width ; Scope_Buffer_Pointer++) {
    LCD_DISPLAY.drawLine(Position_X, Scope_End_Y - Scope_Samples[Scope_Index + Scope_Buffer_Pointer], Position_X + 1, Scope_End_Y - Scope_Samples[Scope_Index + Scope_Buffer_Pointer + 1], TFT_WHITE);
    Position_X++;
  }

  // keep all 512 measurements
  Scope_Min = 255;
  Scope_Max = 0;

  // both buffers
  for (uint16_t Scope_Buffer_Pointer = 0; Scope_Buffer_Pointer < 512 ; Scope_Buffer_Pointer++) {
    if (Capfoil_Polarity_Normal) {
      Scope_Previous_Samples_Normal[Scope_Buffer_Pointer] = Scope_Samples[Scope_Buffer_Pointer];
    }
    else {
      Scope_Previous_Samples_Reverse[Scope_Buffer_Pointer] = Scope_Samples[Scope_Buffer_Pointer];
    }
    if (Scope_Samples[Scope_Buffer_Pointer] > Scope_Max) {
      Scope_Max = Scope_Samples[Scope_Buffer_Pointer];
    }
    else {
      if (Scope_Samples[Scope_Buffer_Pointer] < Scope_Min) {
        Scope_Min = Scope_Samples[Scope_Buffer_Pointer];
      }
    }
  }
  // keep index
  if (Capfoil_Polarity_Normal) {
    Scope_Previous_Index_Normal = Scope_Index;
  }
  else {
    Scope_Previous_Index_Reverse = Scope_Index;
  }
}

// setup audio sampling
void i2sInit(void) {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    // this will give a little more than 1 cycle on screen
    .sample_rate =  12000,              // The format of the signal using ADC_BUILT_IN
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    //    .dma_buf_len = I2S_DMA_BUF_LEN,
    // max 1024
    .dma_buf_len = 1024,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  adc1_config_channel_atten(capfoil_audio, ADC_ATTEN_DB_11);
  adc1_config_width(ADC_WIDTH_BIT_12);
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, capfoil_audio);
  i2s_adc_enable(I2S_NUM_0);

  // The raw ADC data is written to DMA in inverted form. Invert back.
  SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);
}

// audio sampling dma over i2s
uint8_t SampleGauge(void) {

  uint8_t Gauge_Result = 0;

  uint8_t Scope_Max_Height = 118;

  int Scope_Buffer_Pointer = 0;

  Scope_Max = 0;

  // sum all for offset
  unsigned long Scope_Sum = 0;

  i2s_start(I2S_NUM_0);

  // fill buffer from DMA
  // only read what fits on the screen plus some extra for trigger
  i2s_read(I2S_NUM_0, &Scope_Buffer, sizeof(Scope_Buffer), &Scope_Bytes_Read, portMAX_DELAY);

  i2s_stop(I2S_NUM_0);

  // 16 bits, 4 high bits are the channel, and the data is inverted
  // both stereo channels are samples of left channel but shifted in time so we only use 1 in 2 samples
  for (int i = 0; i < Scope_Bytes_Read / 2; i = i + 2) {
    if ((Scope_Buffer[i] & 0xF000) >> 12 == 0) {
      // left channel
      Scope_Samples[Scope_Buffer_Pointer] = Scope_Buffer[i] & 0x0FFF;
      Scope_Sum += Scope_Samples[Scope_Buffer_Pointer];
      Scope_Buffer_Pointer++;
    }
  }

  // calculate offset (average)
  uint16_t Scope_Offset = Scope_Sum / 512;

  // scaling factor to fit to display height
  float ScalingFactor = (float)(Scope_Max_Height) / 1850.0;

  int AdjustedSample = 0;

  // process the samples
  for (int i = 0; i < 512; i++) {
    // remove the offset scale to fit the display height
    AdjustedSample = (int)((Scope_Samples[i] - Scope_Offset) * ScalingFactor);
    // allow room for text
    if (AdjustedSample > Scope_Max_Height) {
      AdjustedSample = Scope_Max_Height;
    }
    if (AdjustedSample < 0) {
      AdjustedSample = 0;
    }
    Scope_Samples[i] = (int)AdjustedSample;
    if (Scope_Samples[i] > Scope_Max) {
      Scope_Max = Scope_Samples[i];
    }
  }
  Gauge_Result = Scope_Max;

  return Gauge_Result;
}

// gauge
void ShowGauge(uint8_t gauge_value, boolean pol_norm) {
  uint8_t x_pos = 5;
  if (pol_norm) {
    x_pos = 136;
  }
  int maxHeight = 118; // Maximum height inside the frame

  // Draw the bar based on barHeight (filling from the bottom up)
  LCD_DISPLAY.fillRect(x_pos, 123 - gauge_value, 18, gauge_value, TFT_GREEN);

  // Blank the remainder above the bar
  LCD_DISPLAY.fillRect(x_pos, 5, 18, maxHeight - gauge_value, TFT_BLACK);
}
