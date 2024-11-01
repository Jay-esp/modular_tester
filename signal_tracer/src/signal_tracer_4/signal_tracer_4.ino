// Modular tester
// Signal Tracer
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

#define sigtrac_RF       26
#define sigtrac_audio_on 25
#define sigtrac_att_1    32
#define sigtrac_att_2    33

// LCD
#define Backlight_control 27

// mic analog signal sampling
#define sigtrac_audio (ADC1_CHANNEL_0)

// sampling is allways stereo when using i2s so double the buffer size
#define I2S_DMA_BUF_LEN (1024)

boolean Sigtrac_Audio_Is_On = false;
boolean Sigtrac_Rf_Is_On = false;
uint8_t Sigtrac_Attenuator_Setting = 3;
boolean Sigtrac_Scope_Is_On = false;

// sleep or startup
boolean Sigtrac_Normalstart = false;

// Menu variables
const int numMenuItems = 4;
// Menu variables
const char* menuItems[] = {"Return", "Scope on", "RF on", "Go to sleep"};
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
uint16_t Scope_Previous_Samples[520];
// trigger point in old samples
int Scope_Previous_Index = 0;
boolean Scope_Previous_Only_Noise = false;


uint8_t Gauge_Value = 0;
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
  //  Serial.begin(115200);

  // setup encoder pins
  pinMode(Encoder_Pin1, INPUT);
  pinMode(Encoder_Pin2, INPUT);
  pinMode(Encoder_Key_Pin, INPUT);

  pinMode(sigtrac_RF, OUTPUT);
  pinMode(sigtrac_audio_on, OUTPUT);
  pinMode(sigtrac_att_1, OUTPUT);
  pinMode(sigtrac_att_2, OUTPUT);

  pinMode(Backlight_control, OUTPUT);

  // set default states
  sigtrac_set_audio_on(Sigtrac_Audio_Is_On);
  sigtrac_set_rf_mode(Sigtrac_Rf_Is_On);
  sigtrac_set_attenuator(Sigtrac_Attenuator_Setting);

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
  if (digitalRead(Encoder_Key_Pin) == 0) {
    Sigtrac_Normalstart = true;
  }

  DisplaySplash();

  if (Sigtrac_Normalstart) {
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
  if (!Sigtrac_Normalstart) {
    GoToSleep();
  }

  // initialize the I2S peripheral
  i2sInit();

  LCD_SPRITE2.createSprite(160, 128);


  BuildScreen(ScreenMode);
  Scope_Timer = millis();

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
      // short press toggle audio on/off
      Sigtrac_Audio_Is_On = !Sigtrac_Audio_Is_On;
      sigtrac_set_audio_on(Sigtrac_Audio_Is_On);
      // rebuild only this setting
      if (ScreenMode == 0) {
        BuildScreen(10);
      }
      else {
        BuildScreen(20);
      }
    }
  }

  switch (ScreenMode) {
    case 0:
      // measurement mode
      if (EncoderCounter != 0) {
        if (EncoderCounter > 0) {
          if (Sigtrac_Attenuator_Setting > 0) {
            Sigtrac_Attenuator_Setting--;
            sigtrac_set_attenuator(Sigtrac_Attenuator_Setting);
            BuildScreen(11);
            EncoderCounter--;
          }
          else {
            EncoderCounter = 0;
          }
        }
        else {
          if (Sigtrac_Attenuator_Setting < 3) {
            Sigtrac_Attenuator_Setting++;
            sigtrac_set_attenuator(Sigtrac_Attenuator_Setting);
            BuildScreen(11);
            EncoderCounter++;
          }
          else {
            EncoderCounter = 0;
          }
        }
      }





      if (millis() - Scope_Timer > 100) {
        Gauge_Value = SampleGauge();

        // small delay before initial display to suppress noise in initial measurement
        if (millis() - Scope_Startup_Delay_timer > 100) {
          ShowGauge(Gauge_Value);
        }
        Scope_Timer = millis();
      }





      break;

    case 1:
      // scope
      if (EncoderCounter != 0) {
        if (EncoderCounter > 0) {
          if (Sigtrac_Attenuator_Setting > 0) {
            Sigtrac_Attenuator_Setting--;
            sigtrac_set_attenuator(Sigtrac_Attenuator_Setting);
            BuildScreen(21);
            EncoderCounter--;
          }
          else {
            EncoderCounter = 0;
          }
        }
        else {
          if (Sigtrac_Attenuator_Setting < 3) {
            Sigtrac_Attenuator_Setting++;
            sigtrac_set_attenuator(Sigtrac_Attenuator_Setting);
            BuildScreen(21);
            EncoderCounter++;
          }
          else {
            EncoderCounter = 0;
          }
        }
      }

      if (millis() - Scope_Timer > 100) {
        Scope_Sample();
        // small delay before initial display to suppress noise in initial measurement
        if (millis() - Scope_Startup_Delay_timer > 100) {
          Scope_Show();
        }
        Scope_Timer = millis();
      }
      break;

    case 2:
      // menu mode
      // check for key pressed
      if (digitalRead(Encoder_Key_Pin) == 0) {
        switch (selectedMenu) {
          case 0:
            // return
            if (Sigtrac_Scope_Is_On) {
              ScreenMode = 1;
            }
            else {
              ScreenMode = 0;
            }
            // start the timer in case of scope display or gauge
            if (ScreenMode == 0 or ScreenMode == 1) {
              Scope_Startup_Delay_timer = millis();
              Scope_Timer = millis();
            }
            BuildScreen(ScreenMode);
            break;

          case 1:
            // toggle fullscreen scope
            Sigtrac_Scope_Is_On = !Sigtrac_Scope_Is_On;
            DrawMenu();
            break;

          case 2:
            // toggle rf mode
            Sigtrac_Rf_Is_On = !Sigtrac_Rf_Is_On;
            sigtrac_set_rf_mode(Sigtrac_Rf_Is_On);
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

// splash screen
void DisplaySplash(void ) {
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.drawString("Signal Tracer", 80, 0);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.drawString("Jef Collin 2024", 80, 25);
  LCD_DISPLAY.drawString("V 1.0", 80, 45);
  // depending on startup mode, sleep or not
  if (Sigtrac_Normalstart) {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Turn knob: attenuator", 80, 65, 2);
    LCD_DISPLAY.drawString("Press short: audio", 80, 85, 2);
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

  // 0 main
  if (updatemode == 0) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.drawString("Audio:", 0, 0);
    LCD_DISPLAY.drawString("Mode:", 0, 40);
    LCD_DISPLAY.drawString("Attenuation:", 0, 80);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    if (Sigtrac_Rf_Is_On) {
      LCD_DISPLAY.drawString("RF", 60, 40);
    }
    else {
      LCD_DISPLAY.drawString("AF", 60, 40);
    }

    // gauge bar
    LCD_DISPLAY.drawLine(4, 108, 155, 108, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(155, 108, 155, 127, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(4, 108, 4, 127, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(4, 127, 155, 127, TFT_DARKGREY);


  }

  // only settings
  if (updatemode == 10 or updatemode == 0) {
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextPadding(65);
    if (Sigtrac_Audio_Is_On) {
      LCD_DISPLAY.drawString("On", 60, 0);
    }
    else {
      LCD_DISPLAY.drawString("Mute", 60, 0);
    }
    LCD_DISPLAY.setTextPadding(0);
  }

  // only settings
  if (updatemode == 11 or updatemode == 0) {
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TR_DATUM);
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextPadding(60);
    switch (Sigtrac_Attenuator_Setting) {
      case 0:
        LCD_DISPLAY.drawString("0 dB", 159, 80);
        break;

      case 1:
        LCD_DISPLAY.drawString("-20 dB", 159, 80);
        break;

      case 2:
        LCD_DISPLAY.drawString("-40 dB", 159, 80);
        break;

      case 3:
        LCD_DISPLAY.drawString("-60 dB", 159, 80);
        break;

    }
    LCD_DISPLAY.setTextPadding(0);
  }

  if (updatemode == 1) {
    // scope
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.drawLine(0, 10, 0, 127, TFT_LIGHTGREY);
    LCD_DISPLAY.drawLine(0, 68, 159, 68, TFT_LIGHTGREY);

    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TC_DATUM);
    if (Sigtrac_Rf_Is_On) {
      LCD_DISPLAY.drawString("RF", 79, 0);
    }
    else {
      LCD_DISPLAY.drawString("AF", 79, 0);
    }
  }

  // only settings
  if (updatemode == 20 or updatemode == 1) {
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextPadding(65);
    if (Sigtrac_Audio_Is_On) {
      LCD_DISPLAY.drawString("Audio On", 0, 0);
    }
    else {
      LCD_DISPLAY.drawString("Audio Mute", 0, 0);
    }
    LCD_DISPLAY.setTextPadding(0);
  }

  // only settings
  if (updatemode == 21 or updatemode == 1) {
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextPadding(60);
    LCD_DISPLAY.setTextDatum(TR_DATUM);
    switch (Sigtrac_Attenuator_Setting) {
      case 0:
        LCD_DISPLAY.drawString("0 dB", 159, 0);
        break;

      case 1:
        LCD_DISPLAY.drawString("-20 dB", 159, 0);
        break;

      case 2:
        LCD_DISPLAY.drawString("-40 dB", 159, 0);
        break;

      case 3:
        LCD_DISPLAY.drawString("-60 dB", 159, 0);
        break;

    }
    LCD_DISPLAY.setTextPadding(0);
  }

}

// set attenuator
void sigtrac_set_attenuator(uint8_t sigtrac_att) {
  // set to max att first
  digitalWrite(sigtrac_att_1, LOW);
  digitalWrite(sigtrac_att_2, LOW);

  switch (sigtrac_att) {
    case 0:
      //0 = no att
      digitalWrite(sigtrac_att_1, HIGH);
      digitalWrite(sigtrac_att_2, HIGH);
      break;
    case 1:
      //1 = /10
      digitalWrite(sigtrac_att_2, HIGH);
      break;
    case 2:
      //2 = /100
      digitalWrite(sigtrac_att_1, HIGH);
      break;
    case 3:
      //3 = /1000
      break;
  }
}







// set rf or audio mode
void sigtrac_set_rf_mode(boolean Sigtrac_Rfmode) {
  digitalWrite(sigtrac_RF, Sigtrac_Rfmode);
}

// turn audio on or off
void sigtrac_set_audio_on(boolean Sigtrac_Audioon) {
  digitalWrite(sigtrac_audio_on, Sigtrac_Audioon);
}

// draw the menu
void DrawMenu(void) {
  if (Sigtrac_Rf_Is_On) {
    menuItems[2] = "RF off";
  }
  else {
    menuItems[2] = "RF on";
  }

  if (Sigtrac_Scope_Is_On) {
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
  // disable all relais
  sigtrac_set_audio_on(false);
  sigtrac_set_rf_mode(false);
  sigtrac_set_attenuator(3);
  // shutdown the backlight
  digitalWrite(Backlight_control, HIGH);
  // lcd chip to sleep
  LCD_DISPLAY.writecommand(ST7735_SLPIN);
  // Set the wakeup source to the button pin (GPIO 35)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0); // 0 means wake up when the pin is LOW (button pressed)
  // Enter deep sleep mode
  esp_deep_sleep_start();
}

// audio sampling dma over i2s
void Scope_Sample(void) {
  //  int Scope_Scale_Factor = 100;
  uint8_t Scope_Max_Height = 116;

  int Scope_Buffer_Pointer = 0;
  // reset min-max for trigger
  Scope_Min = 4096;
  Scope_Max = 0;

  // sum all for offset
  unsigned long Scope_Sum = 0;

  // fill buffer from DMA
  // only read what fits on the screen plus some extra for trigger
  i2s_read(I2S_NUM_0, &Scope_Buffer, sizeof(Scope_Buffer), &Scope_Bytes_Read, portMAX_DELAY);

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
// 106 pix by 158 large
void Scope_Show(void) {

  uint16_t Position_X = 2;

  uint8_t Scope_Max_Width = 157;
  uint8_t Scope_End_Y = 126;

  // calc threshold
  int Scope_Mid = (Scope_Min + Scope_Max) / 2;

  // skip a part due to noise
  // find where the signal starts
  int Scope_Index = -1;
  // test only the first samples so we still have something to display
  for (int i = 1 + Scope_Ignore_Samples; i < 50 + Scope_Ignore_Samples; i++) {
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

  // check if something changed
  Scope_Min = 255;
  Scope_Max = 0;
  boolean Scope_Change = true;
  for (uint16_t Scope_Buffer_Pointer = 0; Scope_Buffer_Pointer < Scope_Max_Width ; Scope_Buffer_Pointer++) {
    if (Scope_Samples[Scope_Buffer_Pointer] > Scope_Max) {
      Scope_Max = Scope_Samples[Scope_Buffer_Pointer];
    }
    else {
      if (Scope_Samples[Scope_Buffer_Pointer] < Scope_Min) {
        Scope_Min = Scope_Samples[Scope_Buffer_Pointer];
      }
    }
  }
  if (Scope_Max - Scope_Min < 4 and Scope_Previous_Only_Noise) {
    Scope_Change = false;
  }



  if (Scope_Change) {
    // blank out previous plot with background color unless its the same
    for (uint16_t Scope_Buffer_Pointer = 0; Scope_Buffer_Pointer < Scope_Max_Width ; Scope_Buffer_Pointer++) {
      if (Scope_Previous_Samples[Scope_Previous_Index + Scope_Buffer_Pointer] != Scope_Samples[Scope_Index + Scope_Buffer_Pointer] or Scope_Previous_Samples[Scope_Previous_Index + Scope_Buffer_Pointer + 1] != Scope_Samples[Scope_Index + Scope_Buffer_Pointer + 1]) {
        LCD_DISPLAY.drawLine(Position_X, Scope_End_Y - Scope_Previous_Samples[Scope_Previous_Index + Scope_Buffer_Pointer], Position_X + 1, Scope_End_Y - Scope_Previous_Samples[Scope_Previous_Index + Scope_Buffer_Pointer + 1], TFT_BLACK);
      }
      Position_X++;
    }

    LCD_DISPLAY.drawLine(0, 68, 159, 68, TFT_LIGHTGREY);

    // plot
    Position_X = 2;
    for (uint16_t Scope_Buffer_Pointer = 0; Scope_Buffer_Pointer < Scope_Max_Width ; Scope_Buffer_Pointer++) {
      LCD_DISPLAY.drawLine(Position_X, Scope_End_Y - Scope_Samples[Scope_Index + Scope_Buffer_Pointer], Position_X + 1, Scope_End_Y - Scope_Samples[Scope_Index + Scope_Buffer_Pointer + 1], TFT_WHITE);
      Position_X++;
    }

    // keep all 512 measurements

    Scope_Min = 255;
    Scope_Max = 0;

    for (uint16_t Scope_Buffer_Pointer = 0; Scope_Buffer_Pointer < 512 ; Scope_Buffer_Pointer++) {
      Scope_Previous_Samples[Scope_Buffer_Pointer] = Scope_Samples[Scope_Buffer_Pointer];
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
    Scope_Previous_Index = Scope_Index;

    // check if only noise
    if (Scope_Max - Scope_Min < 4) {
      Scope_Previous_Only_Noise = true;
    }
    else {
      Scope_Previous_Only_Noise = false;
    }
  }
}

// setup audio sampling
void i2sInit(void) {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate =  24000,              // The format of the signal using ADC_BUILT_IN
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    //    .dma_buf_len = I2S_DMA_BUF_LEN,
    // max 1024
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  adc1_config_channel_atten(sigtrac_audio, ADC_ATTEN_DB_11);
  adc1_config_width(ADC_WIDTH_BIT_12);
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, sigtrac_audio);
  i2s_adc_enable(I2S_NUM_0);

  // The raw ADC data is written to DMA in inverted form. Invert back.
  SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);

}




// audio sampling dma over i2s
uint8_t SampleGauge(void) {

  Gauge_Peak_Value = 0;

  uint8_t Gauge_Result = 0;

  int Gauge_Buffer_Pointer = 0;
  int Gauge_Max = 0;

  float Gauge_Smoothing_Factor = 0.1; // Adjust this value for desired smoothing

  int Gauge_Window_Size = 10; // Choose an appropriate window size
  int Gauge_Smoothed_Samples[520];

  int Gauge_Limit = 380;

  // sum all for offset
  unsigned long Gauge_Sum = 0;

  // fill buffer from DMA
  i2s_read(I2S_NUM_0, &Scope_Buffer, sizeof(Scope_Buffer), &Scope_Bytes_Read, portMAX_DELAY);

  // 16 bits, 4 high bits are the channel, and the data is inverted
  // both stereo channels are samples of left channel but shifted in time so we only use 1 in 2 samples
  for (int i = 0; i < Scope_Bytes_Read / 2; i = i + 2) {
    if ((Scope_Buffer[i] & 0xF000) >> 12 == 0) {
      // left channel
      Scope_Samples[Gauge_Buffer_Pointer] = Scope_Buffer[i] & 0x0FFF;
      Gauge_Sum += Scope_Samples[Gauge_Buffer_Pointer];
      Gauge_Buffer_Pointer++;
    }
  }

  // calculate offset (average)
  uint16_t Gauge_Offset = Gauge_Sum / 512;


  // smooth samples moving average
  for (int i = 0; i < 512; i++) {
    int sum = 0;
    int count = 0;
    for (int j = max(0, i - Gauge_Window_Size); j <= min(512 - 1, i + Gauge_Window_Size); j++) {
      sum += Scope_Samples[j] - Gauge_Offset;
      count++;
    }
    Gauge_Smoothed_Samples[i] = sum / count;
    if (Gauge_Smoothed_Samples[i] > Gauge_Peak_Value) {
      Gauge_Peak_Value = Gauge_Smoothed_Samples[i];
    }
  }




  // Smooth the peak value
  Gauge_Smoothed_Peak_Value = (int)((Gauge_Smoothing_Factor * Gauge_Peak_Value) + ((1 - Gauge_Smoothing_Factor) * Gauge_Smoothed_Peak_Value));

  // Ensure the smoothed value is within range
  Gauge_Smoothed_Peak_Value = constrain(Gauge_Smoothed_Peak_Value, 0, 2047);


  Serial.print(Gauge_Peak_Value);
  Serial.print(" ");
  Serial.println(Gauge_Smoothed_Peak_Value);




  // calc range 0-25
  long templ = (Gauge_Smoothed_Peak_Value * 25) / Gauge_Limit;

  templ = constrain(templ, 0, 25);

  Gauge_Result = (uint8_t) templ;

  return Gauge_Result;

}


// 3 color gauge
void ShowGauge(uint8_t gauge_value) {
  uint8_t gauge_posx = 5;

  for (uint8_t gauge_ledcount = 1; gauge_ledcount <= 25; gauge_ledcount++) {
    if (gauge_value >= gauge_ledcount) {
      if (gauge_ledcount <= 19) {
        LCD_DISPLAY.fillRect(gauge_posx, 109, 5, 18, TFT_GREEN);
      }
      else {
        if (gauge_ledcount > 19 and gauge_ledcount <= 22) {
          LCD_DISPLAY.fillRect(gauge_posx, 109, 5, 18, TFT_ORANGE);
        }
        else {
          if (gauge_ledcount > 22) {
            LCD_DISPLAY.fillRect(gauge_posx, 109, 5, 18, TFT_RED);
          }
        }
      }
    }
    else {
      // draw blanc
      LCD_DISPLAY.fillRect(gauge_posx, 109, 5, 18, TFT_BLACK);
    }
    gauge_posx = gauge_posx + 6;
  }
}
