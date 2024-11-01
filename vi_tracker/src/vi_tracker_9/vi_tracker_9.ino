// Modular tester
// VI tracker module
// Jef Collin 2024


// revisions
// 1.0 first release

// 4 old adc
// 5 new adc, fail
// 6 fail
// 7 update old adc
// 8 average
// 9 sprites




// todo

// note that we are using the old adc and dac drivers since the new libs in esp32 board lib version 3.03 are not stable enough and missing functions such as buffer flush, dac has very coarse steps in new lib



#include <TFT_eSPI.h>
#include <SPI.h>
#include "Free_Fonts.h"
#include "esp_sleep.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include "driver/dac.h"
#include <driver/i2s.h>
#include "soc/syscon_reg.h"
#include "driver/adc.h"
#include <Preferences.h>


// use alps or other decoder
#define Use_Alps_Encoder false

TFT_eSPI LCD_DISPLAY = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

TFT_eSprite LCD_SPRITE1 = TFT_eSprite(&LCD_DISPLAY);
TFT_eSprite LCD_SPRITE2 = TFT_eSprite(&LCD_DISPLAY);

// io pins
// relays
#define vi_ry_1_amplitude_1 27
#define vi_ry_2_amplitude_2 4
#define vi_ry_3_series_r_1 25
#define vi_ry_4_series_r_2 5
#define vi_ry_5_short_ana_y 17
#define vi_ry_6_short_input 16
#define vi_ry_7_ch1_ch2 12

// multiplexer
#define vi_mux_a 13
#define vi_mux_b 19

// LCD
#define Backlight_control 0

// VI tracker
// test voltage 0-3
uint8_t vi_voltage;
// test frequency 0-6
uint8_t vi_frequency;
// resistance range 0-3
uint8_t vi_resistance;
// channel 1 or 12 alternate or 2 (0-2)
uint8_t vi_channelAB;

// discharge input
boolean vi_discharging;
// disconnect from input
boolean vi_connected;

// measurement range 0-3
uint8_t vi_measurement_range;

// enable
unsigned long VItimer;
// channel 1 or 2 active
uint8_t vi_activechannel = 0;
// enable generator
boolean vi_generator_on;
unsigned long VIsampletimer;
uint8_t vi_timebase = 0;

// select setting mode by push buttons
boolean vi_select_r = true;
boolean vi_select_l = true;

// I2S
#define ADC_INPUT (ADC1_CHANNEL_0)
#define ADC_INPUT2 (ADC1_CHANNEL_3)
#define I2S_DMA_BUF_LEN (620)

// create and clear the buffer
uint16_t bufferl[I2S_DMA_BUF_LEN] = {0};

uint8_t graph1_x[128] = {0};
uint8_t graph1_y[128] = {0};
uint8_t graph2_x[128] = {0};
uint8_t graph2_y[128] = {0};

float ADC_matrix[7][4][4];

Preferences preferences;

float ADC_offset1 = 0;
float ADC_offset2 = 0;
float ADC_factor1 = 0;
float ADC_factor2 = 0;

size_t bytes_read;

uint16_t ADC1_raw;
uint16_t ADC2_raw;

float calculated_ADC1;
float calculated_ADC2;

// averaging of samples
uint32_t ADC_average_1;
uint32_t ADC_average_2;
uint8_t ADC_average_counter;

// refresh delay
unsigned long ADC_timer;

// pointer into graph array
uint8_t graph_index = 0;

// graph display mode
// 0 = classic
// 1 = scope mode
uint8_t plot_mode = 0;

// sleep or startup
boolean vi_normalstart = false;

// Menu variables
const int numMenuItems = 3;

// Menu variables
const char* menuItems[] = {"Return", "Calibrate", "Go to sleep"};
int selectedMenu = 0;
int menuOffset = 0;

uint8_t ScreenMode = 0;
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

// rotary encoders io pins
char Encoder_1_Pin1 = 34;
char Encoder_1_Pin2 = 21;
char Encoder_1_Key = 35;

char Encoder_2_Pin1 = 33;
char Encoder_2_Pin2 = 32;
char Encoder_2_Key = 22;

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

  // setup relays
  pinMode(vi_ry_1_amplitude_1, OUTPUT);
  pinMode(vi_ry_2_amplitude_2, OUTPUT);
  pinMode(vi_ry_3_series_r_1, OUTPUT);
  pinMode(vi_ry_4_series_r_2, OUTPUT);
  pinMode(vi_ry_5_short_ana_y, OUTPUT);
  pinMode(vi_ry_6_short_input, OUTPUT);
  pinMode(vi_ry_7_ch1_ch2, OUTPUT);

  // setup mux
  pinMode(vi_mux_a, OUTPUT);
  pinMode(vi_mux_b, OUTPUT);

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
    vi_normalstart = true;
  }

  DisplaySplash();

  if (vi_normalstart) {
    // wait until key is no longer pressed
    while (digitalRead(Encoder_1_Key) == 0) {}
  }

  if (vi_normalstart) {
    delay(4000);
  }
  else {
    delay(2500);
  }

  LCD_DISPLAY.fillScreen(TFT_BLACK);

  // goto sleep
  if (!vi_normalstart) {
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

  vi_discharging = false;
  vi_discharge(vi_discharging);
  vi_channelAB = 0;
  vi_set_channel(vi_channelAB);
  vi_measurement_range = 0;
  vi_set_measurement(vi_measurement_range);
  vi_voltage = 0;
  vi_resistance = 1;
  vi_frequency = 1;

  vi_set_resistance(vi_resistance);
  vi_set_voltage(vi_voltage);

  DAC_Start(1);

  delay(100);

  // Initialize the I2S peripheral
  i2sInit(12800);
  delay(100);

  vi_frequencychange(vi_frequency);

  vi_connected = true;
  vi_connect(vi_connected);

  // attempt to load the matrix from NVS
  if (loadMatrix()) {
  } else {
    // initialize the matrix with default values if data is not present
    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < 4; j++) {
        for (int k = 0; k < 4; k++) {
          ADC_matrix[i][j][k] = 0.0; // Set to default value
        }
      }
    }
    // save the initialized matrix
    saveMatrix();
  }

  GetCalibration();

  LCD_SPRITE1.createSprite(125, 125);
  LCD_SPRITE2.createSprite(160, 128);

  BuildScreen(0);

  // reset switch timer if alt mode
  if (vi_channelAB == 2 or vi_channelAB == 3) {
    VItimer = millis();
  }

}

void loop() {

  switch (ScreenMode) {
    case 0:
      // running mode
      // key 1 short toggle V/F settings, long menu
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
          vi_select_r = !vi_select_r;
          BuildScreen(1);
        }
      }

      // key 2 short switch mode range or sensitivity, long set zero
      if (digitalRead(Encoder_2_Key) == 0 and ScreenMode == 0) {
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

          // nothing for now
          //little debounce
          delay(200);
        }
        else {
          // short press
          vi_select_l = !vi_select_l;
          BuildScreen(1);
        }
      }

      if (EncoderCounter1 != 0 and ScreenMode == 0) {
        if (EncoderCounter1 > 0) {
          if (vi_select_r) {
            if (vi_voltage < 3) {
              vi_voltage++;
              vi_voltagechange(vi_voltage);
              GetCalibration();
            }
          }
          else {
            if (vi_frequency < 6) {
              vi_frequency++;
              vi_frequencychange(vi_frequency);
              GetCalibration();
            }
          }
          EncoderCounter1--;
        }
        else {
          if (vi_select_r) {
            if (vi_voltage > 0) {
              vi_voltage--;
              vi_voltagechange(vi_voltage);
              GetCalibration();
            }
          }
          else {
            if (vi_frequency > 0) {
              vi_frequency--;
              vi_frequencychange(vi_frequency);
              GetCalibration();
            }
          }
          EncoderCounter1++;
        }
        BuildScreen(2);
      }

      if (EncoderCounter2 != 0 and ScreenMode == 0) {
        if (EncoderCounter2 > 0) {
          if (vi_select_l) {
            if (vi_resistance < 3) {
              vi_resistance++;
              vi_resistancechange(vi_resistance);
            }
          }
          else {
            if (vi_channelAB < 3) {
              vi_channelAB++;
              vi_channelchange(vi_channelAB);
            }
          }
          EncoderCounter2--;
        }
        else {
          if (vi_select_l) {
            if (vi_resistance > 0) {
              vi_resistance--;
              vi_resistancechange(vi_resistance);
            }
          }
          else {
            if (vi_channelAB > 0) {
              vi_channelAB--;
              vi_channelchange(vi_channelAB);
            }
          }
          EncoderCounter2++;
        }
        BuildScreen(2);
      }
      break;

    case 1:
      // menu mode
      // check for key pressed
      if (digitalRead(Encoder_1_Key) == 0) {
        switch (selectedMenu) {
          case 0:
            // return
            ScreenMode = 0;
            BuildScreen(ScreenMode);
            break;

          case 1:
            // calibrate
            BuildScreen(3);
            calibrateVI();
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
            if (selectedMenu >= menuOffset + 3) {
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
            if (selectedMenu >= menuOffset + 3) {
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


  if (ScreenMode == 0) {
    calculated_ADC1 = 0;
    calculated_ADC2 = 0;
    graph_index = 0;

    // fill buffer from DMA
    // twice to make sure it is cleared and sampling when idle to avoid noise
    i2s_read(I2S_NUM_0, &bufferl, sizeof(bufferl), &bytes_read, portMAX_DELAY);
    i2s_read(I2S_NUM_0, &bufferl, sizeof(bufferl), &bytes_read, portMAX_DELAY);

    // average
    ADC_average_1 = 0;
    ADC_average_2 = 0;
    ADC_average_counter = 0;

    // split data in 2 channels
    // The 4 high bits are the channel, and the data is inverted
    // for (int i = 0; i < bytes_read / 2; i = i + 2) {

    // skip first part and average 2 samples
    for (int i = 100; i < 600; i = i + 2) {
      if ((bufferl[i] & 0xF000) >> 12 == 0) {
        ADC1_raw = (bufferl[i] & 0x0FFF);
      }
      else {
        ADC2_raw = (bufferl[i] & 0x0FFF);
      }
      if ((bufferl[i + 1] & 0xF000) >> 12 == 0) {
        ADC1_raw = (bufferl[i + 1] & 0x0FFF);
      }
      else {
        ADC2_raw = (bufferl[i + 1] & 0x0FFF);
      }

      ADC_average_1 = ADC_average_1 + ADC1_raw;
      ADC_average_2 = ADC_average_2 + ADC2_raw;
      ADC_average_counter++;

      // done averaging
      if (ADC_average_counter == 2) {
        ADC1_raw = ADC_average_1 / 2;
        ADC2_raw = ADC_average_2 / 2;
        // calibrate to graph
        calculated_ADC1 = (ADC1_raw - ADC_offset1) / ADC_factor1;
        calculated_ADC2 = (ADC2_raw - ADC_offset2) / ADC_factor2;
        // check limits
        if (calculated_ADC1 > 124) {
          calculated_ADC1 = 124;
        }
        if (calculated_ADC1 < 0) {
          calculated_ADC1 = 0;
        }
        if (calculated_ADC2 > 124) {
          calculated_ADC2 = 124;
        }
        if (calculated_ADC2 < 0) {
          calculated_ADC2 = 0;
        }
        // store depending on active channel
        if (vi_activechannel == 0) {
          graph1_y[graph_index] = (uint8_t) calculated_ADC1;
          graph1_x[graph_index++] = (uint8_t) calculated_ADC2;
        }
        else {
          graph2_y[graph_index] = (uint8_t) calculated_ADC1;
          graph2_x[graph_index++] = (uint8_t) calculated_ADC2;
        }
        // reset averages
        ADC_average_1 = 0;
        ADC_average_2 = 0;
        ADC_average_counter = 0;
      }
    }

    // timed update 30ms
    if (millis() - ADC_timer > 30) {

      // graph refresh

      LCD_SPRITE1.fillSprite(TFT_BLACK);

      // LCD_DISPLAY.fillRect(1, 2 , 125, 125, TFT_BLACK);

      LCD_SPRITE1.drawLine(62, 52, 62, 72, TFT_DARKGREY);
      LCD_SPRITE1.drawLine(52, 62, 72, 62, TFT_DARKGREY);

      // Plot the points

      switch (plot_mode) {
        // xy mode
        case 0:
          for (int i = 0; i < 125; i++) {
            if (vi_channelAB == 0 or (vi_channelAB == 2 and vi_activechannel == 0) or vi_channelAB == 3) {
              int xPos = 0 + graph1_x[i];
              int yPos = 124 - graph1_y[i];
              LCD_SPRITE1.drawPixel(xPos, yPos, TFT_GREEN);
            }
            if (vi_channelAB == 1 or (vi_channelAB == 2 and vi_activechannel == 1) or vi_channelAB == 3) {
              int xPos = 0 + graph2_x[i];
              int yPos = 124 - graph2_y[i];
              LCD_SPRITE1.drawPixel(xPos, yPos, TFT_RED);
            }
          }
          break;

        case 1:
          // sine mode
          for (int i = 0; i < 124; i++) {
            int yPos1 = 124 - graph1_x[i];
            int yPos2 = 124 - graph1_x[i + 1]; // Invert y-axis to match screen coordinates
            LCD_SPRITE1.drawLine(i, yPos1, i + 1, yPos2, TFT_GREEN);
            yPos1 = 124 - graph1_y[i];
            yPos2 = 124 - graph1_y[i + 1]; // Invert y-axis to match screen coordinates
            LCD_SPRITE1.drawLine(i, yPos1, i + 1, yPos2, TFT_RED);
          }
          break;
      }

      LCD_SPRITE1.pushSprite(1, 2);

      ADC_timer = millis();
    }
  }

  // channel switching
  if (ScreenMode == 0) {
    // alternate channel mode
    if (vi_channelAB == 2 or vi_channelAB == 3) {
      // check if timer elapsed
      if (millis() - VItimer >= 1000) {
        if (vi_activechannel == 0) {
          vi_activechannel = 1;
        }
        else {
          vi_activechannel = 0;
        }
        vi_set_channel(vi_activechannel);
        // allow relay to settle
        delay(100);

        // reset timer
        VItimer = millis();
        ADC_timer = millis();
      }
    }
  }

  // end of loop
}


// build screen
void BuildScreen(uint8_t updatemode) {
  if (updatemode == 0) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    // graph frame
    LCD_DISPLAY.drawLine(0, 1, 126, 1, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(0, 1, 0, 127, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(0, 127, 126, 127, TFT_DARKGREY);
    LCD_DISPLAY.drawLine(126, 1, 126, 127, TFT_DARKGREY);
  }

  // active selections
  if (updatemode == 0 or updatemode == 1) {
    LCD_DISPLAY.setTextDatum(TC_DATUM);
    if (vi_select_r) {
      LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
      LCD_DISPLAY.drawString("V", 144, 0, 2);
      LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
      LCD_DISPLAY.drawString("F", 144, 32, 2);
    }
    else {
      LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
      LCD_DISPLAY.drawString("V", 144, 0, 2);
      LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
      LCD_DISPLAY.drawString("F", 144, 32, 2);
    }
    if (vi_select_l) {
      LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
      LCD_DISPLAY.drawString("R", 144, 64, 2);
      LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
      LCD_DISPLAY.drawString("CH", 144, 96, 2);
    }
    else {
      LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
      LCD_DISPLAY.drawString("R", 144, 64, 2);
      LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
      LCD_DISPLAY.drawString("CH", 144, 96, 2);
    }
  }

  if (updatemode == 0 or updatemode == 1 or updatemode == 2) {
    LCD_DISPLAY.setTextPadding(32);
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TC_DATUM);
    switch (vi_voltage) {
      case 0:
        LCD_DISPLAY.drawString("0.5", 144, 16, 2);
        break;
      case 1:
        LCD_DISPLAY.drawString("6", 144, 16, 2);
        break;
      case 2:
        LCD_DISPLAY.drawString("10", 144, 16, 2);
        break;
      case 3:
        LCD_DISPLAY.drawString("20", 144, 16, 2);
        break;
    }
    switch (vi_frequency) {
      case 0:
        LCD_DISPLAY.drawString("20", 144, 48, 2);
        break;
      case 1:
        LCD_DISPLAY.drawString("50", 144, 48, 2);
        break;
      case 2:
        LCD_DISPLAY.drawString("100", 144, 48, 2);
        break;
      case 3:
        LCD_DISPLAY.drawString("200", 144, 48, 2);
        break;
      case 4:
        LCD_DISPLAY.drawString("500", 144, 48, 2);
        break;
      case 5:
        LCD_DISPLAY.drawString("1K", 144, 48, 2);
        break;
      case 6:
        LCD_DISPLAY.drawString("2K", 144, 48, 2);
        break;

    }
    switch (vi_resistance) {
      case 0:
        LCD_DISPLAY.drawString("10K", 144, 80, 2);
        break;
      case 1:
        LCD_DISPLAY.drawString("1K", 144, 80, 2);
        break;
      case 2:
        LCD_DISPLAY.drawString("100", 144, 80, 2);
        break;
      case 3:
        LCD_DISPLAY.drawString("10", 144, 80, 2);
        break;
    }
    switch (vi_channelAB) {
      case 0:
        LCD_DISPLAY.drawString("1", 144, 112, 2);
        break;
      case 1:
        LCD_DISPLAY.drawString("2", 144, 112, 2);
        break;
      case 2:
        LCD_DISPLAY.drawString("1/2", 144, 112, 2);
        break;
      case 3:
        LCD_DISPLAY.drawString("1+2", 144, 112, 2);
        break;
    }
    LCD_DISPLAY.setTextPadding(0);
  }

  // calibration busy
  if (updatemode == 3) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setTextSize(1);
    LCD_DISPLAY.setFreeFont(FSS12);
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setTextDatum(MC_DATUM);
    LCD_DISPLAY.drawString("Calibrating...", 79, 63);
  }

}

// splash screen
void DisplaySplash(void ) {
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.drawString("VI tracker", 80, 0);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.drawString("Jef Collin 2024", 80, 25);
  LCD_DISPLAY.drawString("V 1.0", 80, 45);
  // depending on startup mode, sleep or not
  if (vi_normalstart) {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Press R short: toggle V/F", 0, 61);
    LCD_DISPLAY.drawString("Press R long: menu", 0, 71);
    LCD_DISPLAY.drawString("Press L short: toggle R/CH", 0, 81);
    LCD_DISPLAY.drawString("Turn R: set V/F", 0, 91);
    LCD_DISPLAY.drawString("Turn L: set C/CH", 0, 101);
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
  for (int i = 0; i < 3; i++) {
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
  // reset all relays
  vi_shutdownrelays();

  // shutdown the backlight
  digitalWrite(Backlight_control, HIGH);

  // lcd chip to sleep
  LCD_DISPLAY.writecommand(ST7735_SLPIN);
  // Set the wakeup source to the button pin (GPIO 35)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0); // 0 means wake up when the pin is LOW (button pressed)
  // Enter deep sleep mode
  esp_deep_sleep_start();
}

// connect DUT to VI tracker
void vi_connect(boolean connect_dut) {
  if (connect_dut) {
    digitalWrite(vi_ry_6_short_input, HIGH);
  }
  else {
    digitalWrite(vi_ry_6_short_input, LOW);
  }
}

void vi_voltagechange(uint8_t voltage)
{
  // disconnect first
  vi_connect(false);
  // set voltage to lowest value first
  vi_set_voltage(0);
  // set resistance to highest value first
  vi_set_resistance(0);
  // check compatibility
  vi_checkR(voltage);
  vi_set_resistance(vi_resistance);
  vi_set_voltage(voltage);
  vi_connect(vi_connected);
}


// set VI tracker voltage range 0- low  3-high
void vi_set_voltage(uint8_t range) {
  // always set to lowest voltage first
  digitalWrite(vi_ry_1_amplitude_1, LOW);
  digitalWrite(vi_ry_2_amplitude_2, LOW);

  // set measurement to highest reduction
  vi_set_measurement(0);

  // measurement follows output voltage
  switch (range) {
    case 0:
      // 0.5Vpp
      // already on range 0
      vi_set_measurement(3);
      break;

    case 1:
      // 6Vpp
      digitalWrite(vi_ry_2_amplitude_2, HIGH);
      vi_set_measurement(2);
      break;

    case 2:
      // 10Vpp
      digitalWrite(vi_ry_1_amplitude_1, HIGH);
      vi_set_measurement(1);
      break;

    case 3:
      // 20Vpp
      digitalWrite(vi_ry_2_amplitude_2, HIGH);
      digitalWrite(vi_ry_1_amplitude_1, HIGH);
      vi_set_measurement(0);
      break;
  }
}

// set VI tracker measurement range 0- low sensitivity  3-high sensitivity
void vi_set_measurement(uint8_t range) {
  // always set to lowest sensitivity first
  digitalWrite(vi_mux_a, LOW);
  digitalWrite(vi_mux_b, LOW);
  switch (range) {
    case 0:
      // already on range 0
      break;

    case 1:
      digitalWrite(vi_mux_a, HIGH);
      break;

    case 2:
      digitalWrite(vi_mux_b, HIGH);
      break;

    case 3:
      digitalWrite(vi_mux_a, HIGH);
      digitalWrite(vi_mux_b, HIGH);
      break;
  }
}

// set VI tracker resistance 0- low  3-high
void vi_set_resistance(uint8_t range) {
  // always set to highest resistance first
  digitalWrite(vi_ry_3_series_r_1, LOW);
  digitalWrite(vi_ry_4_series_r_2, LOW);
  switch (range) {
    case 0:
      // 10K
      // already on range 0
      break;

    case 1:
      // 1K
      digitalWrite(vi_ry_3_series_r_1, HIGH);
      break;

    case 2:
      // 100
      digitalWrite(vi_ry_4_series_r_2, LOW);
      break;

    case 3:
      // 10
      digitalWrite(vi_ry_4_series_r_2, LOW);
      digitalWrite(vi_ry_3_series_r_1, HIGH);
      break;
  }
}


// VI tracker check range compatibility R based on selected V
void vi_checkR(byte range) {
  switch (range) {
    case 0:
      // 0.5Vpp
      // valid 10-100-1000
      if (vi_resistance == 0) {
        vi_resistance = 1;
      }
      break;

    case 1:
      // 6Vpp
      // valid 100-1000-10000
      if (vi_resistance == 3) {
        vi_resistance = 2;
      }
      break;

    case 2:
      // 10Vpp
      // valid 100-1000-10000
      if (vi_resistance == 3) {
        vi_resistance = 2;
      }
      break;

    case 3:
      // 20Vpp
      // valid 1000-10000
      if (vi_resistance == 2 or vi_resistance == 3) {
        vi_resistance = 1;
      }
      break;
  }
}

void vi_frequencychange(uint8_t frequency)
{
  // double sample rate to allow averaging
  switch (vi_frequency) {
    case 0:
      DAC_SetFrequency(20);
      i2sNewSampleRate(12400);
      break;
    case 1:
      DAC_SetFrequency(50);
      i2sNewSampleRate(24800);
      break;
    case 2:
      DAC_SetFrequency(100);
      i2sNewSampleRate(49600);
      break;
    case 3:
      DAC_SetFrequency(200);
      i2sNewSampleRate(124000);
      break;
    case 4:
      DAC_SetFrequency(500);
      i2sNewSampleRate(248000);
      break;
    case 5:
      DAC_SetFrequency(1000);
      i2sNewSampleRate(496000);
      break;
    case 6:
      DAC_SetFrequency(2000);
      i2sNewSampleRate(992000);
      break;
  }
}

// change measurement series resistance
void vi_resistancechange(uint8_t resistance)
{
  vi_connect(false);
  // set voltage to lowest value first
  vi_set_voltage(0);
  // set resistance to highest value first
  vi_set_resistance(0);
  // check compatibility
  vi_checkV(resistance);
  vi_set_resistance(resistance);
  vi_set_voltage(vi_voltage);
  vi_connect(vi_connected);
}

// VI tracker check range compatibility V based on selected R
void vi_checkV(uint8_t range) {
  switch (range) {
    case 0:
      // 10K
      // valid 6-10-20
      if (vi_voltage == 0) {
        vi_voltage = 1;
      }
      break;

    case 1:
      // 1K
      // valid 0.5-6-10-20
      break;

    case 2:
      // 100
      // valid valid 0.5-6-10
      if (vi_voltage == 3) {
        vi_voltage = 2;
      }
      break;

    case 3:
      // 10
      // valid valid 0.5
      if (vi_voltage == 1 or vi_voltage == 2 or vi_voltage == 3) {
        vi_voltage = 0;
      }
      break;
  }
}

// set VI tracker channel 1 or 2
void vi_set_channel(uint8_t channel) {
  switch (channel) {
    case 0:
      digitalWrite(vi_ry_7_ch1_ch2, LOW);
      break;

    case 1:
      digitalWrite(vi_ry_7_ch1_ch2, HIGH);
      break;
  }
}

// change channel
void vi_channelchange(uint8_t channel) {
  if (channel == 0) {
    vi_activechannel = 0;
    vi_set_channel(vi_activechannel);
  }
  else if (channel == 1) {
    vi_activechannel = 1;
    vi_set_channel(vi_activechannel);
  }
  else if (channel == 2 or channel == 3) {
    // alternate 1-2
    vi_activechannel = 0;
    vi_set_channel(vi_activechannel);
    // reset switch timer
    VItimer = millis();
  }
}

// discharge VI tracker DUT
void vi_discharge(boolean discharge_dut) {
  if (discharge_dut) {
    digitalWrite(vi_ry_5_short_ana_y, HIGH);
  }
  else {
    digitalWrite(vi_ry_5_short_ana_y, LOW);
  }
}

// disable all relays for sleep
void vi_shutdownrelays(void) {
  digitalWrite(vi_ry_1_amplitude_1, LOW);
  digitalWrite(vi_ry_2_amplitude_2, LOW);
  digitalWrite(vi_ry_3_series_r_1, LOW);
  digitalWrite(vi_ry_4_series_r_2, LOW);
  digitalWrite(vi_ry_5_short_ana_y, LOW);
  digitalWrite(vi_ry_6_short_input, LOW);
  digitalWrite(vi_ry_7_ch1_ch2, LOW);
}



// start the dac
void DAC_Start(int scale) {
  // Enable tone generator common to both channels
  SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);
  // Enable / connect tone tone generator on / to this channel
  SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN2_M);
  // Invert MSB, otherwise part of waveform will have inverted
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV2, 2, SENS_DAC_INV2_S);
  // set scale
  //     Scale output of a DAC channel using two bit pattern:
  //     - 00: no scale
  //     - 01: scale to 1/2
  //     - 10: scale to 1/4
  //     - 11: scale to 1/8
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_SCALE2, scale, SENS_DAC_SCALE2_S);
  // enable channel
  dac_output_enable(DAC_CHANNEL_2);
}

// set the dac frequency
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

// setup the I2S ADC DMA
void i2sInit(uint32_t sample_rate) {
  boolean useapll = true;

  if (sample_rate >= 20000 ) {
    useapll = false;
  }

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate =  sample_rate,              // The format of the signal using ADC_BUILT_IN
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = useapll,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  adc1_config_channel_atten(ADC_INPUT, ADC_ATTEN_DB_11);

  adc1_config_channel_atten(ADC_INPUT2, ADC_ATTEN_DB_11);

  adc1_config_width(ADC_WIDTH_BIT_12);

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

  i2s_set_adc_mode(ADC_UNIT_1, ADC_INPUT);

  i2s_adc_enable(I2S_NUM_0);

  // Scan multiple channels.
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_LEN, 1, SYSCON_SARADC_SAR1_PATT_LEN_S);

  // This 32 bit register has 4 bytes for the first set of channels to scan.
  // Each byte consists of:
  // [7:4] Channel
  // [3:2] Bit Width; 3=12bit, 2=11bit, 1=10bit, 0=9bit
  // [1:0] Attenuation; 3=11dB, 2=6dB, 1=2.5dB, 0=0dB
  WRITE_PERI_REG(SYSCON_SARADC_SAR1_PATT_TAB1_REG, 0x0F3F0000);

  // The raw ADC data is written to DMA in inverted form. Invert back.
  SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);

}

// setup new sampling frequency
void i2sNewSampleRate(uint32_t sample_rate) {
  // disable first
  i2s_adc_disable(I2S_NUM_0);
  // stop I2S driver to allow reconfiguration
  i2s_driver_uninstall(I2S_NUM_0);
  // reinitialize I2S with the new sample rate
  i2sInit(sample_rate);
}


// calibrate VI tracker
void calibrateVI(void) {
  vi_connect(false);
  // go through all combinations
  for (byte f = 0; f < 7; f++) {
    for (byte v = 0; v < 4; v++) {
      ADC_calibrate(&ADC_offset1, &ADC_factor1, &ADC_offset2, &ADC_factor2, f, v, 1);
      ADC_matrix[f][v][0] = ADC_offset1;
      ADC_matrix[f][v][1] = ADC_offset2;
      ADC_matrix[f][v][2] = ADC_factor1;
      ADC_matrix[f][v][3] = ADC_factor2;
    }
  }
  // restore settings
  vi_connect(vi_connected);
  vi_discharge(vi_discharging);
  vi_set_channel(vi_channelAB);
  vi_voltagechange(vi_voltage);
  vi_resistancechange(vi_resistance);
  vi_frequencychange(vi_frequency);
  // store calibration
  saveMatrix();
  // restore parameters
  GetCalibration();
}

// calibrate adc channels for each freq - voltage - resistance
void ADC_calibrate(float * offsetpointerch1, float * factorpointerch1, float * offsetpointerch2, float * factorpointerch2, byte frequency, byte voltage, byte resistance) {

  uint16_t adc1_data;
  uint16_t adc2_data;

  float offset1 = 9000;
  float maxval1 = 0;
  float offset2 = 9000;
  float maxval2 = 0;

  vi_discharge(false);

  // set resistance
  vi_set_resistance(resistance);
  // set frequency
  vi_frequencychange(frequency);
  // set voltage
  vi_set_voltage(voltage);

  delay(300);

  // clear the buffer
  bufferl[I2S_DMA_BUF_LEN] = {0};

  // fill buffer from DMA
  i2s_read(I2S_NUM_0, &bufferl, sizeof(bufferl), &bytes_read, portMAX_DELAY);

  // split data in 2 channels
  // The 4 high bits are the channel, and the data is inverted
  for (int i = 50; i < bytes_read / 2; i = i + 2) {
    adc2_data = 0;
    if ((bufferl[i] & 0xF000) >> 12 != 0) {
      adc2_data = (bufferl[i] & 0x0FFF);
    }
    else if ((bufferl[i + 1] & 0xF000) >> 12 != 0) {
      adc2_data = (bufferl[i + 1] & 0x0FFF);
    }
    if (adc2_data != 0) {
      if (adc2_data < offset2) {
        offset2 = adc2_data;
      }
      if (adc2_data > maxval2) {
        maxval2 = adc2_data;
      }
    }
  }
  // adjust to center
  *offsetpointerch2 = offset2 - 50;
  *factorpointerch2 = (float) ((maxval2 - offset2) / 121);

  vi_discharge(true);
  delay(300);

  // clear the buffer
  bufferl[I2S_DMA_BUF_LEN] = {0};

  // fill buffer from DMA
  i2s_read(I2S_NUM_0, &bufferl, sizeof(bufferl), &bytes_read, portMAX_DELAY);
  // split data in 2 channels
  // The 4 high bits are the channel, and the data is inverted
  for (int i = 50; i < bytes_read / 2; i = i + 2) {
    adc1_data = 0;
    if ((bufferl[i] & 0xF000) >> 12 == 0) {
      adc1_data = (bufferl[i] & 0x0FFF);
    }
    else if ((bufferl[i + 1] & 0xF000) >> 12 == 0) {
      adc1_data = (bufferl[i + 1] & 0x0FFF);
    }
    if (adc1_data != 0) {
      if (adc1_data < offset1) {
        offset1 = adc1_data;
      }
      if (adc1_data > maxval1) {
        maxval1 = adc1_data;
      }
    }
  }
  // adjust to center
  *offsetpointerch1 = offset1 - 50;
  *factorpointerch1 = (float) ((maxval1 - offset1) / 121);

  vi_discharge(false);
}

// save calibration
void saveMatrix() {
  preferences.begin("my-app", false); // Open preferences in read-write mode
  // Save a flag indicating data is present
  preferences.putBool("data_present", true);
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 4; k++) {
        // Create a unique key for each value
        String key = "m_" + String(i) + "_" + String(j) + "_" + String(k);
        preferences.putFloat(key.c_str(), ADC_matrix[i][j][k]);
      }
    }
  }
  preferences.end(); // Close preferences
}

// read calibration
bool loadMatrix() {
  preferences.begin("my-app", true); // Open preferences in read-only mode
  // Check if the data is present
  bool dataPresent = preferences.getBool("data_present", false);
  if (dataPresent) {
    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < 4; j++) {
        for (int k = 0; k < 4; k++) {
          // Create the same unique key used during saving
          String key = "m_" + String(i) + "_" + String(j) + "_" + String(k);
          ADC_matrix[i][j][k] = preferences.getFloat(key.c_str(), 0.0); // Default to 0.0 if not found
        }
      }
    }
  }
  preferences.end(); // Close preferences
  return dataPresent;
}

// new calibration parameters are fetched when frequency of voltage changes or calibration is done
void GetCalibration(void) {
  ADC_offset1 = ADC_matrix[vi_frequency][vi_voltage][0];
  ADC_offset2 = ADC_matrix[vi_frequency][vi_voltage][1];
  ADC_factor1 = ADC_matrix[vi_frequency][vi_voltage][2];
  ADC_factor2 = ADC_matrix[vi_frequency][vi_voltage][3];
}
