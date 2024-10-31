// Modular tester
// CB microphone tester module
// Jef Collin 2024


// revisions
// 1.0 first release




// todo


// test accurary with new adc speed

// check forcenewmenu use



#include <TFT_eSPI.h>
#include <SPI.h>
#include "Free_Fonts.h"
#include "esp_sleep.h"
#include <Wire.h>
#include "ADS1X15.h"
#include <Adafruit_MCP23X17.h>
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include <driver/i2s.h>
#include "soc/syscon_reg.h"
#include "driver/adc.h"
#include <driver/dac_oneshot.h>
#include <driver/dac_cosine.h>

#include "DIN_1451_Mittelschrift_Regular8pt7b.h"

// mic analog signal sampling
#define cb_mic_input (ADC1_CHANNEL_0)

// sampling is allways stereo when using i2s so double the buffer size
#define I2S_DMA_BUF_LEN (1024)

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

// cb tester io pins
#define cb_rx_tx_relay 4
#define cb_ptt_input 27

// LCD
#define Backlight_control 13

// io extender pins 0-15
#define IO_mux_A 6
#define IO_mux_B 5
#define IO_mux_C 4

#define IO_pin1 2
#define IO_pin1_R 3
#define IO_pin2 0
#define IO_pin2_R 1
#define IO_pin3 9
#define IO_pin3_R 8
#define IO_pin4 11
#define IO_pin4_R 10
#define IO_pin5 13
#define IO_pin5_R 12
#define IO_pin6 15
#define IO_pin6_R 14

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

// ignore the first x samples to avoid noise
// the sampling sequences are probably to close after each other, noise is visible in the first part but consistant in duration
// using a delay works but is not a good solution, it introduces lag in the scope display
uint16_t Scope_Ignore_Samples = 120;

// short delay when switching to scope screen to suppress noise
unsigned long Scope_Startup_Delay_timer;

// mike model
uint8_t cb_mike_model = 1;

// parameters per model
// model 0 is manual mode, reserved for future use
// pin model
// 4 = 4 pin gx16
// 5 = 5 pin gx16
// 6 = 6 pin gx16
// 15 = 5 pin din

uint8_t cb_pincount[] = {4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 15, 15};

// functions per pin
// 0 = pin is not used / not connected
// 1 = shield (only for mike)
// 2 = common (for rx tx)
// 3 = combined shield/common
// 4 = mike
// 5 = rx
// 6 = tx
// 7 = other functions like up down buttons
// 8 = power for buttons etc (future features)

uint8_t cb_pin1function[] = {0, 4, 3, 4, 5, 6, 3, 4, 4, 4, 3, 3, 7, 0, 4, 4, 4, 4, 0, 3, 7, 3, 3, 4};
uint8_t cb_pin2function[] = {0, 3, 4, 3, 6, 3, 3, 1, 0, 3, 4, 4, 7, 5, 5, 3, 0, 0, 3, 4, 7, 7, 0, 6};
uint8_t cb_pin3function[] = {0, 5, 6, 6, 4, 4, 6, 5, 6, 6, 0, 6, 6, 6, 3, 0, 3, 3, 6, 6, 6, 6, 6, 0};
uint8_t cb_pin4function[] = {0, 6, 5, 5, 3, 3, 4, 2, 5, 5, 5, 5, 3, 3, 0, 0, 0, 5, 0, 5, 4, 5, 4, 3};
uint8_t cb_pin5function[] = {0, 0, 0, 0, 0, 0, 0, 6, 3, 0, 6, 0, 4, 4, 6, 6, 6, 6, 4, 7, 3, 4, 5, 5};
uint8_t cb_pin6function[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 7, 7, 0, 0};

// pin settings
// 0 = float (input)
// 1 = low
// 2 = high via R 681 ohm

uint8_t cb_pinstatus1 = 0;
uint8_t cb_pinstatus2 = 0;
uint8_t cb_pinstatus3 = 0;
uint8_t cb_pinstatus4 = 0;
uint8_t cb_pinstatus5 = 0;
uint8_t cb_pinstatus6 = 0;

// measured resistance versus common or combined shield/common pin
uint32_t cb_pinresistance1 = 0;
uint32_t cb_pinresistance2 = 0;
uint32_t cb_pinresistance3 = 0;
uint32_t cb_pinresistance4 = 0;
uint32_t cb_pinresistance5 = 0;
uint32_t cb_pinresistance6 = 0;

// the mcp23017 has an internal resistance, compensation in the calculations is required
// this values is taken as an average from a range of resistance measurements but focussing on the lower resistance range
uint32_t cb_internal_resistance = 50;

// pin connected to audio measurement input via mux
// 0 = ground
// 1-6 pins
uint8_t cb_mic_pin = 0;

// modulation mode/frequency
uint8_t cb_modulation = 0;

// in tx mode?
boolean cb_TXmodeactive = false;

// sleep or startup
boolean cb_normalstart = false;

// DAC oneshot init
// we use dac 1 to generate the second sine wave in case of a dual frequency mode
// dac 2 is used in cosine mode because dac 1 generates spikes in the signal in this mode
// cosine mode can only be done on the same frequency for both DAC channels so we handle channel 1 differently in single shot mode
dac_oneshot_handle_t chan0_handle;
dac_oneshot_config_t chan0_cfg = {
  .chan_id = DAC_CHAN_0,
};
// sine wave table for oneshot mode
int SineTable[256];

// handle for the cosine mode on dac 2
dac_cosine_handle_t dac_chan_handle;

// Menu variables
const int numMenuItems = 3;

// Menu variables
// tx/rx mode is switched dynamically later
const char* menuItems[] = {"Return", "TX mode", "Go to sleep"};
int selectedMenu = 0;
int menuOffset = 0;

uint8_t ScreenMode = 0;

boolean ForceNewMenu = true;

// U3 ADC
ADS1115 ADS2(0x48);
// U2 ADC
ADS1115 ADS1(0x49);

// port expander
Adafruit_MCP23X17 mcp;



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

// rotary encoders io pins
char Encoder_1_Pin1 = 34;
char Encoder_1_Pin2 = 39;
char Encoder_1_Key = 35;

// track rotary encoder changes
int EncoderCounter1 = 0;

unsigned long timer_encoderbutton1;

unsigned long timer_lastencoderactivity;

boolean Encoder_Key1_Long_Press = false;

// interrupt routine for rotary encoder
// rotations are buffered but limited to 20 changes
void IRAM_ATTR isr1() {
  unsigned char pinstate = (digitalRead(Encoder_1_Pin2) << 1) | digitalRead(Encoder_1_Pin1);
  Encoder_1_State = ttable[Encoder_1_State & 0xf][pinstate];
  unsigned char result = Encoder_1_State & 0x30;
  if (result == DIR_CW) {
    if (EncoderCounter1 < 20) {
      EncoderCounter1++;
    }
  } else if (result == DIR_CCW) {
    if (EncoderCounter1 > -20) {
      EncoderCounter1--;
    }
  }
}


void setup() {

  Serial.begin(115200);

  // setup encoder pins
  pinMode(Encoder_1_Pin1, INPUT);
  pinMode(Encoder_1_Pin2, INPUT);
  pinMode(Encoder_1_Key, INPUT);

  // setup i2c interface
  Wire.begin();

  // setup relay
  pinMode(cb_rx_tx_relay, OUTPUT);
  // relay in rx mode
  Transmit(false);

  // tx button
  pinMode(cb_ptt_input, INPUT_PULLUP);

  mcp.begin_I2C();
  // mux control
  mcp.pinMode(IO_mux_A, OUTPUT);
  mcp.pinMode(IO_mux_B, OUTPUT);
  mcp.pinMode(IO_mux_C, OUTPUT);

  // ground audio input
  CB_SelectAudioPin(0);

  // set all pins as input for now, floating
  mcp_release_pins();

  pinMode(Backlight_control, OUTPUT);

  // turn off backlight
  digitalWrite(Backlight_control, HIGH);

  // setup the LCD
  LCD_DISPLAY.init();
  LCD_DISPLAY.setRotation(3);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  delay(100);
  // turn on backlight
  digitalWrite(Backlight_control, LOW);

  // check for key pressed, enable normal start otherwise go to sleep
  if (digitalRead(Encoder_1_Key) == 0) {
    cb_normalstart = true;
  }

  DisplaySplash();

  if (cb_normalstart) {
    // wait until key is no longer pressed
    while (digitalRead(Encoder_1_Key) == 0) {}
  }

  if (cb_normalstart) {
    delay(4000);
  }
  else {
    delay(2500);
  }

  LCD_DISPLAY.fillScreen(TFT_BLACK);

  // goto sleep
  if (!cb_normalstart) {
    GoToSleep();
  }

  // adc setup
  ADS1.begin();
  ADS1.setGain(0);      //
  ADS1.setDataRate(6);  //
  ADS1.readADC(0);      // first read to trigger
  ADS1.setMode(1);      // single mode
  ADS1.readADC(0);      // first read to trigger

  ADS2.begin();
  ADS2.setGain(0);      //
  ADS2.setDataRate(6);  //
  ADS2.readADC(0);      // first read to trigger
  ADS2.setMode(1);      // single mode
  ADS2.readADC(0);      // first read to trigger

  // get current state to start otherwise encoder might not react to first click
  unsigned char temppinstate1 = (digitalRead(Encoder_1_Pin2) << 1) | digitalRead(Encoder_1_Pin1);
  Encoder_1_State = ttable[Encoder_1_State & 0xf][temppinstate1];

  // setup encoder interrupts
  attachInterrupt(Encoder_1_Pin1, isr1, CHANGE);
  attachInterrupt(Encoder_1_Pin2, isr1, CHANGE);

  LCD_SPRITE2.createSprite(160, 128);

  
  BuildScreen(0);

  cb_SetAllPins(cb_mike_model);

  // initialize the I2S peripheral
  i2sInit();

  // setup sine table for oneshot mode on dac 1
  float ConversionFactor = (2 * PI) / 256;  // convert my 0-255 bits in a circle to radians
  // there are 2 x PI radians in a circle hence the 2*PI
  // Then divide by 256 to get the value in radians
  // for one of my 0-255 bits.
  float RadAngle;                           // Angle in Radians
  // calculate sine values
  for (int MyAngle = 0; MyAngle < 256; MyAngle++) {
    RadAngle = MyAngle * ConversionFactor;           // 8 bit angle converted to radians
    SineTable[MyAngle] = (sin(RadAngle) * 127) + 128; // get the sine of this angle and 'shift' up so
    // there are no negative values in the data
    // as the DAC does not understand them and would
    // convert to positive values.
  }


}

void loop() {

  // main screen
  if ((ScreenMode == 0 or ScreenMode == 1 or ScreenMode == 2  or ScreenMode == 4) and digitalRead(Encoder_1_Key) == 0) {
    // key 1 short reset, long menu
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
      ScreenMode = 3;
      selectedMenu = 0;
      menuOffset = 0;
      EncoderCounter1 = 0;
      DrawMenu();
      while (digitalRead(Encoder_1_Key) == 0) {}
      //little debounce
      delay(200);
    }
    else {
      // short press, ignore in rx/tx mode
      if (ScreenMode != 4) {
        // cycle screens
        if (ScreenMode < 2) {
          ScreenMode++;
        }
        else {
          ScreenMode = 0;
        }
        Status_Clear();

        // build screen
        BuildScreen(ScreenMode);

        // start the timer in case of scope display
        if (ScreenMode == 2) {
          Scope_Startup_Delay_timer = millis();
        }
      }
    }
  }



  switch (ScreenMode) {
    case 0:
      // running mode main screen
      // change mike model if encoder turned
      if (EncoderCounter1 != 0) {
        if (EncoderCounter1 > 0) {
          if (cb_mike_model < 23) {
            cb_mike_model++;
            cb_SetAllPins(cb_mike_model);
            BuildScreen(0);
            EncoderCounter1--;
          }
          else {
            EncoderCounter1 = 0;
          }
        }
        else {
          if (cb_mike_model > 1) {
            cb_mike_model--;
            cb_SetAllPins(cb_mike_model);
            BuildScreen(0);
            EncoderCounter1++;
          }
          else {
            EncoderCounter1 = 0;
          }
        }
        // we keep the time to allow swift changes through the models while postponing the measurements until encoder stops for a defined time
        timer_lastencoderactivity = millis();
      }

      // hold off measurements until encoder stops for x mSec
      if (EncoderCounter1 == 0 and millis() - timer_lastencoderactivity > 300) {
        CB_ShowPinActivity(false);
      }
      break;

    case 1:
      // main screen show ohm values
      // ignore model selection
      if (EncoderCounter1 != 0) {
        EncoderCounter1 = 0;
      }
      CB_ShowPinActivity(true);
      break;

    case 2:
      // main screen scope
      // ignore model selection
      if (EncoderCounter1 != 0) {
        EncoderCounter1 = 0;
      }
      Scope_Sample();
      // small delay before initial display to suppress noise in initial measurement
      if (millis() - Scope_Startup_Delay_timer > 100) {
        Scope_Show();
      }
      break;

    case 3:
      // menu mode
      // check for key pressed
      if (digitalRead(Encoder_1_Key) == 0) {
        switch (selectedMenu) {
          case 0:
            // return
            if (cb_TXmodeactive) {
              ScreenMode = 4;
            }
            else {
              ScreenMode = 0;
            }
            BuildScreen(ScreenMode);
            break;

          case 1:
            // toggle tx/rx mode
            if (cb_TXmodeactive) {
              ScreenMode = 0;
              cb_TXmodeactive = false;
            }
            else {
              ScreenMode = 4;
              cb_TXmodeactive = true;
              cb_modulation = 0;
            }
            BuildScreen(ScreenMode);
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

    case 4:
      // tx mode
      // encoder selects modulation mode
      if (EncoderCounter1 != 0) {
        if (EncoderCounter1 > 0) {
          if (cb_modulation < 5) {
            cb_modulation++;
            Update_Modulation(cb_modulation);
            EncoderCounter1--;
          }
          else {
            EncoderCounter1 = 0;
          }
        }
        else {
          if (cb_modulation > 0) {
            cb_modulation--;
            Update_Modulation(cb_modulation);
            EncoderCounter1++;
          }
          else {
            EncoderCounter1 = 0;
          }
        }
      }
      // transmit button is pressed
      if (digitalRead(cb_ptt_input) == 0) {
        // set frequency of the cosine dac and start it
        // due to inaccuracy and large internal steps the settings is a little off to get as close as possible to the target value
        switch (cb_modulation) {
          case 0:
            DAC_Stop();
            break;

          case 1:
            DAC_Start(1050);
            break;

          case 2:
            DAC_Start(525);
            break;

          case 3:
            DAC_Start(2400);
            break;

          case 4:
            DAC_Start(2500);
            break;

          case 5:
            DAC_Start(2400);
            break;

        }

        Update_RX_TX(true);
        // relay on
        Transmit(true);

        // to allow cascading loop and while exit
        boolean breakout = false;

        // loop until button is depressed
        while (digitalRead(cb_ptt_input) == 0) {
          // in case of dual frequency dac2 is handled by cw, dac1 needs to be run manually
          if (cb_modulation == 5) {
            dac_oneshot_new_channel(&chan0_cfg, &chan0_handle);
            while (1) {
              // cycle through sine table
              for (int i = 0; i < 256; i++) {
                dac_oneshot_output_voltage(chan0_handle, SineTable[i]);
                // adjust to get as close as possible to 500Hz
                delayMicroseconds(4);
                // stop if transmit button is no longer pressed
                if (digitalRead(cb_ptt_input) != 0) {
                  breakout = true;
                  break;
                }
              }
              if (breakout) {
                break;
              }
            }
            dac_oneshot_del_channel(chan0_handle);
          }
          if (breakout) {
            break;
          }
        }

        Transmit(false);
        Update_RX_TX(false);
        if (cb_modulation != 0) {
          DAC_Stop();
        }
        //little debounce
        delay(200);
      }
      break;

  }


}


// build screen
void BuildScreen(uint8_t updatemode) {

  // 0 main or ohm
  if (updatemode == 0 or updatemode == 1) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);

    // draw connector

    uint8_t center_x = 40;
    uint8_t center_y = 100;
    LCD_DISPLAY.fillCircle(center_x, center_y, 24, TFT_BLUE);
    LCD_DISPLAY.fillCircle(center_x, center_y + 24, 4, TFT_BLACK);

    switch (cb_pincount[cb_mike_model]) {
      case 4:
        // 4 pin gx16
        LCD_DISPLAY.fillCircle(center_x - 10, center_y - 10, 4, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x - 10, center_y + 10, 4, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x + 10, center_y - 10, 4, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x + 10, center_y + 10, 4, TFT_GREEN);
        LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
        LCD_DISPLAY.setFreeFont(GLCD);
        LCD_DISPLAY.setTextDatum(MC_DATUM);
        LCD_DISPLAY.drawString("1", center_x + 28, center_y + 10);
        LCD_DISPLAY.drawString("2", center_x + 28, center_y - 10);
        LCD_DISPLAY.drawString("3", center_x - 27, center_y - 10);
        LCD_DISPLAY.drawString("4", center_x - 27, center_y + 10);
        break;

      case 5:
        // 5 pin gx16
        LCD_DISPLAY.fillCircle(center_x, center_y - 14, 3, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x - 12, center_y - 5, 3, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x + 12, center_y - 5, 3, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x - 10, center_y + 10, 3, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x + 10, center_y + 10, 3, TFT_GREEN);
        LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
        LCD_DISPLAY.setFreeFont(GLCD);
        LCD_DISPLAY.setTextDatum(MC_DATUM);
        LCD_DISPLAY.drawString("1", center_x + 28, center_y + 11);
        LCD_DISPLAY.drawString("2", center_x + 29, center_y - 6);
        LCD_DISPLAY.drawString("3", center_x + 1, center_y - 29);
        LCD_DISPLAY.drawString("4", center_x - 27, center_y - 6);
        LCD_DISPLAY.drawString("5", center_x - 27, center_y + 11);
        break;

      case 6:
        // 6 pin gx16
        LCD_DISPLAY.fillCircle(center_x, center_y - 14, 2, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x - 12, center_y - 5, 2, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x + 12, center_y - 5, 2, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x - 10, center_y + 10, 2, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x + 10, center_y + 10, 2, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x, center_y, 2, TFT_GREEN);
        LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
        LCD_DISPLAY.setFreeFont(GLCD);
        LCD_DISPLAY.setTextDatum(MC_DATUM);
        LCD_DISPLAY.drawString("1", center_x + 28, center_y + 11);
        LCD_DISPLAY.drawString("2", center_x + 29, center_y - 6);
        LCD_DISPLAY.drawString("3", center_x + 1, center_y - 29);
        LCD_DISPLAY.drawString("4", center_x - 27, center_y - 6);
        LCD_DISPLAY.drawString("5", center_x - 27, center_y + 11);
        LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLUE);
        LCD_DISPLAY.drawString("6", center_x + 1, center_y + 15);
        break;

      case 15:
        // 5 pin din
        LCD_DISPLAY.fillCircle(center_x, center_y - 14, 2, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x - 10, center_y - 9, 2, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x + 10, center_y - 9, 2, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x - 14, center_y, 2, TFT_GREEN);
        LCD_DISPLAY.fillCircle(center_x + 14, center_y, 2, TFT_GREEN);
        LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
        LCD_DISPLAY.setFreeFont(GLCD);
        LCD_DISPLAY.setTextDatum(MC_DATUM);
        LCD_DISPLAY.drawString("1", center_x + 28, center_y + 1);
        LCD_DISPLAY.drawString("3", center_x - 27, center_y + 1);
        LCD_DISPLAY.drawString("2", center_x + 1, center_y - 29);
        LCD_DISPLAY.drawString("4", center_x + 27, center_y - 13);
        LCD_DISPLAY.drawString("5", center_x - 25, center_y - 13);
        break;
    }

    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(&DIN_1451_Mittelschrift_Regular8pt7b);

    LCD_DISPLAY.setTextDatum(ML_DATUM);
    LCD_DISPLAY.drawString("1", 80, 10);
    LCD_DISPLAY.drawString("2", 80, 30);
    LCD_DISPLAY.drawString("3", 80, 50);
    LCD_DISPLAY.drawString("4", 80, 70);
    LCD_DISPLAY.drawString("5", 80, 90);
    LCD_DISPLAY.drawString("6", 80, 110);

    // pin functions
    if (updatemode == 0) {
      LCD_DISPLAY.drawString(TranslatePinFunction(cb_pin1function[cb_mike_model]), 95, 10);
      LCD_DISPLAY.drawString(TranslatePinFunction(cb_pin2function[cb_mike_model]), 95, 30);
      LCD_DISPLAY.drawString(TranslatePinFunction(cb_pin3function[cb_mike_model]), 95, 50);
      LCD_DISPLAY.drawString(TranslatePinFunction(cb_pin4function[cb_mike_model]), 95, 70);
      LCD_DISPLAY.drawString(TranslatePinFunction(cb_pin5function[cb_mike_model]), 95, 90);
      LCD_DISPLAY.drawString(TranslatePinFunction(cb_pin6function[cb_mike_model]), 95, 110);
    }

    LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(ML_DATUM);
    // c for custom, future feature
    if (cb_mike_model == 0) {
      LCD_DISPLAY.drawString("C", 0, center_y - 29);
    }
    else {
      LCD_DISPLAY.drawNumber((long) cb_mike_model, 0, center_y - 29);
    }

  }

  // ohm mode
  if (updatemode == 1) {
    draw20PixBitmap(30, 25, OmegaBitmap, 20, 20, TFT_WHITE);
  }

  // cb radio names
  if (updatemode == 0) {
    LCD_DISPLAY.setTextColor(TFT_ORANGE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TL_DATUM);

    switch (cb_mike_model) {
      case 1:
        LCD_DISPLAY.drawString("Midland(old)", 0, 0);
        LCD_DISPLAY.drawString("Cybernet", 0, 8);
        break;

      case 2:
        LCD_DISPLAY.drawString("Midland(new)", 0, 0);
        LCD_DISPLAY.drawString("Cobra", 0, 8);
        LCD_DISPLAY.drawString("President", 0, 16);
        LCD_DISPLAY.drawString("Uniden", 0, 24);
        break;

      case 3:
        LCD_DISPLAY.drawString("Robyn", 0, 0);
        LCD_DISPLAY.drawString("Royce", 0, 8);
        break;

      case 4:
        LCD_DISPLAY.drawString("Tokai", 0, 0);
        LCD_DISPLAY.drawString("Academy", 0, 8);
        LCD_DISPLAY.drawString("Elftone", 0, 16);
        LCD_DISPLAY.drawString("Fidelity", 0, 24);
        break;

      case 5:
        LCD_DISPLAY.drawString("Eurosonic", 0, 0);
        break;

      case 6:
        LCD_DISPLAY.drawString("Murphy", 0, 0);
        break;

      case 7:
        LCD_DISPLAY.drawString("Cobra", 0, 0);
        LCD_DISPLAY.drawString("President", 0, 8);
        LCD_DISPLAY.drawString("Uniden", 0, 16);
        break;

      case 8:
        LCD_DISPLAY.drawString("Cobra", 0, 0);
        LCD_DISPLAY.drawString("Uniden", 0, 8);
        LCD_DISPLAY.drawString("Midland", 0, 16);
        break;

      case 9:
        LCD_DISPLAY.drawString("Cobra 19+ O", 0, 0);
        LCD_DISPLAY.drawString("Cobra 18U", 0, 8);
        LCD_DISPLAY.drawString("Cobra 23+", 0, 16);
        LCD_DISPLAY.drawString("Cobra 41+", 0, 24);
        break;

      case 10:
        LCD_DISPLAY.drawString("Realistic", 0, 0);
        LCD_DISPLAY.drawString("Sears", 0, 8);
        LCD_DISPLAY.drawString("GE", 0, 16);
        break;

      case 11:
        LCD_DISPLAY.drawString("Cobra 19+ N", 0, 0);
        break;

      case 12:
        LCD_DISPLAY.drawString("Up/Down btns", 0, 0);
        LCD_DISPLAY.drawString("President", 0, 8);
        LCD_DISPLAY.drawString("Lincoln", 0, 16);
        LCD_DISPLAY.drawString("Emperor", 0, 24);
        break;

      case 13:
        LCD_DISPLAY.drawString("Shogun", 0, 0);
        break;

      case 14:
        LCD_DISPLAY.drawString("Gemini", 0, 0);
        LCD_DISPLAY.drawString("Telecoms", 0, 8);
        break;

      case 15:
        LCD_DISPLAY.drawString("DNT", 0, 0);
        LCD_DISPLAY.drawString("Radiotechnic", 0, 8);
        break;

      case 16:
        LCD_DISPLAY.drawString("LCL", 0, 0);
        break;

      case 17:
        LCD_DISPLAY.drawString("Bleubird", 0, 0);
        break;

      case 18:
        LCD_DISPLAY.drawString("Midland 1165", 0, 0);
        break;

      case 19:
        LCD_DISPLAY.drawString("Galaxy", 0, 0);
        break;

      case 20:
        LCD_DISPLAY.drawString("Sommerkamp", 0, 0);
        LCD_DISPLAY.drawString("RCI", 0, 8);
        break;

      case 21:
        LCD_DISPLAY.drawString("George", 0, 0);
        LCD_DISPLAY.drawString("Jackson", 0, 8);
        LCD_DISPLAY.drawString("DeltaForce", 0, 16);
        LCD_DISPLAY.drawString("Radio Shack", 0, 24);
        break;

      case 22:
        LCD_DISPLAY.drawString("Realistic", 0, 0);
        LCD_DISPLAY.drawString("Cybernet", 0, 8);
        break;

      case 23:
        LCD_DISPLAY.drawString("Cobra", 0, 0);
        LCD_DISPLAY.drawString("President", 0, 8);
        break;

    }
  }

  if (updatemode == 2) {
    // full screen scope
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.drawLine(0, 0, 0, 127, TFT_LIGHTGREY);
    LCD_DISPLAY.drawLine(0, 63, 159, 63, TFT_LIGHTGREY);
  }

  if (updatemode == 4) {
    // tx mode

    uint8_t center_x2 = 40;
    uint8_t center_y2 = 100;

    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.fillCircle(center_x2, center_y2, 24, TFT_BLUE);
    LCD_DISPLAY.fillCircle(center_x2, center_y2 + 24, 4, TFT_BLACK);

    // 5 pin din
    LCD_DISPLAY.fillCircle(center_x2, center_y2 - 14, 2, TFT_GREEN);
    LCD_DISPLAY.fillCircle(center_x2 - 10, center_y2 - 9, 2, TFT_GREEN);
    LCD_DISPLAY.fillCircle(center_x2 + 10, center_y2 - 9, 2, TFT_GREEN);
    LCD_DISPLAY.fillCircle(center_x2 - 14, center_y2, 2, TFT_GREEN);
    LCD_DISPLAY.fillCircle(center_x2 + 14, center_y2, 2, TFT_GREEN);
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(MC_DATUM);
    LCD_DISPLAY.drawString("3", center_x2 + 28, center_y2 + 1);
    LCD_DISPLAY.drawString("1", center_x2 - 27, center_y2 + 1);
    LCD_DISPLAY.drawString("2", center_x2 + 1, center_y2 - 29);
    LCD_DISPLAY.drawString("5", center_x2 + 27, center_y2 - 13);
    LCD_DISPLAY.drawString("4", center_x2 - 25, center_y2 - 13);

    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(&DIN_1451_Mittelschrift_Regular8pt7b);
    LCD_DISPLAY.setTextDatum(ML_DATUM);
    LCD_DISPLAY.drawString("1 C", 110, 10);
    LCD_DISPLAY.drawString("2 RX", 110, 30);
    LCD_DISPLAY.drawString("3 TX", 110, 50);
    LCD_DISPLAY.drawString("4 M", 110, 70);
    LCD_DISPLAY.drawString("5 S", 110, 90);

    Update_RX_TX(false);

    drawSineWave(0, 48, 17);

    Update_Modulation(cb_modulation);

  }



}

// rx or tx
void Update_RX_TX(boolean TXmode) {
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.setTextPadding(45);
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  if (TXmode) {
    LCD_DISPLAY.setTextColor(TFT_RED, TFT_BLACK);
    LCD_DISPLAY.drawString("TX", 5, 5);
  }
  else {
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.drawString("RX", 5, 5);
  }
  LCD_DISPLAY.setTextPadding(0);
}

// modulation frequency
void Update_Modulation(uint8_t modulation) {
  LCD_DISPLAY.setFreeFont(&DIN_1451_Mittelschrift_Regular8pt7b);
  LCD_DISPLAY.setTextPadding(58);
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  switch (modulation) {
    case 0:
      LCD_DISPLAY.drawString("None", 25, 41);
      break;

    case 1:
      LCD_DISPLAY.drawString("1 KHz", 25, 41);
      break;

    case 2:
      LCD_DISPLAY.drawString("500 Hz", 25, 41);
      break;

    case 3:
      LCD_DISPLAY.drawString("2.4 KHz", 25, 41);
      break;

    case 4:
      LCD_DISPLAY.drawString("2.5 KHz", 25, 41);
      break;

    case 5:
      LCD_DISPLAY.drawString("500+2.4", 25, 41);
      break;

  }
  LCD_DISPLAY.setTextPadding(0);
}


// return name for pin type
const char* TranslatePinFunction(uint8_t number) {
  switch (number) {
    case 0: return "-";
    case 1: return "S";
    case 2: return "C";
    case 3: return "SC";
    case 4: return "M";
    case 5: return "RX";
    case 6: return "TX";
    case 7: return "?";
    case 8: return "P";
    default: return "X";
  }
}

// splash screen
void DisplaySplash(void ) {
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.drawString("CB mike tester", 80, 0);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.drawString("Jef Collin 2024", 80, 25);
  LCD_DISPLAY.drawString("V 1.0", 80, 45);
  // depending on startup mode, sleep or not
  if (cb_normalstart) {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Turn:  model", 0, 71);
    LCD_DISPLAY.drawString("TX: Turn: modulation", 0, 81);
    LCD_DISPLAY.drawString("Press short: toggle display", 0, 91);
    LCD_DISPLAY.drawString("Press long: menu", 0, 101);

  }
  else {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Going to sleep", 80, 85, 2);
    LCD_DISPLAY.drawString("Press knob to wakeup", 80, 105, 2);
  }
}

// draw the menu
void DrawMenu(void) {
  if (cb_TXmodeactive) {
    menuItems[1] = "RX mode";
  }
  else {
    menuItems[1] = "TX mode";
  }

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
  // ground audio input
  CB_SelectAudioPin(0);
  // set all mcp pins to float
  mcp_release_pins();

  // tx relay off
  Transmit(false);

  // shutdown the backlight
  digitalWrite(Backlight_control, HIGH);
  // lcd chip to sleep
  LCD_DISPLAY.writecommand(ST7735_SLPIN);
  // Set the wakeup source to the button pin (GPIO 35)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0); // 0 means wake up when the pin is LOW (button pressed)
  // Enter deep sleep mode
  esp_deep_sleep_start();
}

// transmit relay control
void Transmit(boolean tx_on) {
  digitalWrite(cb_rx_tx_relay, tx_on);
}

// select pin to connect to audio input
void CB_SelectAudioPin(uint8_t pin) {
  switch (pin) {
    case 0:
      // ground X6 or X7
      mcp.digitalWrite(IO_mux_A, LOW);
      mcp.digitalWrite(IO_mux_B, HIGH);
      mcp.digitalWrite(IO_mux_C, HIGH);
      break;

    case 1:
      // X4
      mcp.digitalWrite(IO_mux_A, LOW);
      mcp.digitalWrite(IO_mux_B, LOW);
      mcp.digitalWrite(IO_mux_C, HIGH);
      break;

    case 2:
      // X2
      mcp.digitalWrite(IO_mux_A, LOW);
      mcp.digitalWrite(IO_mux_B, HIGH);
      mcp.digitalWrite(IO_mux_C, LOW);
      break;

    case 3:
      // X1
      mcp.digitalWrite(IO_mux_A, HIGH);
      mcp.digitalWrite(IO_mux_B, LOW);
      mcp.digitalWrite(IO_mux_C, LOW);
      break;

    case 4:
      // X0
      mcp.digitalWrite(IO_mux_A, LOW);
      mcp.digitalWrite(IO_mux_B, LOW);
      mcp.digitalWrite(IO_mux_C, LOW);
      break;

    case 5:
      // X3
      mcp.digitalWrite(IO_mux_A, HIGH);
      mcp.digitalWrite(IO_mux_B, HIGH);
      mcp.digitalWrite(IO_mux_C, LOW);
      break;

    case 6:
      // X5
      mcp.digitalWrite(IO_mux_A, HIGH);
      mcp.digitalWrite(IO_mux_B, LOW);
      mcp.digitalWrite(IO_mux_C, HIGH);
      break;

  }
}


// read selected pin
float CB_ReadVoltage(uint8_t pin) {
  float voltage = 0;
  float factor = 1;
  int16_t rawvoltage = 0;

  switch (pin) {
    case 1:
      factor = ADS2.toVoltage(1);  // voltage factor
      rawvoltage = ADS2.readADC(1);
      voltage = rawvoltage * factor * 1;
      break;

    case 2:
      factor = ADS2.toVoltage(1);  // voltage factor
      rawvoltage = ADS2.readADC(0);
      voltage = rawvoltage * factor * 1;
      break;

    case 3:
      factor = ADS1.toVoltage(1);  // voltage factor
      rawvoltage = ADS1.readADC(0);
      voltage = rawvoltage * factor * 1;
      break;

    case 4:
      factor = ADS1.toVoltage(1);  // voltage factor
      rawvoltage = ADS1.readADC(1);
      voltage = rawvoltage * factor * 1;
      break;

    case 5:
      factor = ADS1.toVoltage(1);  // voltage factor
      rawvoltage = ADS1.readADC(2);
      voltage = rawvoltage * factor * 1;
      break;

    case 6:
      factor = ADS1.toVoltage(1);  // voltage factor
      rawvoltage = ADS1.readADC(3);
      voltage = rawvoltage * factor * 1;
      break;

  }


  return voltage;
}

// configure all pins
void cb_SetAllPins(uint8_t model) {
  // mute audio first
  CB_SelectAudioPin(0);
  cb_SetPin(1, cb_pin1function[model]);
  cb_SetPin(2, cb_pin2function[model]);
  cb_SetPin(3, cb_pin3function[model]);
  cb_SetPin(4, cb_pin4function[model]);
  cb_SetPin(5, cb_pin5function[model]);
  cb_SetPin(6, cb_pin6function[model]);
}

// set one pin
void cb_SetPin(uint8_t pin, uint8_t function) {

  uint8_t pinmode = 0;
  // modes
  // 0 = floating
  // 1 = pull low no R
  // 2 = pull low via R
  // 3 = pull high via R

  // 0 or 4 = floating
  if (function == 1 or function == 2 or function == 3) {
    pinmode = 1;
  }
  else {
    if (function == 5 or function == 6 or function == 7 or function == 8) {
      pinmode = 3;
    }
  }

  // change rev 11
  // pull down if mike pin to avoid noise when in rx mode
  if (function == 4) {
    pinmode = 2;
  }

  // functions per pin
  // 0 = not used
  // 1 = shield (for mike)
  // 2 = common (for rx tx)
  // 3 = combined shield/common
  // 4 = mike
  // 5 = rx
  // 6 = tx
  // 7 = other functions like up down buttons
  // 8 = power for buttons etc

  switch (pin) {
    case 1:
      switch (pinmode) {
        case 0:
          mcp.pinMode(IO_pin1, INPUT);
          mcp.pinMode(IO_pin1_R, INPUT);
          break;

        case 1:
          mcp.pinMode(IO_pin1_R, INPUT);
          mcp.pinMode(IO_pin1, OUTPUT);
          mcp.digitalWrite(IO_pin1, LOW);
          break;

        case 2:
          mcp.pinMode(IO_pin1, INPUT);
          mcp.pinMode(IO_pin1_R, OUTPUT);
          mcp.digitalWrite(IO_pin1_R, LOW);
          break;

        case 3:
          mcp.pinMode(IO_pin1, INPUT);
          mcp.pinMode(IO_pin1_R, OUTPUT);
          mcp.digitalWrite(IO_pin1_R, HIGH);
          break;

      }
      break;

    case 2:
      switch (pinmode) {
        case 0:
          mcp.pinMode(IO_pin2, INPUT);
          mcp.pinMode(IO_pin2_R, INPUT);
          break;

        case 1:
          mcp.pinMode(IO_pin2_R, INPUT);
          mcp.pinMode(IO_pin2, OUTPUT);
          mcp.digitalWrite(IO_pin2, LOW);
          break;

        case 2:
          mcp.pinMode(IO_pin2, INPUT);
          mcp.pinMode(IO_pin2_R, OUTPUT);
          mcp.digitalWrite(IO_pin2_R, LOW);
          break;

        case 3:
          mcp.pinMode(IO_pin2, INPUT);
          mcp.pinMode(IO_pin2_R, OUTPUT);
          mcp.digitalWrite(IO_pin2_R, HIGH);
          break;

      }
      break;

    case 3:
      switch (pinmode) {
        case 0:
          mcp.pinMode(IO_pin3, INPUT);
          mcp.pinMode(IO_pin3_R, INPUT);
          break;

        case 1:
          mcp.pinMode(IO_pin3_R, INPUT);
          mcp.pinMode(IO_pin3, OUTPUT);
          mcp.digitalWrite(IO_pin3, LOW);
          break;

        case 2:
          mcp.pinMode(IO_pin3, INPUT);
          mcp.pinMode(IO_pin3_R, OUTPUT);
          mcp.digitalWrite(IO_pin3_R, LOW);
          break;

        case 3:
          mcp.pinMode(IO_pin3, INPUT);
          mcp.pinMode(IO_pin3_R, OUTPUT);
          mcp.digitalWrite(IO_pin3_R, HIGH);
          break;

      }
      break;

    case 4:
      switch (pinmode) {
        case 0:
          mcp.pinMode(IO_pin4, INPUT);
          mcp.pinMode(IO_pin4_R, INPUT);
          break;

        case 1:
          mcp.pinMode(IO_pin4_R, INPUT);
          mcp.pinMode(IO_pin4, OUTPUT);
          mcp.digitalWrite(IO_pin4, LOW);
          break;

        case 2:
          mcp.pinMode(IO_pin4, INPUT);
          mcp.pinMode(IO_pin4_R, OUTPUT);
          mcp.digitalWrite(IO_pin4_R, LOW);
          break;

        case 3:
          mcp.pinMode(IO_pin4, INPUT);
          mcp.pinMode(IO_pin4_R, OUTPUT);
          mcp.digitalWrite(IO_pin4_R, HIGH);
          break;

      }
      break;

    case 5:
      switch (pinmode) {
        case 0:
          mcp.pinMode(IO_pin5, INPUT);
          mcp.pinMode(IO_pin5_R, INPUT);
          break;

        case 1:
          mcp.pinMode(IO_pin5_R, INPUT);
          mcp.pinMode(IO_pin5, OUTPUT);
          mcp.digitalWrite(IO_pin5, LOW);
          break;

        case 2:
          mcp.pinMode(IO_pin5, INPUT);
          mcp.pinMode(IO_pin5_R, OUTPUT);
          mcp.digitalWrite(IO_pin5_R, LOW);
          break;

        case 3:
          mcp.pinMode(IO_pin5, INPUT);
          mcp.pinMode(IO_pin5_R, OUTPUT);
          mcp.digitalWrite(IO_pin5_R, HIGH);
          break;

      }
      break;

    case 6:
      switch (pinmode) {
        case 0:
          mcp.pinMode(IO_pin6, INPUT);
          mcp.pinMode(IO_pin6_R, INPUT);
          break;

        case 1:
          mcp.pinMode(IO_pin6_R, INPUT);
          mcp.pinMode(IO_pin6, OUTPUT);
          mcp.digitalWrite(IO_pin6, LOW);
          break;

        case 2:
          mcp.pinMode(IO_pin6, INPUT);
          mcp.pinMode(IO_pin6_R, OUTPUT);
          mcp.digitalWrite(IO_pin6_R, LOW);
          break;

        case 3:
          mcp.pinMode(IO_pin6, INPUT);
          mcp.pinMode(IO_pin6_R, OUTPUT);
          mcp.digitalWrite(IO_pin6_R, HIGH);
          break;

      }
      break;
  }

  // connect audio
  if (function == 4) {
    CB_SelectAudioPin(pin);
  }

}


// measure resistance
uint32_t CB_ReadResistance(uint8_t pin) {

  float groundreferencevoltage = 0;

  // locate common or combined pin for ground reference and measure offset
  if (cb_pin1function[cb_mike_model] == 2 or cb_pin1function[cb_mike_model] == 3) {
    groundreferencevoltage = CB_ReadVoltage(1);
  }
  else {
    if (cb_pin2function[cb_mike_model] == 2 or cb_pin2function[cb_mike_model] == 3) {
      groundreferencevoltage = CB_ReadVoltage(2);
    }
    else {
      if (cb_pin3function[cb_mike_model] == 2 or cb_pin3function[cb_mike_model] == 3) {
        groundreferencevoltage = CB_ReadVoltage(3);
      }
      else {
        if (cb_pin4function[cb_mike_model] == 2 or cb_pin4function[cb_mike_model] == 3) {
          groundreferencevoltage = CB_ReadVoltage(4);
        }
        else {
          if (cb_pin5function[cb_mike_model] == 2 or cb_pin5function[cb_mike_model] == 3) {
            groundreferencevoltage = CB_ReadVoltage(5);
          }
          else {

            if (cb_pin6function[cb_mike_model] == 2 or cb_pin6function[cb_mike_model] == 3) {
              groundreferencevoltage = CB_ReadVoltage(6);
            }
          }
        }
      }
    }
  }

  // measure pin voltage
  float voltage = CB_ReadVoltage(pin);

  // read 5v reference voltage on adc2 pin A2
  float factor = ADS2.toVoltage(1);  // voltage factor
  int16_t rawvoltage = ADS2.readADC(2);
  float refvoltage = rawvoltage * factor * 1;

  // default for open circuit
  float resistance = 100000;

  // check in case due to variation measured voltage is higher than ref voltage, this can happen in open circuit condition
  if (voltage < refvoltage) {
    resistance = ((voltage - groundreferencevoltage) * (681 + cb_internal_resistance)) / (refvoltage - voltage);
  }

  if (resistance > 100000) {
    resistance = 100000;
  }
  if (resistance < 0) {
    resistance = 0;
  }

  uint32_t finalresistance = round(resistance);

  return finalresistance;
}


void CB_ShowPinActivity(boolean ohm_mode) {

  // keep current status and resistance
  // 0 = blanc
  // 1 = green <20 ohm
  // 2 = orange >20 and < 1000
  // 3 = red > 1000 and < 20000
  uint8_t pinstatus1 = cb_pinstatus1;
  uint8_t pinstatus2 = cb_pinstatus2;
  uint8_t pinstatus3 = cb_pinstatus3;
  uint8_t pinstatus4 = cb_pinstatus4;
  uint8_t pinstatus5 = cb_pinstatus5;
  uint8_t pinstatus6 = cb_pinstatus6;

  uint32_t resistancepin1 = cb_pinresistance1;
  uint32_t resistancepin2 = cb_pinresistance2;
  uint32_t resistancepin3 = cb_pinresistance3;
  uint32_t resistancepin4 = cb_pinresistance4;
  uint32_t resistancepin5 = cb_pinresistance5;
  uint32_t resistancepin6 = cb_pinresistance6;

  Status_Clear();

  uint32_t colors[] = {TFT_BLACK, TFT_GREEN, TFT_ORANGE, TFT_RED};

  if (cb_pin1function[cb_mike_model] == 5 or cb_pin1function[cb_mike_model] == 6 or cb_pin1function[cb_mike_model] == 7) {
    cb_pinresistance1 = CB_ReadResistance(1);
    if (!ohm_mode) {
      if (cb_pinresistance1 <= 20) {
        cb_pinstatus1 = 1;
      }
      else if (cb_pinresistance1 > 20 and cb_pinresistance1 <= 200) {
        cb_pinstatus1 = 2;
      }
      else if (cb_pinresistance1 > 200 and cb_pinresistance1 <= 20000) {
        cb_pinstatus1 = 3;
      }
    }
  }

  if (cb_pin2function[cb_mike_model] == 5 or cb_pin2function[cb_mike_model] == 6 or cb_pin2function[cb_mike_model] == 7) {
    cb_pinresistance2 = CB_ReadResistance(2);
    if (!ohm_mode) {
      if (cb_pinresistance2 <= 20) {
        cb_pinstatus2 = 1;
      }
      else if (cb_pinresistance2 > 20 and cb_pinresistance2 <= 200) {
        cb_pinstatus2 = 2;
      }
      else if (cb_pinresistance2 > 200 and cb_pinresistance2 <= 20000) {
        cb_pinstatus2 = 3;
      }
    }
  }

  if (cb_pin3function[cb_mike_model] == 5 or cb_pin3function[cb_mike_model] == 6 or cb_pin3function[cb_mike_model] == 7) {
    cb_pinresistance3 = CB_ReadResistance(3);
    if (!ohm_mode) {
      if (cb_pinresistance3 <= 20) {
        cb_pinstatus3 = 1;
      }
      else if (cb_pinresistance3 > 20 and cb_pinresistance3 <= 200) {
        cb_pinstatus3 = 2;
      }
      else if (cb_pinresistance3 > 200 and cb_pinresistance3 <= 20000) {
        cb_pinstatus3 = 3;
      }
    }
  }

  if (cb_pin4function[cb_mike_model] == 5 or cb_pin4function[cb_mike_model] == 6 or cb_pin4function[cb_mike_model] == 7) {
    cb_pinresistance4 = CB_ReadResistance(4);
    if (!ohm_mode) {
      if (cb_pinresistance4 <= 20) {
        cb_pinstatus4 = 1;
      }
      else if (cb_pinresistance4 > 20 and cb_pinresistance4 <= 200) {
        cb_pinstatus4 = 2;
      }
      else if (cb_pinresistance4 > 200 and cb_pinresistance4 <= 20000) {
        cb_pinstatus4 = 3;
      }
    }
  }

  if (cb_pin5function[cb_mike_model] == 5 or cb_pin5function[cb_mike_model] == 6 or cb_pin5function[cb_mike_model] == 7) {
    cb_pinresistance5 = CB_ReadResistance(5);
    if (!ohm_mode) {
      if (cb_pinresistance5 <= 20) {
        cb_pinstatus5 = 1;
      }
      else if (cb_pinresistance5 > 20 and cb_pinresistance5 <= 200) {
        cb_pinstatus5 = 2;
      }
      else if (cb_pinresistance5 > 200 and cb_pinresistance5 <= 20000) {
        cb_pinstatus5 = 3;
      }
    }
  }

  if (cb_pin6function[cb_mike_model] == 5 or cb_pin6function[cb_mike_model] == 6 or cb_pin6function[cb_mike_model] == 7) {
    cb_pinresistance6 = CB_ReadResistance(6);
    if (!ohm_mode) {
      if (cb_pinresistance6 <= 20) {
        cb_pinstatus6 = 1;
      }
      else if (cb_pinresistance6 > 20 and cb_pinresistance6 <= 200) {
        cb_pinstatus6 = 2;
      }
      else if (cb_pinresistance6 > 200 and cb_pinresistance6 <= 20000) {
        cb_pinstatus6 = 3;
      }
    }
  }

  // check if any pins changed
  if (!ohm_mode and (cb_pinstatus1 != pinstatus1 or cb_pinstatus2 != pinstatus2 or cb_pinstatus3 != pinstatus3 or cb_pinstatus4 != pinstatus4 or cb_pinstatus5 != pinstatus5 or cb_pinstatus6 != pinstatus6)) {
    LCD_DISPLAY.fillCircle(140, 10, 7, colors[cb_pinstatus1]);
    LCD_DISPLAY.fillCircle(140, 30, 7, colors[cb_pinstatus2]);
    LCD_DISPLAY.fillCircle(140, 50, 7, colors[cb_pinstatus3]);
    LCD_DISPLAY.fillCircle(140, 70, 7, colors[cb_pinstatus4]);
    LCD_DISPLAY.fillCircle(140, 90, 7, colors[cb_pinstatus5]);
    LCD_DISPLAY.fillCircle(140, 110, 7, colors[cb_pinstatus6]);
  }

  if (ohm_mode and (cb_pinresistance1 != resistancepin1 or cb_pinresistance2 != resistancepin2 or cb_pinresistance3 != resistancepin3 or cb_pinresistance4 != resistancepin4 or cb_pinresistance5 != resistancepin5 or cb_pinresistance6 != resistancepin6 )) {
    LCD_DISPLAY.fillRect(95, 0, 159, 127, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(&DIN_1451_Mittelschrift_Regular8pt7b);
    LCD_DISPLAY.setTextDatum(ML_DATUM);
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);

    if (cb_pinresistance1 != 999999) {
      if (cb_pinresistance1 == 100000) {
        Draw_Infinity(95, 10);
      }
      else {
        LCD_DISPLAY.drawNumber(cb_pinresistance1, 95, 10);
      }
    }
    if (cb_pinresistance2 != 999999) {
      if (cb_pinresistance2 == 100000) {
        Draw_Infinity(95, 30);
      }
      else {
        LCD_DISPLAY.drawNumber(cb_pinresistance2, 95, 30);
      }
    }
    if (cb_pinresistance3 != 999999) {
      if (cb_pinresistance3 == 100000) {
        Draw_Infinity(95, 50);
      }
      else {
        LCD_DISPLAY.drawNumber(cb_pinresistance3, 95, 50);
      }
    }
    if (cb_pinresistance4 != 999999) {
      if (cb_pinresistance4 == 100000) {
        Draw_Infinity(95, 70);
      }
      else {
        LCD_DISPLAY.drawNumber(cb_pinresistance4, 95, 70);
      }
    }
    if (cb_pinresistance5 != 999999) {
      if (cb_pinresistance5 == 100000) {
        Draw_Infinity(95, 90);
      }
      else {
        LCD_DISPLAY.drawNumber(cb_pinresistance5, 95, 90);
      }
    }
    if (cb_pinresistance6 != 999999) {
      if (cb_pinresistance6 == 100000) {
        Draw_Infinity(95, 110);
      }
      else {
        LCD_DISPLAY.drawNumber(cb_pinresistance6, 95, 110);
      }
    }
  }
}

// audio sampling dma over i2s
void Scope_Sample(void) {
  int Scope_Scale_Factor = 30;
  uint8_t Scope_Max_Height = 126;

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

  float scalingFactor = (float)(Scope_Max_Height / 2) / 434.0;

  // process the samples
  for (int i = 0; i < 512; i++) {

    // remove the offset rcale to fit the display height
    int adjustedSample = (Scope_Max_Height / 2) + (int)((Scope_Samples[i] - Scope_Offset) * scalingFactor);

    if (adjustedSample > Scope_Max_Height) {
      adjustedSample = Scope_Max_Height;
    }

    if (adjustedSample < 0) {
      adjustedSample = 0;
    }
    Scope_Samples[i] = (int)adjustedSample;

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
// 127 pix by 158 large
void Scope_Show(void) {

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

  uint16_t Position_X = 2;

  uint8_t Scope_Max_Width = 157;
  uint8_t Scope_End_Y = 126;

  // blank out previous plot with background color unless its the same
  for (uint16_t Scope_Buffer_Pointer = 0; Scope_Buffer_Pointer < Scope_Max_Width ; Scope_Buffer_Pointer++) {
    if (Scope_Previous_Samples[Scope_Previous_Index + Scope_Buffer_Pointer] != Scope_Samples[Scope_Index + Scope_Buffer_Pointer] or Scope_Previous_Samples[Scope_Previous_Index + Scope_Buffer_Pointer + 1] != Scope_Samples[Scope_Index + Scope_Buffer_Pointer + 1]) {
      LCD_DISPLAY.drawLine(Position_X, Scope_End_Y - Scope_Previous_Samples[Scope_Previous_Index + Scope_Buffer_Pointer], Position_X + 1, Scope_End_Y - Scope_Previous_Samples[Scope_Previous_Index + Scope_Buffer_Pointer + 1], TFT_BLACK);
    }
    Position_X++;
  }

  LCD_DISPLAY.drawLine(0, 63, 159, 63, TFT_LIGHTGREY);

  // plot
  Position_X = 2;
  for (uint16_t Scope_Buffer_Pointer = 0; Scope_Buffer_Pointer < Scope_Max_Width ; Scope_Buffer_Pointer++) {
    LCD_DISPLAY.drawLine(Position_X, Scope_End_Y - Scope_Samples[Scope_Index + Scope_Buffer_Pointer], Position_X + 1, Scope_End_Y - Scope_Samples[Scope_Index + Scope_Buffer_Pointer + 1], TFT_WHITE);
    Position_X++;
  }

  // keep all 512 measurements
  for (uint16_t Scope_Buffer_Pointer = 0; Scope_Buffer_Pointer < 512 ; Scope_Buffer_Pointer++) {
    Scope_Previous_Samples[Scope_Buffer_Pointer] = Scope_Samples[Scope_Buffer_Pointer];
  }
  // keep index
  Scope_Previous_Index = Scope_Index;

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
  adc1_config_channel_atten(cb_mic_input, ADC_ATTEN_DB_11);
  adc1_config_width(ADC_WIDTH_BIT_12);
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, cb_mic_input);
  i2s_adc_enable(I2S_NUM_0);

  // The raw ADC data is written to DMA in inverted form. Invert back.
  SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);

}

// clear previous pin status and pin resistance
void Status_Clear(void) {
  cb_pinstatus1 = 0;
  cb_pinstatus2 = 0;
  cb_pinstatus3 = 0;
  cb_pinstatus4 = 0;
  cb_pinstatus5 = 0;
  cb_pinstatus6 = 0;
  cb_pinresistance1 = 999999;
  cb_pinresistance2 = 999999;
  cb_pinresistance3 = 999999;
  cb_pinresistance4 = 999999;
  cb_pinresistance5 = 999999;
  cb_pinresistance6 = 999999;
}

// infinity symbol
void Draw_Infinity(uint8_t pos_X, uint8_t pos_Y) {
  for (int t = 0; t <= 1000; t++) {
    float scale = 2 / (3 - cos(2 * t));
    int x = 12 * scale * cos(t);
    int y = 9 * scale * sin(2 * t) / 2;
    LCD_DISPLAY.drawPixel(pos_X + x + 12, pos_Y + y , TFT_WHITE);
    LCD_DISPLAY.drawPixel(pos_X + x + 13, pos_Y + y + 1, TFT_WHITE);
  }
}

// float all pins
void mcp_release_pins(void) {
  mcp.pinMode(IO_pin1, INPUT);
  mcp.pinMode(IO_pin1_R, INPUT);
  mcp.pinMode(IO_pin2, INPUT);
  mcp.pinMode(IO_pin2_R, INPUT);
  mcp.pinMode(IO_pin3, INPUT);
  mcp.pinMode(IO_pin3_R, INPUT);
  mcp.pinMode(IO_pin4, INPUT);
  mcp.pinMode(IO_pin4_R, INPUT);
  mcp.pinMode(IO_pin5, INPUT);
  mcp.pinMode(IO_pin5_R, INPUT);
  mcp.pinMode(IO_pin6, INPUT);
  mcp.pinMode(IO_pin6_R, INPUT);
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

// draw a sine wave
void drawSineWave(int startX, int baselineY, int height) {
  // Determine wave parameters
  float amplitude = height / 2.0; // Amplitude of the sine wave
  int waveWidth = 2 * amplitude; // Width of one cycle (one full sine wave cycle)

  int endX = startX + waveWidth; // End position for one full cycle

  int prevX = startX;
  int prevY = baselineY; // Initialize previous y to baseline (not used but needed for initial condition)

  for (int x = startX; x <= endX; x++) {
    // Calculate the sine value for the current x position
    float sineValue = sin((x - startX) * (2 * PI) / waveWidth);
    int y = baselineY - int(amplitude * sineValue); // Invert y to match the display coordinate system

    // Draw a line from the previous point to the current point
    if (x > startX) {
      LCD_DISPLAY.drawLine(prevX, prevY, x, y, TFT_WHITE);
    }

    // Update previous point coordinates
    prevX = x;
    prevY = y;
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

    .atten = DAC_COSINE_ATTEN_DEFAULT, // normal amplitude
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
