// Modular tester
// Frequency Counter module
// Jef Collin 2024


// revisions
// 1.0 first release

// 4 10116p input module

// sensitivity at 10MHz 20mV
// range 5Hz (with a strong signal) to 57MHz



// todo
// add prescalers to menu



#include <TFT_eSPI.h>
#include <SPI.h>
#include "Free_Fonts.h"
#include "esp_sleep.h"
#include <Wire.h>

// use alps or other decoder
#define Use_Alps_Encoder false

TFT_eSPI LCD_DISPLAY = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

TFT_eSprite LCD_SPRITE2 = TFT_eSprite(&LCD_DISPLAY);

//  interface pins

#define fc_mosi 26
#define fc_cs 4
#define fc_miso 19
#define fc_sck 25
#define fc_mod_ctl 17


#define I2C_ADDRESS 0x09  // I2C address of the FPGA

// LCD
#define Backlight_control 13

// sleep or startup
boolean fc_normalstart = false;

// gate time
uint8_t fc_gate_time = 0;

// fpga setting parameter
uint8_t fc_fpga_parameter = 0;

// update timing interval
unsigned long fc_looptime = 1000;

// update timer
long unsigned fc_looptimer;

char fc_formatted_frequency[16]; // Buffer to store the formatted string
char fc_formatted_period[16];
char fc_formatted_unit[4];


uint32_t fc_frequency = 0;
uint32_t fc_previous_frequency = 999999999;

uint8_t fc_read_byte0;
uint8_t fc_read_byte1;
uint8_t fc_read_byte2;
uint8_t fc_read_byte3;

// Menu variables
const int numMenuItems = 2;

// Menu variables
const char* menuItems[] = {"Return", "Go to sleep"};
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

  //Serial.begin(115200);

  Wire.begin();

  // setup encoder pins
  pinMode(Encoder_1_Pin1, INPUT);
  pinMode(Encoder_1_Pin2, INPUT);
  pinMode(Encoder_1_Key, INPUT);

  pinMode(Encoder_2_Pin1, INPUT);
  pinMode(Encoder_2_Pin2, INPUT);
  pinMode(Encoder_2_Key, INPUT);


  // backup plan SPI interface
  // setup as input for now
  pinMode(fc_mosi, INPUT);
  pinMode(fc_cs, INPUT);
  pinMode(fc_miso, INPUT);
  pinMode(fc_sck, INPUT);

  pinMode(fc_mod_ctl, OUTPUT);

  digitalWrite(fc_mod_ctl, HIGH);

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
    fc_normalstart = true;
  }

  DisplaySplash();

  if (fc_normalstart) {
    // wait until key is no longer pressed
    while (digitalRead(Encoder_1_Key) == 0) {}
  }

  if (fc_normalstart) {
    delay(4000);
  }
  else {
    delay(2500);
  }

  LCD_DISPLAY.fillScreen(TFT_BLACK);

  // goto sleep
  if (!fc_normalstart) {
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


  //  byte error, address;
  //  int nDevices;
  //
  //  Serial.println("Scanning...");
  //
  //  nDevices = 0;
  //  for (address = 1; address < 127; address++ )
  //  {
  //    // The i2c_scanner uses the return value of
  //    // the Write.endTransmisstion to see if
  //    // a device did acknowledge to the address.
  //    Wire.beginTransmission(address);
  //    error = Wire.endTransmission();
  //
  //    if (error == 0)
  //    {
  //      Serial.print("I2C device found at address 0x");
  //      if (address < 16)
  //        Serial.print("0");
  //      Serial.print(address, HEX);
  //      Serial.println("  !");
  //
  //      nDevices++;
  //    }
  //    else if (error == 4)
  //    {
  //      Serial.print("Unknown error at address 0x");
  //      if (address < 16)
  //        Serial.print("0");
  //      Serial.println(address, HEX);
  //    }
  //  }
  //  Serial.println("Scanning end...");
  //
  //  delay(1000);


  set_looptime();
  update_gate_time();
  send_fpga_setting();
  update_frequency();
  update_freq_unit();
  fc_looptimer = millis() - 2000;

}

void loop() {

  if (ScreenMode == 0 or ScreenMode == 1) {

    if (EncoderCounter1 != 0) {
      if (EncoderCounter1 > 0) {
        if (ScreenMode < 1) {
          ScreenMode++;
          BuildScreen(ScreenMode);
          if (ScreenMode == 0) {
            update_frequency();
            update_freq_unit();
            update_gate_time();
          }
          else {
            update_period();
            update_gate_time();
          }
        }
        EncoderCounter1--;
      }
      else {
        if (ScreenMode > 0) {
          ScreenMode--;
          BuildScreen(ScreenMode);
          if (ScreenMode == 0) {
            update_frequency();
            update_freq_unit();
            update_gate_time();
          }
          else {
            update_period();
            update_gate_time();
          }
        }
        EncoderCounter1++;
      }
    }

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

      }
    }

    // gate time
    if (EncoderCounter2 != 0) {
      if (EncoderCounter2 > 0) {
        if (fc_gate_time < 3) {
          fc_gate_time++;
          fc_frequency = 0;
          fc_previous_frequency = 999999999;
          if (ScreenMode == 0) {
            update_frequency();
            update_freq_unit();
            update_gate_time();
          }
          else {
            update_period();
            update_gate_time();
          }
          send_fpga_setting();
          fc_looptimer = millis();
        }
        EncoderCounter2--;
      }
      else {
        if (fc_gate_time > 0) {
          fc_gate_time--;
          fc_frequency = 0;
          fc_previous_frequency = 999999999;
          if (ScreenMode == 0) {
            update_frequency();
            update_freq_unit();
            update_gate_time();
          }
          else {
            update_period();
            update_gate_time();
          }
          send_fpga_setting();
          fc_looptimer = millis();
        }
        EncoderCounter2++;
      }
    }
  }

  switch (ScreenMode) {
    case 0:
      // running mode

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
            set_looptime();
            update_gate_time();
            send_fpga_setting();
            update_frequency();
            update_freq_unit();
            fc_looptimer = millis() - 2000;
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

  // read frequency
  if ((ScreenMode == 0 or ScreenMode == 1) and (millis() - fc_looptimer) >= fc_looptime) {
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(0x05);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDRESS, 1);
    fc_read_byte0 = Wire.read();

    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(0x05);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDRESS, 1);
    fc_read_byte1 = Wire.read();

    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(0x05);
    Wire.write(0x02);
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDRESS, 1);
    fc_read_byte2 = Wire.read();

    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(0x05);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDRESS, 1);
    fc_read_byte3 = Wire.read();

    fc_frequency = (static_cast<uint32_t>(fc_read_byte3) << 24) |
                   (static_cast<uint32_t>(fc_read_byte2) << 16) |
                   (static_cast<uint32_t>(fc_read_byte1) << 8)  |
                   static_cast<uint32_t>(fc_read_byte0);

    if (fc_frequency > 199999999) {
      fc_frequency = 199999999;
    }

    // frequency
    if (fc_frequency != fc_previous_frequency) {
      if (ScreenMode == 0) {
        update_frequency();
      }
      else {
        // period
        update_period();
      }
      fc_previous_frequency = fc_frequency;
    }

    fc_looptimer = millis();
  }

}

// build screen
void BuildScreen(byte updatemode) {
  LCD_DISPLAY.fillScreen(TFT_BLACK);

}

void update_gate_time(void) {
  LCD_DISPLAY.setFreeFont(GLCD);
  LCD_DISPLAY.setTextDatum(TL_DATUM);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(79);
  switch (fc_gate_time) {
    case 0:
      LCD_DISPLAY.drawString("Gate 1S", 0, 0);
      if (ScreenMode == 0) {
        LCD_DISPLAY.drawString("Res 1Hz", 80, 0);
      }
      break;

    case 1:
      LCD_DISPLAY.drawString("Gate 0.1S", 0, 0);
      if (ScreenMode == 0) {
        LCD_DISPLAY.drawString("Res 10Hz", 80, 0);
      }
      break;

    case 2:
      LCD_DISPLAY.drawString("Gate 0.01S", 0, 0);
      if (ScreenMode == 0) {
        LCD_DISPLAY.drawString("Res 100Hz", 80, 0);
      }
      break;

    case 3:
      LCD_DISPLAY.drawString("Gate 0.001S", 0, 0);
      if (ScreenMode == 0) {
        LCD_DISPLAY.drawString("Res 1KHz", 80, 0);
      }
      break;

  }
  LCD_DISPLAY.setTextPadding(0);
}

void update_frequency(void) {
  formatFrequency(fc_frequency, fc_gate_time, fc_formatted_frequency);
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.setTextDatum(TR_DATUM);
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(160);
  LCD_DISPLAY.drawString(fc_formatted_frequency, 142, 50);

  // LCD_DISPLAY.drawString("888.888.888", 142, 30);

  LCD_DISPLAY.setTextPadding(0);
}

void update_freq_unit(void) {
  LCD_DISPLAY.setTextDatum(TR_DATUM);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(50);
  if (fc_gate_time == 0) {
    LCD_DISPLAY.drawString("Hz", 142, 80);
  }
  else {
    LCD_DISPLAY.drawString("KHz", 142, 80);
  }
  LCD_DISPLAY.setTextPadding(0);
}

void update_period(void) {
  formatPeriod(fc_frequency, fc_gate_time, fc_formatted_period, fc_formatted_unit);
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.setTextDatum(TR_DATUM);
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(160);
  LCD_DISPLAY.drawString(fc_formatted_period, 142, 50);

  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextPadding(50);
  LCD_DISPLAY.drawString(fc_formatted_unit, 142, 80);
  LCD_DISPLAY.setTextPadding(0);

}

// splash screen
void DisplaySplash(void ) {
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.drawString("Freq Counter", 80, 0);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.drawString("Jef Collin 2024", 80, 25);
  LCD_DISPLAY.drawString("V 1.0", 80, 45);
  // depending on startup mode, sleep or not
  if (fc_normalstart) {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    LCD_DISPLAY.drawString("Press R long: menu", 0, 71);
    LCD_DISPLAY.drawString("Turn L: gate time", 0, 91);
    LCD_DISPLAY.drawString("Turn R: Freq/Period", 0, 111);
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
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(0x01);
  Wire.write(0x00);
  // set stdby flag
  Wire.write(0x80);
  Wire.endTransmission();

  digitalWrite(fc_mod_ctl, LOW);

  // shutdown the backlight
  digitalWrite(Backlight_control, HIGH);
  // lcd chip to sleep
  LCD_DISPLAY.writecommand(ST7735_SLPIN);
  // Set the wakeup source to the button pin (GPIO 36)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_36, 0); // 0 means wake up when the pin is LOW (button pressed)
  // Enter deep sleep mode
  esp_deep_sleep_start();
}

void send_fpga_setting(void) {
  fc_fpga_parameter = (fc_fpga_parameter & 0xF0) | (fc_gate_time & 0x0F);
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(0x01);
  Wire.write(0x00);
  Wire.write(fc_fpga_parameter);
  Wire.endTransmission();
}

void set_looptime(void) {
  switch (fc_gate_time) {
    case 0:
      fc_looptime = 1200;
      break;

    case 1:
      fc_looptime = 120;
      break;

    case 2:
      fc_looptime = 12;
      break;

    case 3:
      fc_looptime = 12;
      break;
  }
}

// Function to format frequency into a European-style formatted string
void formatFrequency(uint32_t frequency, uint8_t gateTime, char* outputBuffer) {
  char tempBuffer[32] = "";  // Temporary buffer for frequency formatting
  char integerPart[32] = ""; // Buffer for integer portion
  char decimalPart[4] = "";  // Buffer for the decimal portion
  int length;                // Length of the formatted frequency
  int decimalDigits = 0;     // Number of decimal digits

  // Adjust frequency and determine decimal placement based on gate time
  switch (gateTime) {
    case 0: // 1 second gate time (Hertz)
      length = snprintf(tempBuffer, sizeof(tempBuffer), "%lu", frequency);
      decimalDigits = 0;
      break;
    case 1: // 0.1 second gate time (KHz, two decimals)
      length = snprintf(tempBuffer, sizeof(tempBuffer), "%lu", frequency / 100);
      snprintf(decimalPart, sizeof(decimalPart), "%02lu", frequency % 100);
      decimalDigits = 2;
      break;
    case 2: // 0.01 second gate time (KHz, one decimal)
      length = snprintf(tempBuffer, sizeof(tempBuffer), "%lu", frequency / 10);
      snprintf(decimalPart, sizeof(decimalPart), "%lu", frequency % 10);
      decimalDigits = 1;
      break;
    case 3: // 0.001 second gate time (KHz, no decimals)
      length = snprintf(tempBuffer, sizeof(tempBuffer), "%lu", frequency);
      decimalDigits = 0;
      break;
    default: // Invalid gate time
      strcpy(outputBuffer, "ERROR");
      return;
  }

  // Add thousands separators to the integer part
  int integerIndex = 0;
  int commaCount = 0;
  for (int i = length - 1; i >= 0; i--) {
    if (commaCount > 0 && commaCount % 3 == 0) {
      integerPart[integerIndex++] = '.';
    }
    integerPart[integerIndex++] = tempBuffer[i];
    commaCount++;
  }
  integerPart[integerIndex] = '\0';

  // Reverse the integer part to get the correct format
  for (int i = 0; i < integerIndex / 2; i++) {
    char temp = integerPart[i];
    integerPart[i] = integerPart[integerIndex - i - 1];
    integerPart[integerIndex - i - 1] = temp;
  }

  // Combine integer and decimal parts into the output buffer
  if (decimalDigits > 0) {
    snprintf(outputBuffer, 32, "%s,%s", integerPart, decimalPart);
  } else {
    snprintf(outputBuffer, 32, "%s", integerPart);
  }
}

// Format period into a European-style string with appropriate units
void formatPeriod(uint32_t frequency, uint8_t gateTime, char *output, char *unit) {
  uint32_t scaledFrequency;
  char temp[32];

  // Scale frequency based on gate time
  switch (gateTime) {
    case 0:  // 1s gate time
      scaledFrequency = frequency;
      break;
    case 1:  // 0.1s gate time
      scaledFrequency = frequency * 10;
      break;
    case 2:  // 0.01s gate time
      scaledFrequency = frequency * 100;
      break;
    case 3:  // 0.001s gate time
      scaledFrequency = frequency * 1000;
      break;
    default:
      scaledFrequency = 1; // Prevent divide-by-zero
      break;
  }

  float period = 0;

  if (scaledFrequency == 0) {

  }
  else {
    period = 1.0 / (float)scaledFrequency; // Period in seconds
  }

  // Determine unit and scale
  float displayValue;
  if (period < 1e-6) {  // nanoseconds
    displayValue = period * 1e9;
    strcpy(unit, "ns");
  } else if (period < 1e-3) {  // microseconds
    displayValue = period * 1e6;
    strcpy(unit, "us");
  } else if (period < 1.0) {  // milliseconds
    displayValue = period * 1e3;
    strcpy(unit, "ms");
  } else {  // seconds
    displayValue = period;
    strcpy(unit, "s");
  }

  // Format number with decimals
  char formattedValue[32];
  uint8_t decimals = (strcmp(unit, "ns") == 0 || strcmp(unit, "us") == 0) ? 1 : 3;
  dtostrf(displayValue, 0, decimals, formattedValue);

  // Replace '.' with ',' for decimal separator
  for (char *p = formattedValue; *p; p++) {
    if (*p == '.') *p = ',';
  }

  // Insert thousands separators
  int len = strlen(formattedValue);
  char formatted[32] = "";
  int insertPos = 0;
  bool pastDecimal = false;

  for (int i = len - 1, group = 0; i >= 0; i--) {
    if (formattedValue[i] == ',') pastDecimal = true;
    if (!pastDecimal && group > 0 && group % 3 == 0) {
      formatted[insertPos++] = '.';
      group = 0;
    }
    formatted[insertPos++] = formattedValue[i];
    group++;
  }
  formatted[insertPos] = '\0';

  // Reverse the formatted string
  int formattedLen = strlen(formatted);
  for (int i = 0; i < formattedLen / 2; i++) {
    char swap = formatted[i];
    formatted[i] = formatted[formattedLen - 1 - i];
    formatted[formattedLen - 1 - i] = swap;
  }

  // Copy the result to the output and append the unit
  sprintf(output, " % s", formatted);
}
