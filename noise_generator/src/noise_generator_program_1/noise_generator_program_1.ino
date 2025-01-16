// Modular tester
// noise generator module programmer
// Jef Collin 2024-2025


// revisions
// 1.0 first release


// note
// only used to program the renesas module
// use once and then load the module code
// watch serial monitor output

// generated NVM file in go configure
// converted in pycharm
// copy to this folder

#include "slg47004_config.h" // Include the generated header file

#include <TFT_eSPI.h>
#include <SPI.h>
#include "Free_Fonts.h"
#include "esp_sleep.h"
#include <Wire.h>

TFT_eSPI LCD_DISPLAY = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

//  interface pins

#define noise_pwr 17
#define noise_mux_a 32
#define noise_mux_b 33

// LCD
#define Backlight_control 13

// mode
// 0 = off
// 1 = white noise
// 2 = pink noise
// 3 = brown noise
uint8_t noise_mode = 0;

// keep registers for reostat calibration
uint8_t noise_0xe6 = 0;
uint8_t noise_0xe7 = 0;
uint8_t noise_0xe8 = 0;
uint8_t noise_0xe9 = 0;

// array 16 pages of 16 bytes
uint8_t noise_data_array[16][16] = {};

char Encoder_1_Key = 36;

long unsigned timer_encoderbutton1;

boolean Encoder_Key1_Long_Press = false;

void setup() {

  Serial.begin(115200);

  Wire.begin();

  // setup encoder pins
  pinMode(Encoder_1_Key, INPUT);

  // noise off
  pinMode(noise_mux_a, OUTPUT);
  pinMode(noise_mux_b, OUTPUT);

  set_mode(noise_mode);

  // power up noise module
  pinMode(noise_pwr, OUTPUT);
  digitalWrite(noise_pwr, HIGH);

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

  DisplaySplash();

  delay(3000);

  BuildScreen(0);

  report_i2c();

  BuildScreen(1);

}

void loop() {

  // key 1 long start
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
      BuildScreen(2);

      // check config file
      if (sizeof(slg47004_config) != 256 ) {
        Serial.println("Error in size of slg47004_config");
      }
      else {
        // read config data
        Serial.println("Config data from slg47004_config file");
        for (uint8_t noise_pageindex = 0; noise_pageindex < 16; noise_pageindex++) {
          for (uint8_t noise_byteindex = 0; noise_byteindex < 16; noise_byteindex++) {
            noise_data_array[noise_pageindex][noise_byteindex] = slg47004_config[((noise_pageindex * 16) + noise_byteindex)];
            PrintHex8(noise_data_array[noise_pageindex][noise_byteindex]);
          }
          Serial.println();
        }
        Serial.println();

        // read current NVM and separate reostat bytes
        NVM_read();

        // add reostat bytes in array
        noise_data_array[14][6] = noise_0xe6;
        noise_data_array[14][7] = noise_0xe7;
        noise_data_array[14][8] = noise_0xe8;
        noise_data_array[14][9] = noise_0xe9;

        // consolidated data and reostat
        Serial.println("Consolidated configuration");
        for (uint8_t noise_pageindex = 0; noise_pageindex < 16; noise_pageindex++) {
          for (uint8_t noise_byteindex = 0; noise_byteindex < 16; noise_byteindex++) {
            PrintHex8(noise_data_array[noise_pageindex][noise_byteindex]);
          }
          Serial.println();
        }
        Serial.println();

        delay(200);

        // erase
        NVM_erase();
        delay(200);

        // program
        NVM_write();
        delay(200);

        // read back current NVM
        Serial.println("Verification config data");
        NVM_read();

        BuildScreen(3);
        delay(5000);

        GoToSleep();
      }
    }
  }

}

// build screen
void BuildScreen(uint8_t screenmode) {
  switch (screenmode) {
    case 0:
      LCD_DISPLAY.fillScreen(TFT_BLACK);
      LCD_DISPLAY.setTextSize(1);
      LCD_DISPLAY.setFreeFont(FSS9);
      LCD_DISPLAY.fillScreen(TFT_BLACK);
      LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
      LCD_DISPLAY.setTextDatum(TL_DATUM);
      break;

    case 1:
      LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
      LCD_DISPLAY.setFreeFont(GLCD);
      LCD_DISPLAY.setTextDatum(TL_DATUM);
      LCD_DISPLAY.drawString("Press R long to start", 0, 100);
      break;

    case 2:
      LCD_DISPLAY.fillScreen(TFT_BLACK);
      LCD_DISPLAY.setFreeFont(FSS9);
      LCD_DISPLAY.fillScreen(TFT_BLACK);
      LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
      LCD_DISPLAY.setTextDatum(TL_DATUM);
      LCD_DISPLAY.drawString("Downloading...", 0, 50);
      break;

    case 3:
      LCD_DISPLAY.fillScreen(TFT_BLACK);
      LCD_DISPLAY.setFreeFont(FSS9);
      LCD_DISPLAY.fillScreen(TFT_BLACK);
      LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
      LCD_DISPLAY.setTextDatum(TL_DATUM);
      LCD_DISPLAY.drawString("Completed", 0, 50);
      LCD_DISPLAY.drawString("Going to sleep...", 0, 85);
      break;

  }
}


// splash screen
void DisplaySplash(void ) {
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.drawString("Noise Generator", 80, 0);
  LCD_DISPLAY.drawString("Downloader", 80, 20);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.drawString("Jef Collin 2025", 80, 40);
  LCD_DISPLAY.drawString("V 1.0", 80, 60);
}


void GoToSleep (void) {
  digitalWrite(noise_pwr, LOW);
  // select mode 0, mux all 1
  set_mode(0);
  // shutdown the backlight
  digitalWrite(Backlight_control, HIGH);
  // lcd chip to sleep
  LCD_DISPLAY.writecommand(ST7735_SLPIN);
  // Enter deep sleep mode
  esp_deep_sleep_start();
}

void set_mode(uint8_t mode_selector) {
  switch (mode_selector) {
    case 0:
      // no noise
      digitalWrite(noise_mux_a, HIGH);
      digitalWrite(noise_mux_b, HIGH);
      break;

    case 1:
      digitalWrite(noise_mux_a, LOW);
      digitalWrite(noise_mux_b, LOW);
      break;

    case 2:
      digitalWrite(noise_mux_a, HIGH);
      digitalWrite(noise_mux_b, LOW);
      break;

    case 3:
      digitalWrite(noise_mux_a, LOW);
      digitalWrite(noise_mux_b, HIGH);
      break;
  }
}

void report_i2c(void) {
  uint8_t ypos = 0;
  String hexAddress;
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      hexAddress = String(address, HEX);
      hexAddress.toUpperCase();
      // Display the message
      LCD_DISPLAY.drawString("Detected at 0x" + hexAddress, 0, ypos);
      ypos = ypos + 15;

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.println("Scanning end...");
}

void NVM_erase(void) {
  // erase the NVM 16 pages
  for (uint8_t i = 0; i < 16; i++) {
    if (i == 8 or i == 15 ) {
      // locked service page
      // skip
    }
    else {
      if (i == 14) {
        // reostat tolerance data
        // no action since we read these with NVM_read
      }
      // config address = base address
      Wire.beginTransmission(0x08);
      Wire.write(0xE3);
      // page address
      Wire.write(0xC0 | i);
      Wire.endTransmission();
      // since we don't use acknowledgement we use a delay as specified
      delay(20);
    }
  }
}

void NVM_read(void) {
  Serial.println("Current data in NVM");
  // keep track of byte location
  uint16_t bytecount = 0;
  uint8_t byteread = 0;
  for (int i = 0; i < 16; i++) {
    Wire.beginTransmission(0x0A);
    Wire.write(i << 4);
    Wire.endTransmission(false);
    delay(10);
    Wire.requestFrom(0x0A, 16);
    while (Wire.available()) {
      byteread = Wire.read();
      PrintHex8(byteread);
      // check for reostat bytes
      switch (bytecount) {
        case 0xe6:
          noise_0xe6 = byteread;
          break;

        case 0xe7:
          noise_0xe7 = byteread;
          break;

        case 0xe8:
          noise_0xe8 = byteread;
          break;

        case 0xe9:
          noise_0xe9 = byteread;
          break;
      }
      bytecount++;
    }
    Serial.println();
  }
  Serial.println("reostat bytes: ");
  Serial.println(noise_0xe6, HEX);
  Serial.println(noise_0xe7, HEX);
  Serial.println(noise_0xe8, HEX);
  Serial.println(noise_0xe9, HEX);
  Serial.println();

}

void NVM_write(void) {
  // Write each byte of data_array[][] array to the chip
  for (int i = 0; i < 16; i++) {
    if (i == 8 or i == 15 ) {
      // locked service page
      // skip
    }
    else {
      Wire.beginTransmission(0x0A);
      Wire.write(i << 4);
      for (int j = 0; j < 16; j++) {
        Wire.write(noise_data_array[i][j]);
      }
      Wire.endTransmission();
      // since we don't use acknowledgement we use a delay as specified
      delay(20);
    }
  }
}



void PrintHex8(uint8_t data) {
  if (data < 0x10) {
    Serial.print("0");
  }
  Serial.print(data, HEX);
  Serial.print(" ");
}
