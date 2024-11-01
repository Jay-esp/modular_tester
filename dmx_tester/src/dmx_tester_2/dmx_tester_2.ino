// Modular tester
// DMX tester module
// Jef Collin 2024


// revisions
// 1.0 first release




// todo



#include <TFT_eSPI.h>
#include <SPI.h>
#include "Free_Fonts.h"
#include "esp_sleep.h"
#include <esp_dmx.h>


// use alps or other decoder
#define Use_Alps_Encoder false

TFT_eSPI LCD_DISPLAY = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

TFT_eSprite LCD_SPRITE2 = TFT_eSprite(&LCD_DISPLAY);

// DMX interface pins
#define DMX_enable 25
#define DMX_transmit 26
#define DMX_receive 27
#define DMX_power 16

// LCD
#define Backlight_control 13

// enable power
boolean dmx_enable = false;
// mode for controls, rx or tx
boolean dmx_txmode = true;
// channel number
uint16_t dmx_txchannel = 1;
// channel setting
uint8_t dmx_txsetting = 0;
// tx buffer
// 513 places in this buffer, byte at position 0 is the start code, which is always 0
uint8_t dmx_txbuffer[DMX_PACKET_SIZE];
// rx buffer
uint8_t dmx_rxbuffer[DMX_PACKET_SIZE];
// pointer in rx buffer
uint16_t dmx_rxchannel = 1;

// sleep or startup
boolean dmx_normalstart = false;

dmx_port_t dmx_port = 1;

dmx_packet_t packet;



// Menu variables
const char* menuItems[] = {"Return", "Enable DMX", "RX mode", "Go to sleep"};
int selectedMenu = 0;
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
char Encoder_1_Pin2 = 39;
char Encoder_1_Key = 36;

char Encoder_2_Pin1 = 33;
char Encoder_2_Pin2 = 32;
char Encoder_2_Key = 35;

// track rotary encoder changes
int EncoderCounter1 = 0;
int EncoderCounter2 = 0;

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

  // setup interface
  pinMode(DMX_enable, OUTPUT);
  pinMode(DMX_transmit, OUTPUT);
  pinMode(DMX_receive, INPUT);
  pinMode(DMX_power, OUTPUT);

  pinMode(Backlight_control, OUTPUT);

  // turn off dc/dc converter
  digitalWrite(DMX_power, LOW);

  // disable driver
  digitalWrite(DMX_enable, HIGH);

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
    dmx_normalstart = true;
  }

  DisplaySplash();

  if (dmx_normalstart) {
    // wait until key is no longer pressed
    while (digitalRead(Encoder_1_Key) == 0) {}
  }

  if (dmx_normalstart) {
    delay(4000);
  }
  else {
    delay(2500);
  }

  LCD_DISPLAY.fillScreen(TFT_BLACK);

  // goto sleep
  if (!dmx_normalstart) {
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

  // setup dmx library
  dmx_config_t dmx_config = DMX_CONFIG_DEFAULT;
  int dmx_personality_count = 1;
  dmx_personality_t dmx_personalities[] = {{1, "Default Personality"}};
  dmx_driver_install(dmx_port, &dmx_config, dmx_personalities, dmx_personality_count);
  dmx_set_pin(dmx_port, DMX_transmit, DMX_receive, DMX_enable);

  dmx_clear_tx_buffer();
  dmx_clear_rx_buffer();

  LCD_SPRITE2.createSprite(160, 128);

  BuildScreen(0);

}

void loop() {

  switch (ScreenMode) {
    case 0:
      // running mode
      // key 1 for menu
      if (digitalRead(Encoder_1_Key) == 0) {
        // switch to menu
        ScreenMode = 1;
        selectedMenu = 0;
        EncoderCounter1 = 0;
        drawMenu();
        // wait until key is no longer pressed
        while (digitalRead(Encoder_1_Key) == 0) {}
        // debounce
        delay(200);
      }


      // key 2 reset setting or list
      if (digitalRead(Encoder_2_Key) == 0) {
        // wait until key is no longer pressed
        while (digitalRead(Encoder_2_Key) == 0) {}
        if (dmx_txmode) {
          // reset setting
          dmx_txsetting = 0;
          dmx_txbuffer[dmx_txchannel] = dmx_txsetting;
          if (dmx_enable) {
            dmx_tx();
          }
          UpdateChannelSetting();
        }
        else {
          // reset list
          dmx_clear_rx_buffer();
          // update screen
          UpdateRX();
        }
      }

      // check rotary encoder 1
      if (EncoderCounter2 != 0) {
        if (dmx_txmode) {
          // tx, set channel
          if (EncoderCounter2 > 0) {
            if (dmx_txchannel < DMX_PACKET_SIZE - 1) {
              dmx_txchannel++;
              dmx_txsetting = dmx_txbuffer[dmx_txchannel];
              UpdateChannelSetting();
            }
            EncoderCounter2--;
          }
          else {
            if (dmx_txchannel > 1) {
              dmx_txchannel--;
              dmx_txsetting = dmx_txbuffer[dmx_txchannel];
              UpdateChannelSetting();
            }
            EncoderCounter2++;
          }
        }
        else {
          // rx, scroll, circular buffer
          if (EncoderCounter2 > 0) {
            if (dmx_rxchannel < DMX_PACKET_SIZE - 1) {
              dmx_rxchannel++;
              UpdateRX();
            }
            else {
              if (dmx_rxchannel == DMX_PACKET_SIZE - 1) {
                dmx_rxchannel = 1;
                UpdateRX();
              }
            }
            EncoderCounter2--;
          }
          else {
            if (dmx_rxchannel > 1) {
              dmx_rxchannel--;
              UpdateRX();
            }
            else {
              if (dmx_rxchannel == 1) {
                dmx_rxchannel = DMX_PACKET_SIZE - 1;
                UpdateRX();
              }
            }
            EncoderCounter2++;
          }
        }
      }

      // check rotary encoder 1
      if (EncoderCounter1 != 0) {
        if (dmx_txmode) {
          // tx, set setting
          if (EncoderCounter1 > 0) {
            if (dmx_txsetting < 255) {
              dmx_txsetting++;
              dmx_txbuffer[dmx_txchannel] = dmx_txsetting;
              UpdateChannelSetting();
              if (dmx_enable) {
                dmx_tx();
              }
            }
            EncoderCounter1--;
          }
          else {
            if (dmx_txsetting > 0) {
              dmx_txsetting--;
              dmx_txbuffer[dmx_txchannel] = dmx_txsetting;
              UpdateChannelSetting();
              if (dmx_enable) {
                dmx_tx();
              }
            }
            EncoderCounter1++;
          }
        }
        else {
          // rx no function

        }
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
            BuildScreen(0);
            break;

          case 1:
            // enable/disable dmx
            dmx_enable = !dmx_enable;
            if (dmx_enable) {
              // turn on dc/dc converter
              digitalWrite(DMX_power, HIGH);
              // allow to settle
              delay(200);
              dmx_tx();
              dmx_clear_rx_buffer();
            }
            else {
              // turn off dc/dc converter
              digitalWrite(DMX_power, LOW);
            }
            ForceNewMenu = true;
            break;

          case 2:
            // tx/rx mode
            dmx_txmode = !dmx_txmode;
            ForceNewMenu = true;
            break;

          case 3:
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
          if (selectedMenu < 3) {
            selectedMenu++;
            ForceNewMenu = true;
          }
          EncoderCounter1--;
        }
        if (EncoderCounter1 < 0) {
          if (selectedMenu > 0) {
            selectedMenu--;
            ForceNewMenu = true;
          }
          EncoderCounter1++;
        }
      }
      if (ForceNewMenu) {
        drawMenu();
      }
      break;
  }

  // receive if in rx mode and main screen
  if (!dmx_txmode and ScreenMode == 0) {
    // check for data but no timeout, non blocking
    if (dmx_receive(dmx_port, &packet, 0)) {
      // if we received something, get all data. packet will be 22ms
      if (dmx_receive(dmx_port, &packet, 200)) {
        if (!packet.err) {
          // read the data to the buffer
          dmx_read(dmx_port, dmx_rxbuffer, packet.size);
          // update screen
          UpdateRX();
        }
      }
    }
  }
}


// build screen
void BuildScreen(byte updatemode) {
  if (updatemode == 0) {
    LCD_DISPLAY.fillScreen(TFT_BLACK);
    LCD_DISPLAY.setFreeFont(GLCD);
    LCD_DISPLAY.setTextDatum(TL_DATUM);
    if (dmx_txmode) {
      LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
      LCD_DISPLAY.drawString("TX", 0, 0);
      LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
      LCD_DISPLAY.drawString("RX", 0, 53);
    }
    else {
      LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
      LCD_DISPLAY.drawString("TX", 0, 0);
      LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
      LCD_DISPLAY.drawString("RX", 0, 53);
    }
    LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
    LCD_DISPLAY.setTextDatum(TR_DATUM);
    if (dmx_enable) {
      LCD_DISPLAY.drawString("DMX on", 159, 0);
    }
    else {
      LCD_DISPLAY.drawString("DMX off", 159, 0);
    }
    LCD_DISPLAY.setFreeFont(FSS9);
    LCD_DISPLAY.setTextDatum(TC_DATUM);
    LCD_DISPLAY.drawString("Channel", 39, 12);
    LCD_DISPLAY.drawString("Setting", 119, 12);
    LCD_DISPLAY.drawLine(0, 48, 159, 48, TFT_YELLOW);
    UpdateChannelSetting();
    UpdateRX();
  }
}

void UpdateChannelSetting(void) {
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.setTextPadding(30);
  LCD_DISPLAY.drawNumber(dmx_txchannel, 39, 30);
  LCD_DISPLAY.drawNumber(dmx_txsetting, 119, 30);
  LCD_DISPLAY.setTextPadding(0);
}

void UpdateRX(void) {
  LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.setTextPadding(30);
  uint16_t rowpointer = dmx_rxchannel;
  for (byte row = 0; row < 5; row++) {
    LCD_DISPLAY.drawNumber(rowpointer, 39, (row * 15) + 53);
    LCD_DISPLAY.drawNumber(dmx_rxbuffer[rowpointer], 119, (row * 15) + 53);
    rowpointer++;
    if (rowpointer > DMX_PACKET_SIZE - 1) {
      rowpointer = 1;
    }
  }
  LCD_DISPLAY.setTextPadding(0);
}

// splash screen
void DisplaySplash(void ) {
  LCD_DISPLAY.setTextSize(1);
  LCD_DISPLAY.setFreeFont(FSS12);
  LCD_DISPLAY.fillScreen(TFT_BLACK);
  LCD_DISPLAY.setTextColor(TFT_GREEN, TFT_BLACK);
  LCD_DISPLAY.setTextDatum(TC_DATUM);
  LCD_DISPLAY.drawString("DMX tester", 80, 0);
  LCD_DISPLAY.setTextColor(TFT_BLUE, TFT_BLACK);
  LCD_DISPLAY.setFreeFont(FSS9);
  LCD_DISPLAY.drawString("Jef Collin 2024", 80, 25);
  LCD_DISPLAY.drawString("V 1.0", 80, 45);
  // depending on startup mode, sleep or not
  if (dmx_normalstart) {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Press knob R: menu", 80, 61, 2);
    LCD_DISPLAY.drawString("TX turn L: channel", 80, 78, 2);
    LCD_DISPLAY.drawString("TX turn R: setting", 80, 95, 2);
    LCD_DISPLAY.drawString("RX turn L: scroll", 80, 112, 2);
  }
  else {
    LCD_DISPLAY.setTextColor(TFT_WHITE, TFT_BLACK);
    LCD_DISPLAY.drawString("Going to sleep", 80, 85, 2);
    LCD_DISPLAY.drawString("Press knob R to wakeup", 80, 105, 2);
  }
}

// draw the menu
void drawMenu(void) {
  // adapt menu
  if (dmx_enable) {
    menuItems[1] = "Disable DMX";
  }
  else {
    menuItems[1] = "Enable DMX";
  }

  if (dmx_txmode) {
    menuItems[2] = "RX mode";
  }
  else {
    menuItems[2] = "TX mode";
  }
  LCD_SPRITE2.fillSprite(TFT_BLACK);
  LCD_SPRITE2.setTextDatum(TL_DATUM);
  LCD_SPRITE2.setFreeFont(FSS9);
  LCD_SPRITE2.setTextSize(1);
  LCD_SPRITE2.setTextColor(TFT_WHITE, TFT_BLACK);
  for (int i = 0; i < 4; i++) {
    if (i == selectedMenu) {
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
  // turn off dc/dc converter
  digitalWrite(DMX_power, LOW);
  // shutdown the backlight
  digitalWrite(Backlight_control, HIGH);
  // lcd chip to sleep
  LCD_DISPLAY.writecommand(ST7735_SLPIN);
  // Set the wakeup source to the button pin (GPIO 36)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_36, 0); // 0 means wake up when the pin is LOW (button pressed)
  // Enter deep sleep mode
  esp_deep_sleep_start();
}

void dmx_clear_tx_buffer(void) {
  // setup tx buffer, first byte is allways 0
  for (uint16_t i = 0; i < DMX_PACKET_SIZE; i++) {
    dmx_txbuffer[i] = 0;
  }
}

void dmx_clear_rx_buffer(void) {
  // setup rx buffer
  for (uint16_t i = 0; i < DMX_PACKET_SIZE; i++) {
    dmx_rxbuffer[i] = 0;
  }
  dmx_rxchannel = 1;
}

// send dmx block 22 ms
void dmx_tx(void) {
  // wait if transfers pending
  dmx_wait_sent(dmx_port, DMX_TIMEOUT_TICK);
  // write to the buffer
  dmx_write(dmx_port, dmx_txbuffer, DMX_PACKET_SIZE);
  // send the buffer
  dmx_send_num(dmx_port, DMX_PACKET_SIZE);
}
