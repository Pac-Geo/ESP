/*
  ==================================================================
  ESP32-S3 PHOTO FRAME
  POWER + AMG8833 + OLED + USB HOST TEST
  ==================================================================

  CURRENT WIRING
  ------------------------------------------------------------------

  POWER / CONTROL:

    GPIO15  <- AMG8833 INT
    GPIO16  -> Relay module IN
    GPIO21  <- Power button -> GND


  I2C BUS:

    GPIO17  -> SDA
               |-- AMG8833 SDA
               `-- OLED SDA

    GPIO18  -> SCL
               |-- AMG8833 SCL
               `-- OLED SCL


  USB HOST:

    GPIO19  -> USB-A D-
    GPIO20  -> USB-A D+

    5V      -> USB-A VBUS
    GND     -> USB-A GND


  ==================================================================
  POWER BUTTON
  ==================================================================

  Hold GPIO21 button for 3 seconds:

      -> OLED OFF
      -> Relay OFF
      -> System OFF
      -> ESP32 enters deep sleep
      -> AMG8833 CANNOT wake system

  Only a NEW power-button press turns the system back ON.


  ==================================================================
  SENSOR / DISPLAY SLEEP
  ==================================================================

  While system is ON:

      Person detected
          -> display stays awake
          -> inactivity timer resets

      No person for 30 seconds
          -> OLED OFF
          -> USB/Wi-Fi/VGA will eventually stop
          -> Relay STAYS ON
          -> Monitor keeps AC power
          -> ESP32 enters deep sleep

      AMG8833 INT
          -> ESP32 wakes
          -> OLED/display resumes
          -> Relay stayed ON the whole time


  ==================================================================
  USB HOST
  ==================================================================

  When system is ON:

      ESP32 starts native USB Host
          ->
      USB flash drive inserted
          ->
      ESP32 detects device
          ->
      VID/PID printed to Serial Monitor
          ->
      OLED shows "USB"

  THIS VERSION TESTS USB MASS STORAGE + FAT LFN + CHUNKED DRIVE SYNC.

  USB STORAGE:
      - mounts FAT32
      - uses FAT long filenames (LFN must be enabled in sdkconfig)
      - lists files
      - writes/reads a long-name test file

  DRIVE SYNC:
      - gets the Drive photo list from Apps Script
      - downloads only missing JPEG/PNG files
      - streams in 16 KiB chunks
      - supports files up to 100 MiB
      - writes to DOWNLOAD.TMP first
      - verifies byte count
      - renames to the final long filename only after success


  ==================================================================
  VALUES TO CHANGE LATER
  ==================================================================

  BUTTON_HOLD_TIME:

      Current:
          3000 = 3 seconds


  INACTIVITY_TIME:

      Current TEST:
          30000 = 30 seconds

      FINAL 30 MINUTES:

          30UL * 60UL * 1000UL


  PERSON_TEMP_THRESHOLD:

      Current:
          28.0 C

      Adjust based on the room.


  AMG_INTERRUPT_HIGH:

      Current:
          28.0 C

      AMG8833 hardware wake threshold.


  OLED_ADDRESS:

      Usually:
          0x3C

      Sometimes:
          0x3D


  ==================================================================
  FUTURE FUNCTIONS
  ==================================================================

  displayPhotos()

      Currently draws a fake photo on the OLED.

      Eventually:

          USB photo
              ->
          JPEG decode
              ->
          framebuffer
              ->
          VGA


  sleepDisplay()

      Currently turns OLED off.

      Eventually also:
          stop VGA
          stop slideshow
          stop Wi-Fi
          stop Drive downloads


  wakeDisplay()

      Currently turns OLED back on.

      Eventually restarts:
          VGA
          slideshow
          Wi-Fi as needed

  ==================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#include <Adafruit_AMG88xx.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "esp_sleep.h"
#include "driver/gpio.h"

#include "usb/usb_host.h"
#include "esp_err.h"

#include <dirent.h>
#include <sys/stat.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <new>

#include "usb/msc_host.h"
#include "usb/msc_host_vfs.h"
#include "esp_vfs_fat.h"
#include "mbedtls/base64.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "esp_jpeg_dec.h"
#include "esp_jpeg_common.h"
// ==================================================================
// PIN ASSIGNMENTS
// ==================================================================

const gpio_num_t AMG_INT_PIN = GPIO_NUM_15;
const gpio_num_t RELAY_PIN   = GPIO_NUM_16;
const gpio_num_t BUTTON_PIN  = GPIO_NUM_21;

const int SDA_PIN = 17;
const int SCL_PIN = 18;

// Native USB:
//
// GPIO19 = USB D-
// GPIO20 = USB D+
//
// These are handled by the ESP32-S3 USB peripheral.
// We do NOT configure them manually with pinMode().


// ==================================================================
// USER SETTINGS
// ==================================================================

const unsigned long BUTTON_HOLD_TIME = 3000;


// ------------------------------------------------------------------
// WIFI + GOOGLE DRIVE BRIDGE
//
// Fill these in before testing Drive synchronization.
// ------------------------------------------------------------------

const char *WIFI_SSID = "Palomec";
const char *WIFI_PASSWORD = "Getstarted";

// Current Apps Script deployment URL.
const char *APPS_SCRIPT_URL =
  "https://script.google.com/macros/s/"
  "AKfycbzke5eN-5VNLriVYGjG8mH4-dEWb1km7iGoxhgm262S62JwzOVulGjC9ajhb3DQ8ZdRJA/"
  "exec";

const char *APPS_SCRIPT_TOKEN =
  "facildeconectar";

// Raw bytes per Drive request.
// 16 KiB keeps ESP RAM use bounded while still supporting very large files.
const size_t DOWNLOAD_CHUNK_SIZE = 16 * 1024;

// Policy limit only.  The whole file is NEVER stored in ESP RAM.
const uint64_t MAX_PHOTO_SIZE =
  100ULL * 1024ULL * 1024ULL;   // 100 MiB

const unsigned long WIFI_CONNECT_TIMEOUT_MS = 20000;
const unsigned long HTTP_TIMEOUT_MS = 10000;
const int DOWNLOAD_RETRIES = 3;


// TEST: 30 seconds
const unsigned long INACTIVITY_TIME = 30000;


// FINAL 30 MINUTES:
//
// const unsigned long INACTIVITY_TIME =
//     30UL * 60UL * 1000UL;


const float PERSON_TEMP_THRESHOLD = 28.0;


// AMG8833 interrupt thresholds

const float AMG_INTERRUPT_HIGH = 28.0;
const float AMG_INTERRUPT_LOW  = 0.0;
const float AMG_HYSTERESIS     = 27.0;


// ==================================================================
// OLED
// ==================================================================

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

const uint8_t OLED_ADDRESS = 0x3C;

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  -1
);


// ==================================================================
// RELAY
// ==================================================================

const int RELAY_ON  = HIGH;
const int RELAY_OFF = LOW;


// ==================================================================
// AMG8833
// ==================================================================

Adafruit_AMG88xx amg;

float pixels[64];


// ==================================================================
// USB MASS STORAGE HOST
// ==================================================================

#define USB_MOUNT_POINT "/usb"

static msc_host_device_handle_t mscDevice = NULL;
static msc_host_vfs_handle_t mscVfs = NULL;
volatile bool usbDeviceConnected = false;
volatile bool usbStatusChanged = false;
static QueueHandle_t usbEventQueue = NULL;

volatile bool usbTransferActive = false;
volatile bool abortUsbTransfer = false;

static uint8_t downloadDecodeBuffer[DOWNLOAD_CHUNK_SIZE + 4];

enum UsbEventType { USB_DEVICE_CONNECTED, USB_DEVICE_DISCONNECTED };
struct UsbEvent { UsbEventType type; uint8_t address; };


// ==================================================================
// SYSTEM STATE
// ==================================================================

// Survives deep sleep.
//
// false = manually powered OFF
// true  = system logically ON

RTC_DATA_ATTR bool systemOn = false;


// ==================================================================
// RUNTIME STATE
// ==================================================================

unsigned long lastPersonTime = 0;

bool previousButton = HIGH;

unsigned long buttonPressStart = 0;


// ==================================================================
// RELAY CONTROL
// ==================================================================

void setRelay(bool state)
{
  digitalWrite(
    RELAY_PIN,
    state ? RELAY_ON : RELAY_OFF
  );

  Serial.print("Relay: ");

  Serial.println(
    state ? "ON" : "OFF"
  );
}

void scanI2CBus()
{
  Serial.println();
  Serial.println("========================");
  Serial.println("I2C BUS SCAN");
  Serial.println("========================");

  int found = 0;

  for (uint8_t address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("FOUND I2C DEVICE: 0x");

      if (address < 0x10)
      {
        Serial.print("0");
      }

      Serial.println(address, HEX);
      found++;
    }
  }

  Serial.print("Devices found: ");
  Serial.println(found);
  Serial.println("========================");
}

// ==================================================================
// DISPLAY / FAKE PHOTO
// ==================================================================

void displayPhotos()
{
  display.ssd1306_command(
    SSD1306_DISPLAYON
  );

  display.clearDisplay();


  // ---------------------------------------------------------------
  // Frame
  // ---------------------------------------------------------------

  display.drawRect(
    0,
    0,
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    SSD1306_WHITE
  );


  // ---------------------------------------------------------------
  // Fake mountains
  // ---------------------------------------------------------------

  display.drawTriangle(
    5, 52,
    35, 20,
    65, 52,
    SSD1306_WHITE
  );


  display.drawTriangle(
    42, 52,
    82, 14,
    122, 52,
    SSD1306_WHITE
  );


  // ---------------------------------------------------------------
  // Fake sun
  // ---------------------------------------------------------------

  display.fillCircle(
    103,
    14,
    6,
    SSD1306_WHITE
  );


  // ---------------------------------------------------------------
  // Ground
  // ---------------------------------------------------------------

  display.drawLine(
    5,
    53,
    122,
    53,
    SSD1306_WHITE
  );


  // ---------------------------------------------------------------
  // USB indicator
  // ---------------------------------------------------------------

  display.setTextSize(1);
  display.setTextColor(
    SSD1306_WHITE
  );


  display.setCursor(
    3,
    3
  );


  if (usbDeviceConnected)
  {
    display.print("USB");
  }
  else
  {
    display.print("---");
  }


  // ---------------------------------------------------------------
  // Status
  // ---------------------------------------------------------------

  display.setCursor(
    7,
    56
  );

  display.print(
    "PHOTO FRAME AWAKE"
  );


  display.display();
}


// ==================================================================
// DISPLAY OFF
// ==================================================================

void sleepDisplay()
{
  Serial.println(
    "DISPLAY OFF"
  );

  display.clearDisplay();

  display.display();

  display.ssd1306_command(
    SSD1306_DISPLAYOFF
  );
}


// ==================================================================
// DISPLAY ON
// ==================================================================

void wakeDisplay()
{
  Serial.println(
    "DISPLAY ON"
  );

  display.ssd1306_command(
    SSD1306_DISPLAYON
  );

  displayPhotos();
}


// ==================================================================
// RELAY HOLD DURING DEEP SLEEP
// ==================================================================

void holdRelay()
{
  gpio_hold_en(
    RELAY_PIN
  );

  gpio_deep_sleep_hold_en();
}


// ==================================================================
// RESTORE RELAY AFTER DEEP-SLEEP WAKE
// ==================================================================

void restoreRelay(bool state)
{
  pinMode(
    RELAY_PIN,
    OUTPUT
  );


  digitalWrite(
    RELAY_PIN,
    state ? RELAY_ON : RELAY_OFF
  );


  gpio_hold_dis(
    RELAY_PIN
  );

  gpio_deep_sleep_hold_dis();
}


// ==================================================================
// WAIT FOR POWER BUTTON RELEASE
// ==================================================================

void waitForButtonRelease()
{
  Serial.println(
    "Release power button..."
  );


  while (
    digitalRead(BUTTON_PIN) == LOW
  )
  {
    delay(10);
  }


  delay(100);


  Serial.println(
    "Button released"
  );
}


// ==================================================================
// FULL MANUAL POWER OFF
// ==================================================================

void fullPowerOff()
{
  // Manual power button remains highest priority.
  // Ask any Drive transfer to stop before entering deep sleep.
  abortUsbTransfer = true;

  unsigned long abortWaitStarted = millis();

  while (
    usbTransferActive &&
    millis() - abortWaitStarted < 6000
  )
  {
    delay(20);
  }
  Serial.println();

  Serial.println(
    "========================"
  );

  Serial.println(
    "MANUAL POWER OFF"
  );

  Serial.println(
    "========================"
  );


  // ---------------------------------------------------------------
  // Save state
  // ---------------------------------------------------------------

  systemOn = false;


  // ---------------------------------------------------------------
  // Display OFF
  // ---------------------------------------------------------------

  sleepDisplay();


  // ---------------------------------------------------------------
  // Monitor power OFF
  // ---------------------------------------------------------------

  setRelay(false);


  // ---------------------------------------------------------------
  // Keep relay LOW during deep sleep
  // ---------------------------------------------------------------

  holdRelay();


  // ---------------------------------------------------------------
  // Finish current button press
  // ---------------------------------------------------------------

  waitForButtonRelease();


  // ---------------------------------------------------------------
  // Remove all previous wake sources
  // ---------------------------------------------------------------

  esp_sleep_disable_wakeup_source(
    ESP_SLEEP_WAKEUP_ALL
  );


  // ---------------------------------------------------------------
  // ONLY POWER BUTTON CAN WAKE
  // ---------------------------------------------------------------

  esp_sleep_enable_ext1_wakeup(
    (1ULL << BUTTON_PIN),
    ESP_EXT1_WAKEUP_ANY_LOW
  );


  Serial.println();

  Serial.println(
    "SYSTEM OFF"
  );

  Serial.println(
    "Relay OFF"
  );

  Serial.println(
    "Display OFF"
  );

  Serial.println(
    "AMG wake disabled"
  );

  Serial.println(
    "Press button to turn ON"
  );


  Serial.flush();


  delay(100);


  esp_deep_sleep_start();
}


// ==================================================================
// SENSOR INACTIVITY SLEEP
// ==================================================================

void sensorSleep()
{
  Serial.println();

  Serial.println(
    "========================"
  );

  Serial.println(
    "SENSOR SLEEP"
  );

  Serial.println(
    "========================"
  );


  // ---------------------------------------------------------------
  // Display OFF
  // ---------------------------------------------------------------

  sleepDisplay();


  // ---------------------------------------------------------------
  // Relay intentionally remains ON
  // ---------------------------------------------------------------

  setRelay(true);

  holdRelay();


  Serial.println(
    "Relay remains ON"
  );

  Serial.println(
    "Display OFF"
  );

  Serial.println(
    "USB/VGA/Wi-Fi would stop"
  );

  Serial.println(
    "ESP entering deep sleep"
  );


  // ---------------------------------------------------------------
  // Clear old AMG interrupt
  // ---------------------------------------------------------------

  amg.clearInterrupt();


  delay(50);


  // ---------------------------------------------------------------
  // Configure wake sources
  // ---------------------------------------------------------------

  esp_sleep_disable_wakeup_source(
    ESP_SLEEP_WAKEUP_ALL
  );


  uint64_t wakeMask =
      (1ULL << AMG_INT_PIN)
    | (1ULL << BUTTON_PIN);


  esp_sleep_enable_ext1_wakeup(
    wakeMask,
    ESP_EXT1_WAKEUP_ANY_LOW
  );


  Serial.println(
    "Waiting for thermal wake..."
  );


  Serial.flush();


  delay(100);


  esp_deep_sleep_start();
}


// ==================================================================
// POWER BUTTON
// ==================================================================

void handlePowerButton()
{
  bool button =
    digitalRead(
      BUTTON_PIN
    );


  // ---------------------------------------------------------------
  // New button press
  // ---------------------------------------------------------------

  if (
    button == LOW &&
    previousButton == HIGH
  )
  {
    buttonPressStart =
      millis();


    Serial.println(
      "Power button pressed"
    );
  }


  // ---------------------------------------------------------------
  // 3-second hold
  // ---------------------------------------------------------------

  if (
    systemOn &&
    button == LOW
  )
  {
    if (
      millis() - buttonPressStart
      >= BUTTON_HOLD_TIME
    )
    {
      Serial.println(
        "3-second hold detected"
      );


      // Never returns

      fullPowerOff();
    }
  }


  previousButton = button;
}


// ==================================================================
// I2C BUS RECOVERY / REINITIALIZATION
// ==================================================================

bool resetAndStartI2C()
{
  Serial.println("Resetting shared I2C bus...");

  // Explicitly release any Arduino I2C driver state left across
  // startup/deep-sleep transitions, then recreate the bus on 17/18.
  Wire.end();
  delay(20);

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(10);

  if (!Wire.begin(SDA_PIN, SCL_PIN, 100000))
  {
    Serial.println("ERROR: Wire.begin() failed");
    return false;
  }

  delay(50);
  Serial.println("I2C bus ready: SDA=17 SCL=18 @ 100 kHz");
  return true;
}


// ==================================================================
// AMG8833 SETUP
// ==================================================================

bool setupAMG()
{
  if (!amg.begin())
  {
    Serial.println(
      "ERROR: AMG8833 not detected"
    );

    return false;
  }


  pinMode(
    AMG_INT_PIN,
    INPUT_PULLUP
  );


  amg.setInterruptMode(
    AMG88xx_ABSOLUTE_VALUE
  );


  amg.setInterruptLevels(
    AMG_INTERRUPT_HIGH,
    AMG_INTERRUPT_LOW,
    AMG_HYSTERESIS
  );


  amg.clearInterrupt();


  amg.enableInterrupt();


  Serial.println(
    "AMG8833 ready"
  );


  return true;
}


// ==================================================================
// OLED SETUP
// ==================================================================

bool setupOLED()
{
  if (
    !display.begin(
      SSD1306_SWITCHCAPVCC,
      OLED_ADDRESS
    )
  )
  {
    Serial.println(
      "ERROR: OLED not detected"
    );

    return false;
  }


  display.clearDisplay();

  display.display();


  Serial.println(
    "OLED ready"
  );


  return true;
}


// ==================================================================
// PERSON DETECTION
// ==================================================================

bool detectPerson()
{
  amg.readPixels(
    pixels
  );


  float hottest =
    pixels[0];


  for (
    int i = 1;
    i < 64;
    i++
  )
  {
    if (
      pixels[i] > hottest
    )
    {
      hottest =
        pixels[i];
    }
  }


  Serial.print(
    "Hottest: "
  );

  Serial.print(
    hottest,
    1
  );

  Serial.print(
    " C"
  );


  if (
    hottest >=
    PERSON_TEMP_THRESHOLD
  )
  {
    Serial.println(
      "  PERSON DETECTED"
    );

    return true;
  }


  Serial.println(
    "  no person"
  );


  return false;
}


// ==================================================================
// VGA RGB565 - STOP / RENDER / START ARCHITECTURE
// ==================================================================
// PHYSICAL RGB565 PINOUT:
//   R0/R1/R2/R3/R4       = GPIO4, GPIO5, GPIO6, GPIO7, GPIO8
//   G0/G1/G2/G3/G4/G5    = GPIO9, GPIO10, GPIO11, GPIO38, GPIO39, GPIO40
//   B0/B1/B2/B3/B4       = GPIO41, GPIO42, GPIO47, GPIO1, GPIO2
//   HSYNC = GPIO12, VSYNC = GPIO13
//   GPIO14 is the LCD peripheral PCLK ONLY; do not connect it to VGA.
//
// IMPORTANT:
//   RGB565 scanout is stable when ESP-IDF copies a STATIC PSRAM framebuffer
//   into small internal-SRAM bounce buffers. It became unstable when the CPU
//   modified PSRAM while scanout was active.
//
//   Therefore this version NEVER writes the VGA framebuffer while the panel
//   is scanning. To change the picture it does:
//
//     stop/delete RGB panel -> decode JPEG -> create panel (not started yet)
//     -> fill its PSRAM framebuffer -> start scanout
//
//   Once scanout starts, that framebuffer stays read-only.

const int VGA_WIDTH = 640;
const int VGA_HEIGHT = 480;
const int VGA_BOUNCE_LINES = 4;

static esp_lcd_panel_handle_t vgaPanel = NULL;
static uint16_t *vgaFB = NULL;
static bool vgaReady = false;

static inline uint16_t toRGB565(uint8_t r, uint8_t g, uint8_t b)
{
  return (uint16_t)(((uint16_t)(r & 0xF8) << 8) |
                    ((uint16_t)(g & 0xFC) << 3) |
                    ((uint16_t)b >> 3));
}

static esp_lcd_rgb_panel_config_t makeVgaConfig()
{
  esp_lcd_rgb_panel_config_t c = {};
  c.clk_src = LCD_CLK_SRC_DEFAULT;
  c.data_width = 16;
  c.bits_per_pixel = 16;
  c.num_fbs = 1;
  c.bounce_buffer_size_px = VGA_BOUNCE_LINES * VGA_WIDTH;
  c.dma_burst_size = 64;
  c.hsync_gpio_num = 12;
  c.vsync_gpio_num = 13;
  c.de_gpio_num = -1;
  c.pclk_gpio_num = 14;
  c.disp_gpio_num = -1;

  // ESP LCD RGB565 bus order: D0..D4=B0..B4,
  // D5..D10=G0..G5, D11..D15=R0..R4.
  c.data_gpio_nums[0]  = 41; // B0 LSB
  c.data_gpio_nums[1]  = 42; // B1
  c.data_gpio_nums[2]  = 47; // B2
  c.data_gpio_nums[3]  = 1;  // B3
  c.data_gpio_nums[4]  = 2;  // B4 MSB

  c.data_gpio_nums[5]  = 9;  // G0 LSB
  c.data_gpio_nums[6]  = 10; // G1
  c.data_gpio_nums[7]  = 11; // G2
  c.data_gpio_nums[8]  = 38; // G3
  c.data_gpio_nums[9]  = 39; // G4
  c.data_gpio_nums[10] = 40; // G5 MSB

  c.data_gpio_nums[11] = 4;  // R0 LSB
  c.data_gpio_nums[12] = 5;  // R1
  c.data_gpio_nums[13] = 6;  // R2
  c.data_gpio_nums[14] = 7;  // R3
  c.data_gpio_nums[15] = 8;  // R4 MSB

  c.timings.pclk_hz = 25175000;
  c.timings.h_res = VGA_WIDTH;
  c.timings.v_res = VGA_HEIGHT;
  c.timings.hsync_pulse_width = 96;
  c.timings.hsync_back_porch = 48;
  c.timings.hsync_front_porch = 16;
  c.timings.vsync_pulse_width = 2;
  c.timings.vsync_back_porch = 33;
  c.timings.vsync_front_porch = 10;
  c.timings.flags.hsync_idle_low = 0;
  c.timings.flags.vsync_idle_low = 0;

  c.flags.fb_in_psram = 1;
  c.flags.no_fb = 0;
  c.flags.bb_invalidate_cache = 1;
  return c;
}

static void stopVGA()
{
  vgaReady = false;
  vgaFB = NULL;

  if (vgaPanel) {
    Serial.println("Stopping VGA scanout before framebuffer update...");
    esp_err_t e = esp_lcd_panel_del(vgaPanel);
    if (e != ESP_OK) {
      Serial.printf("VGA panel delete warning: %s\n", esp_err_to_name(e));
    }
    vgaPanel = NULL;
    delay(20);
  }
}

static bool createVgaPanelUnstarted()
{
  esp_lcd_rgb_panel_config_t c = makeVgaConfig();
  esp_err_t e = esp_lcd_new_rgb_panel(&c, &vgaPanel);
  if (e != ESP_OK) {
    Serial.printf("VGA create failed: %s\n", esp_err_to_name(e));
    vgaPanel = NULL;
    return false;
  }

  void *fb = NULL;
  e = esp_lcd_rgb_panel_get_frame_buffer(vgaPanel, 1, &fb);
  if (e != ESP_OK || !fb) {
    Serial.printf("VGA framebuffer unavailable before init: %s\n", esp_err_to_name(e));
    esp_lcd_panel_del(vgaPanel);
    vgaPanel = NULL;
    return false;
  }

  vgaFB = static_cast<uint16_t *>(fb);
  return true;
}

static bool startVGA()
{
  if (!vgaPanel || !vgaFB) return false;

  esp_err_t e = esp_lcd_panel_reset(vgaPanel);
  if (e == ESP_OK) e = esp_lcd_panel_init(vgaPanel);
  if (e != ESP_OK) {
    Serial.printf("VGA init failed: %s\n", esp_err_to_name(e));
    esp_lcd_panel_del(vgaPanel);
    vgaPanel = NULL;
    vgaFB = NULL;
    return false;
  }

  vgaReady = true;
  return true;
}

static void vgaDrawRGB565ColorBars()
{
  if (!vgaFB) return;

  const uint16_t bars[8] = {
    0xF800, 0x07E0, 0x001F, 0xFFE0,
    0x07FF, 0xF81F, 0xFFFF, 0x0000
  };

  for (int y = 0; y < VGA_HEIGHT; ++y) {
    uint16_t *row = vgaFB + (size_t)y * VGA_WIDTH;
    for (int x = 0; x < VGA_WIDTH; ++x) {
      row[x] = bars[(x * 8) / VGA_WIDTH];
    }
    if ((y & 15) == 0) taskYIELD();
  }
}

bool setupVGA()
{
  Serial.println();
  Serial.println("========================");
  Serial.println("VGA RGB565 STOP-RENDER-START");
  Serial.println("========================");
  Serial.println("640x480 @ 25.175 MHz");
  Serial.println("PSRAM framebuffer + 4-line SRAM bounce buffers");
  Serial.println("Framebuffer is read-only whenever scanout is active");

  stopVGA();
  if (!createVgaPanelUnstarted()) return false;

  Serial.println("Prefilling startup RGB565 color bars...");
  vgaDrawRGB565ColorBars();

  if (!startVGA()) return false;

  Serial.println("VGA STARTUP BARS ACTIVE");
  Serial.println("Waiting for USB JPEG...");
  return true;
}

bool loadFilePSRAM(const char *path, uint8_t **buf, size_t *len)
{
  *buf = NULL;
  *len = 0;
  FILE *f = fopen(path, "rb");
  if (!f) return false;

  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  rewind(f);
  if (n <= 0) {
    fclose(f);
    return false;
  }

  uint8_t *p = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, (size_t)n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
  );
  if (!p) {
    fclose(f);
    return false;
  }

  size_t got = fread(p, 1, (size_t)n, f);
  fclose(f);
  if (got != (size_t)n) {
    free(p);
    return false;
  }

  *buf = p;
  *len = got;
  return true;
}

bool showJpegVGA(const char *path)
{
  Serial.printf("\nVGA RGB565 JPEG: %s\n", path);
  Serial.println("Stopping VGA before JPEG/PSRAM work...");

  // Absolutely no LCD scanout while JPEG data or the RGB565 framebuffer
  // is being created in PSRAM.
  stopVGA();

  uint8_t *jpg = NULL;
  size_t jpgLen = 0;
  if (!loadFilePSRAM(path, &jpg, &jpgLen)) {
    Serial.println("JPEG read failed; restoring color bars.");
    setupVGA();
    return false;
  }

  jpeg_dec_config_t probeCfg = DEFAULT_JPEG_DEC_CONFIG();
  jpeg_dec_handle_t probe = NULL;
  if (jpeg_dec_open(&probeCfg, &probe) != JPEG_ERR_OK) {
    free(jpg);
    setupVGA();
    return false;
  }

  jpeg_dec_io_t pio = {};
  pio.inbuf = jpg;
  pio.inbuf_len = jpgLen;
  jpeg_dec_header_info_t src = {};
  jpeg_error_t je = jpeg_dec_parse_header(probe, &pio, &src);
  jpeg_dec_close(probe);
  if (je != JPEG_ERR_OK) {
    free(jpg);
    Serial.println("JPEG header failed; restoring color bars.");
    setupVGA();
    return false;
  }

  Serial.printf("Source %ux%u\n", src.width, src.height);

  float s = (float)VGA_WIDTH / src.width;
  float sy = (float)VGA_HEIGHT / src.height;
  if (sy < s) s = sy;
  if (s > 1.0f) s = 1.0f;
  int w = ((int)(src.width * s)) & ~7;
  int h = ((int)(src.height * s)) & ~7;
  if (w < 8) w = 8;
  if (h < 8) h = 8;

  if (w < src.width / 8 || h < src.height / 8 ||
      w > VGA_WIDTH || h > VGA_HEIGHT) {
    Serial.println("JPEG too large for decoder scaling at this VGA mode.");
    free(jpg);
    setupVGA();
    return false;
  }

  jpeg_dec_config_t cfg = DEFAULT_JPEG_DEC_CONFIG();
  cfg.output_type = JPEG_PIXEL_FORMAT_RGB888;
  cfg.scale.width = w;
  cfg.scale.height = h;

  jpeg_dec_handle_t dec = NULL;
  if (jpeg_dec_open(&cfg, &dec) != JPEG_ERR_OK) {
    free(jpg);
    setupVGA();
    return false;
  }

  jpeg_dec_io_t io = {};
  io.inbuf = jpg;
  io.inbuf_len = jpgLen;
  jpeg_dec_header_info_t info = {};
  je = jpeg_dec_parse_header(dec, &io, &info);

  int outLen = 0;
  if (je == JPEG_ERR_OK) je = jpeg_dec_get_outbuf_len(dec, &outLen);

  uint8_t *rgb = NULL;
  if (je == JPEG_ERR_OK) {
    rgb = static_cast<uint8_t *>(
      heap_caps_aligned_alloc(16, (size_t)outLen, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
    );
    if (!rgb) je = JPEG_ERR_NO_MEM;
  }

  if (je == JPEG_ERR_OK) {
    io.outbuf = rgb;
    je = jpeg_dec_process(dec, &io);
  }

  jpeg_dec_close(dec);
  free(jpg);

  if (je != JPEG_ERR_OK) {
    if (rgb) free(rgb);
    Serial.println("JPEG decode failed; restoring color bars.");
    setupVGA();
    return false;
  }

  Serial.printf("Decoded %ux%u; creating stopped VGA framebuffer...\n",
                info.width, info.height);

  if (!createVgaPanelUnstarted()) {
    free(rgb);
    return false;
  }

  // The panel exists but scanout has NOT started yet, so these PSRAM writes
  // cannot contend with the LCD bounce-buffer ISR.
  memset(vgaFB, 0, VGA_WIDTH * VGA_HEIGHT * sizeof(uint16_t));

  int x0 = (VGA_WIDTH - info.width) / 2;
  int y0 = (VGA_HEIGHT - info.height) / 2;

  for (int y = 0; y < info.height; ++y) {
    uint16_t *d = vgaFB + (size_t)(y0 + y) * VGA_WIDTH + x0;
    const uint8_t *srow = rgb + (size_t)y * info.width * 3;

    for (int x = 0; x < info.width; ++x) {
      d[x] = toRGB565(srow[x * 3], srow[x * 3 + 1], srow[x * 3 + 2]);
    }

    if ((y & 15) == 0) taskYIELD();
  }

  free(rgb);
  Serial.println("RGB565 PSRAM framebuffer complete; starting VGA scanout...");

  if (!startVGA()) {
    Serial.println("Could not restart VGA after JPEG render.");
    return false;
  }

  Serial.printf("VGA RGB565 JPEG DISPLAYED %ux%u\n", info.width, info.height);
  Serial.println("Framebuffer is now LOCKED read-only during scanout.");
  return true;
}

bool showFirstUsbJpegVGA()
{
  DIR *d = opendir(USB_MOUNT_POINT "/PHOTOS");
  if (!d) {
    Serial.println("No /PHOTOS yet; keeping VGA RGB565 bars.");
    return false;
  }

  struct dirent *e;
  bool ok = false;
  while ((e = readdir(d))) {
    String n = e->d_name;
    String l = n;
    l.toLowerCase();
    if (l.endsWith(".jpg") || l.endsWith(".jpeg")) {
      String p = String(USB_MOUNT_POINT "/PHOTOS/") + n;
      ok = showJpegVGA(p.c_str());
      if (ok) break;
    }
  }

  closedir(d);
  if (!ok) Serial.println("No decodable USB JPEG; VGA bars restored.");
  return ok;
}


// ==================================================================
// USB MASS STORAGE FUNCTIONS
// ==================================================================

static void mscEventCallback(const msc_host_event_t *event, void *arg)
{
  UsbEvent msg;
  if (event->event == msc_host_event_t::MSC_DEVICE_CONNECTED) {
    msg.type = USB_DEVICE_CONNECTED;
    msg.address = event->device.address;
    xQueueSend(usbEventQueue, &msg, 0);
  } else if (event->event == msc_host_event_t::MSC_DEVICE_DISCONNECTED) {
    msg.type = USB_DEVICE_DISCONNECTED;
    msg.address = 0;
    xQueueSend(usbEventQueue, &msg, 0);
  }
}

void listUSBFiles()
{
  Serial.println("\nFILES ON USB:");
  Serial.println("------------------------");
  DIR *dir = opendir(USB_MOUNT_POINT);
  if (!dir) {
    Serial.println("Could not open USB root directory");
    return;
  }
  struct dirent *entry;
  int count = 0;
  while ((entry = readdir(dir)) != NULL) {
    Serial.print("  ");
    Serial.println(entry->d_name);
    count++;
  }
  closedir(dir);
  Serial.println("------------------------");
  Serial.print("File entries: ");
  Serial.println(count);
}

bool writeUSBTestFile()
{
  const char *path = USB_MOUNT_POINT "/THIS_IS_A_LONG_FILENAME_TEST.txt";

  Serial.println("\nCreating long filename test...");

  errno = 0;

  FILE *file = fopen(path, "w");

  if (!file)
  {
    int errnum = errno;

    Serial.println("ERROR: Could not create long filename test file");

    Serial.print("Path: ");
    Serial.println(path);

    Serial.print("errno: ");
    Serial.println(errnum);

    Serial.print("reason: ");
    Serial.println(strerror(errnum));

    return false;
  }

  int written = fprintf(
    file,
    "ESP32-S3 PHOTO FRAME USB TEST\n"
    "USB mass storage is working.\n"
  );

  if (written < 0)
  {
    int errnum = errno;

    Serial.println("ERROR: fprintf() failed");

    Serial.print("errno: ");
    Serial.println(errnum);

    Serial.print("reason: ");
    Serial.println(strerror(errnum));

    fclose(file);

    return false;
  }

  if (fflush(file) != 0)
  {
    int errnum = errno;

    Serial.println("ERROR: fflush() failed");

    Serial.print("errno: ");
    Serial.println(errnum);

    Serial.print("reason: ");
    Serial.println(strerror(errnum));

    fclose(file);

    return false;
  }

  if (fclose(file) != 0)
  {
    int errnum = errno;

    Serial.println("ERROR: fclose() failed");

    Serial.print("errno: ");
    Serial.println(errnum);

    Serial.print("reason: ");
    Serial.println(strerror(errnum));

    return false;
  }

  Serial.print("Write successful, bytes written: ");
  Serial.println(written);

  return true;
}

bool readUSBTestFile()
{
  const char *path = USB_MOUNT_POINT "/THIS_IS_A_LONG_FILENAME_TEST.txt";
  Serial.println("\nReading long filename test:");
  Serial.println("------------------------");
  FILE *file = fopen(path, "r");
  if (!file) {
    Serial.println("ERROR: Could not open long filename test file");
    return false;
  }
  char buffer[128];
  while (fgets(buffer, sizeof(buffer), file)) Serial.print(buffer);
  fclose(file);
  Serial.println("------------------------");
  return true;
}


// ==================================================================
// WIFI + DRIVE CHUNKED DOWNLOAD
// ==================================================================

struct DrivePhoto
{
  String id;
  String name;
  String mimeType;
  uint64_t size;
};

const int MAX_DRIVE_PHOTOS = 128;


// ------------------------------------------------------------------
// URL ENCODING
// ------------------------------------------------------------------

String urlEncode(const String &input)
{
  const char hex[] = "0123456789ABCDEF";
  String out;
  out.reserve(input.length() * 3);

  for (size_t i = 0; i < input.length(); i++)
  {
    uint8_t c = (uint8_t)input[i];

    if (
      (c >= 'a' && c <= 'z') ||
      (c >= 'A' && c <= 'Z') ||
      (c >= '0' && c <= '9') ||
      c == '-' || c == '_' || c == '.' || c == '~'
    )
    {
      out += (char)c;
    }
    else
    {
      out += '%';
      out += hex[(c >> 4) & 0x0F];
      out += hex[c & 0x0F];
    }
  }

  return out;
}


// ------------------------------------------------------------------
// VERY SMALL JSON FIELD HELPERS
//
// These are intentionally limited to the predictable JSON returned by
// our own Apps Script bridge, so no additional JSON library is needed.
// ------------------------------------------------------------------

bool jsonStringField(
  const String &json,
  const char *key,
  String &value
)
{
  String pattern = "\"";
  pattern += key;
  pattern += "\"";

  int p = json.indexOf(pattern);

  if (p < 0)
  {
    return false;
  }

  p = json.indexOf(':', p + pattern.length());

  if (p < 0)
  {
    return false;
  }

  p++;

  while (p < (int)json.length() && isspace((unsigned char)json[p]))
  {
    p++;
  }

  if (p >= (int)json.length() || json[p] != '"')
  {
    return false;
  }

  p++;

  value = "";

  while (p < (int)json.length())
  {
    char c = json[p++];

    if (c == '"')
    {
      return true;
    }

    if (c == '\\' && p < (int)json.length())
    {
      char esc = json[p++];

      switch (esc)
      {
        case '"': value += '"'; break;
        case '\\': value += '\\'; break;
        case '/': value += '/'; break;
        case 'b': value += '\b'; break;
        case 'f': value += '\f'; break;
        case 'n': value += '\n'; break;
        case 'r': value += '\r'; break;
        case 't': value += '\t'; break;

        // Keep uncommon \uXXXX sequences harmless for now.
        // Typical camera filenames are ASCII.
        case 'u':
          value += '_';

          for (int i = 0; i < 4 && p < (int)json.length(); i++)
          {
            p++;
          }
          break;

        default:
          value += esc;
          break;
      }
    }
    else
    {
      value += c;
    }
  }

  return false;
}


bool jsonUInt64Field(
  const String &json,
  const char *key,
  uint64_t &value
)
{
  String pattern = "\"";
  pattern += key;
  pattern += "\"";

  int p = json.indexOf(pattern);

  if (p < 0)
  {
    return false;
  }

  p = json.indexOf(':', p + pattern.length());

  if (p < 0)
  {
    return false;
  }

  p++;

  while (p < (int)json.length() && isspace((unsigned char)json[p]))
  {
    p++;
  }

  bool quoted = false;

  if (p < (int)json.length() && json[p] == '"')
  {
    quoted = true;
    p++;
  }

  uint64_t result = 0;
  bool foundDigit = false;

  while (p < (int)json.length())
  {
    char c = json[p];

    if (c < '0' || c > '9')
    {
      break;
    }

    foundDigit = true;
    result = result * 10ULL + (uint64_t)(c - '0');
    p++;
  }

  if (!foundDigit)
  {
    return false;
  }

  if (quoted && p < (int)json.length() && json[p] != '"')
  {
    return false;
  }

  value = result;

  return true;
}


// ------------------------------------------------------------------
// WIFI
// ------------------------------------------------------------------

bool wifiSettingsConfigured()
{
  return
    String(WIFI_SSID) != "PUT_YOUR_WIFI_SSID_HERE" &&
    String(WIFI_PASSWORD) != "PUT_YOUR_WIFI_PASSWORD_HERE" &&
    String(APPS_SCRIPT_TOKEN) != "PUT_YOUR_EXISTING_SECRET_TOKEN_HERE";
}


bool ensureWiFi()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    return true;
  }

  if (!wifiSettingsConfigured())
  {
    Serial.println();
    Serial.println("WiFi/Apps Script credentials not configured.");
    Serial.println("USB storage will still work; Drive sync skipped.");

    return false;
  }

  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  Serial.print("Current task stack free before WiFi: ");
  Serial.print(uxTaskGetStackHighWaterMark(NULL));
  Serial.println(" words");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long started = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - started < WIFI_CONNECT_TIMEOUT_MS
  )
  {
    handlePowerButton();
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi connection failed.");

    return false;
  }

  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());

  return true;
}


// ------------------------------------------------------------------
// HTTP GET
//
// TLS certificate verification is disabled for this prototype.
// Before final deployment we should install Google's trusted root CA.
// ------------------------------------------------------------------

bool httpsGetString(
  const String &url,
  String &responseBody
)
{
  // Handle redirects ourselves so we can see exactly where an
  // Apps Script request is spending its time.
  const int MAX_HTTP_REDIRECTS = 5;

  String currentUrl = url;
  responseBody = "";

  for (int redirectCount = 0;
       redirectCount <= MAX_HTTP_REDIRECTS;
       redirectCount++)
  {
    Serial.println();
    Serial.println("------------------------");
    Serial.print("HTTP request #");
    Serial.println(redirectCount + 1);

    Serial.print("Free heap before request: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");

    Serial.print("URL length: ");
    Serial.println(currentUrl.length());

    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(HTTP_TIMEOUT_MS / 1000);

    HTTPClient http;

    // IMPORTANT:
    // Do not let HTTPClient automatically follow Google redirects.
    // We want to observe and follow each redirect ourselves.
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.setReuse(false);

    Serial.println("HTTP begin...");

    unsigned long beginStarted = millis();

    if (!http.begin(client, currentUrl))
    {
      Serial.print("HTTP begin FAILED after ");
      Serial.print(millis() - beginStarted);
      Serial.println(" ms");

      return false;
    }

    Serial.print("HTTP begin OK in ");
    Serial.print(millis() - beginStarted);
    Serial.println(" ms");

    Serial.println("HTTP GET starting...");

    unsigned long getStarted = millis();

    int code = http.GET();

    unsigned long getElapsed =
      millis() - getStarted;

    Serial.print("HTTP GET returned: ");
    Serial.println(code);

    Serial.print("HTTP GET elapsed: ");
    Serial.print(getElapsed);
    Serial.println(" ms");

    Serial.print("Reported content length: ");
    Serial.println(http.getSize());

    Serial.print("Free heap after GET: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");

    if (code < 0)
    {
      Serial.print("HTTP transport error: ");
      Serial.println(
        HTTPClient::errorToString(code).c_str()
      );

      http.end();

      return false;
    }

    // Google Apps Script commonly redirects from script.google.com
    // to a googleusercontent.com endpoint.
    if (
      code == HTTP_CODE_MOVED_PERMANENTLY ||
      code == HTTP_CODE_FOUND ||
      code == HTTP_CODE_SEE_OTHER ||
      code == HTTP_CODE_TEMPORARY_REDIRECT ||
      code == HTTP_CODE_PERMANENT_REDIRECT
    )
    {
      String location =
        http.getLocation();

      Serial.print("Redirect received. Location length: ");
      Serial.println(location.length());

      if (location.length() == 0)
      {
        Serial.println("ERROR: Redirect had no Location header.");

        http.end();

        return false;
      }

      // Print only the destination host/path prefix, not the complete
      // redirect URL, because it can contain temporary authorization data.
      int schemePos =
        location.indexOf("://");

      int hostStart =
        schemePos >= 0
          ? schemePos + 3
          : 0;

      int pathStart =
        location.indexOf('/', hostStart);

      String destination =
        pathStart >= 0
          ? location.substring(0, pathStart)
          : location;

      Serial.print("Redirect destination: ");
      Serial.println(destination);

      http.end();

      currentUrl = location;

      Serial.println("Following redirect manually...");

      continue;
    }

    if (code != HTTP_CODE_OK)
    {
      Serial.print("HTTP GET failed. Code: ");
      Serial.println(code);

      Serial.println("Reading error response body...");

      unsigned long bodyStarted =
        millis();

      responseBody =
        http.getString();

      Serial.print("Error body read in ");
      Serial.print(millis() - bodyStarted);
      Serial.println(" ms");

      Serial.print("Error body length: ");
      Serial.println(responseBody.length());

      if (responseBody.length())
      {
        Serial.println(responseBody);
      }

      http.end();

      return false;
    }

    Serial.println("HTTP 200 OK");
    Serial.println("Reading response body...");

    unsigned long bodyStarted =
      millis();

    responseBody =
      http.getString();

    unsigned long bodyElapsed =
      millis() - bodyStarted;

    Serial.print("Response body read in ");
    Serial.print(bodyElapsed);
    Serial.println(" ms");

    Serial.print("Response body length: ");
    Serial.println(responseBody.length());

    Serial.print("Free heap after body: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");

    http.end();

    Serial.println("------------------------");

    return true;
  }

  Serial.println(
    "ERROR: Too many HTTP redirects."
  );

  return false;
}


// ------------------------------------------------------------------
// READ DRIVE PHOTO LIST
// ------------------------------------------------------------------

int parseDrivePhotoList(
  const String &json,
  DrivePhoto photos[],
  int maxPhotos
)
{
  int photosPos = json.indexOf("\"photos\"");

  if (photosPos < 0)
  {
    return 0;
  }

  int arrayStart = json.indexOf('[', photosPos);

  if (arrayStart < 0)
  {
    return 0;
  }

  int p = arrayStart + 1;
  int count = 0;

  while (p < (int)json.length() && count < maxPhotos)
  {
    int objectStart = json.indexOf('{', p);

    if (objectStart < 0)
    {
      break;
    }

    bool inString = false;
    bool escaped = false;
    int depth = 0;
    int objectEnd = -1;

    for (int i = objectStart; i < (int)json.length(); i++)
    {
      char c = json[i];

      if (inString)
      {
        if (escaped)
        {
          escaped = false;
        }
        else if (c == '\\')
        {
          escaped = true;
        }
        else if (c == '"')
        {
          inString = false;
        }

        continue;
      }

      if (c == '"')
      {
        inString = true;
      }
      else if (c == '{')
      {
        depth++;
      }
      else if (c == '}')
      {
        depth--;

        if (depth == 0)
        {
          objectEnd = i;
          break;
        }
      }
    }

    if (objectEnd < 0)
    {
      break;
    }

    String objectJson =
      json.substring(objectStart, objectEnd + 1);

    DrivePhoto photo;

    if (
      jsonStringField(objectJson, "id", photo.id) &&
      jsonStringField(objectJson, "name", photo.name) &&
      jsonStringField(objectJson, "mimeType", photo.mimeType) &&
      jsonUInt64Field(objectJson, "size", photo.size)
    )
    {
      photos[count++] = photo;
    }

    p = objectEnd + 1;
  }

  return count;
}


bool fetchDrivePhotoList(
  DrivePhoto photos[],
  int &photoCount
)
{
  photoCount = 0;

  if (!ensureWiFi())
  {
    return false;
  }

  String url = APPS_SCRIPT_URL;
  url += "?action=list&token=";
  url += urlEncode(APPS_SCRIPT_TOKEN);

  Serial.println();
  Serial.println("Requesting Google Drive photo list...");
  Serial.println("HTTP redirect diagnostics enabled.");

  String body;

  if (!httpsGetString(url, body))
  {
    Serial.println("Drive list request failed.");

    return false;
  }

  if (body.indexOf("\"success\":true") < 0)
  {
    Serial.println("Drive list returned an error:");
    Serial.println(body);

    return false;
  }

  photoCount =
    parseDrivePhotoList(
      body,
      photos,
      MAX_DRIVE_PHOTOS
    );

  Serial.print("Drive photos parsed: ");
  Serial.println(photoCount);

  return true;
}


// ------------------------------------------------------------------
// FAT-SAFE LONG FILENAME
// ------------------------------------------------------------------

String sanitizeFilename(const String &source)
{
  String out;
  out.reserve(source.length());

  for (size_t i = 0; i < source.length(); i++)
  {
    char c = source[i];

    if (
      c == '/' || c == '\\' || c == ':' || c == '*' ||
      c == '?' || c == '"' || c == '<' || c == '>' ||
      c == '|' || (uint8_t)c < 32
    )
    {
      out += '_';
    }
    else
    {
      out += c;
    }

    // Keep plenty of room under FAT's 255-character LFN limit.
    if (out.length() >= 220)
    {
      break;
    }
  }

  out.trim();

  if (out.length() == 0)
  {
    out = "UNNAMED.JPG";
  }

  return out;
}


bool ensurePhotosDirectory()
{
  const char *path = USB_MOUNT_POINT "/PHOTOS";

  struct stat st;

  if (stat(path, &st) == 0)
  {
    return S_ISDIR(st.st_mode);
  }

  if (mkdir(path, 0777) == 0)
  {
    Serial.println("Created /PHOTOS directory.");

    return true;
  }

  Serial.print("Could not create /PHOTOS. errno=");
  Serial.print(errno);
  Serial.print(" ");
  Serial.println(strerror(errno));

  return false;
}


bool fileMatchesSize(
  const String &path,
  uint64_t expectedSize
)
{
  struct stat st;

  if (stat(path.c_str(), &st) != 0)
  {
    return false;
  }

  return
    S_ISREG(st.st_mode) &&
    (uint64_t)st.st_size == expectedSize;
}


// ------------------------------------------------------------------
// GET ONE BASE64-ENCODED DRIVE RANGE
// ------------------------------------------------------------------

bool fetchDriveChunk(
  const DrivePhoto &photo,
  uint64_t offset,
  size_t requestedLength,
  size_t &decodedLength
)
{
  decodedLength = 0;

  String url = APPS_SCRIPT_URL;
  url += "?action=chunk&id=";
  url += urlEncode(photo.id);
  url += "&offset=";
  url += String((unsigned long long)offset);
  url += "&length=";
  url += String((unsigned long)requestedLength);
  url += "&token=";
  url += urlEncode(APPS_SCRIPT_TOKEN);

  for (int attempt = 1; attempt <= DOWNLOAD_RETRIES; attempt++)
  {
    if (abortUsbTransfer)
    {
      return false;
    }

    String body;

    if (!httpsGetString(url, body))
    {
      Serial.print("Chunk request retry ");
      Serial.print(attempt);
      Serial.print("/");
      Serial.println(DOWNLOAD_RETRIES);

      delay(250);

      continue;
    }

    if (body.indexOf("\"success\":true") < 0)
    {
      Serial.println("Chunk endpoint returned an error:");
      Serial.println(body);

      delay(250);

      continue;
    }

    String base64Data;
    uint64_t returnedOffset = 0;
    uint64_t rawLength = 0;

    if (
      !jsonStringField(body, "data", base64Data) ||
      !jsonUInt64Field(body, "offset", returnedOffset) ||
      !jsonUInt64Field(body, "length", rawLength)
    )
    {
      Serial.println("Could not parse chunk response.");

      delay(250);

      continue;
    }

    if (returnedOffset != offset)
    {
      Serial.println("Chunk offset mismatch.");

      delay(250);

      continue;
    }

    size_t outputLength = 0;

    int decodeResult =
      mbedtls_base64_decode(
        downloadDecodeBuffer,
        sizeof(downloadDecodeBuffer),
        &outputLength,
        (const unsigned char *)base64Data.c_str(),
        base64Data.length()
      );

    if (decodeResult != 0)
    {
      Serial.print("Base64 decode failed: ");
      Serial.println(decodeResult);

      delay(250);

      continue;
    }

    if (outputLength != (size_t)rawLength)
    {
      Serial.println("Decoded chunk length mismatch.");

      delay(250);

      continue;
    }

    decodedLength = outputLength;

    return true;
  }

  return false;
}


// ------------------------------------------------------------------
// STREAM ONE PHOTO TO USB
// ------------------------------------------------------------------

bool downloadPhotoToUSB(const DrivePhoto &photo)
{
  if (
    photo.mimeType != "image/jpeg" &&
    photo.mimeType != "image/png"
  )
  {
    return false;
  }

  if (photo.size == 0 || photo.size > MAX_PHOTO_SIZE)
  {
    Serial.print("Skipping size outside limit: ");
    Serial.println(photo.name);

    return false;
  }

  if (!ensurePhotosDirectory())
  {
    return false;
  }

  String safeName = sanitizeFilename(photo.name);

  String finalPath = USB_MOUNT_POINT "/PHOTOS/";
  finalPath += safeName;

  if (fileMatchesSize(finalPath, photo.size))
  {
    Serial.print("Already on USB: ");
    Serial.println(safeName);

    return true;
  }

  const char *tempPath =
    USB_MOUNT_POINT "/PHOTOS/DOWNLOAD.TMP";

  remove(tempPath);

  FILE *file = fopen(tempPath, "wb");

  if (!file)
  {
    Serial.print("Could not create DOWNLOAD.TMP. errno=");
    Serial.print(errno);
    Serial.print(" ");
    Serial.println(strerror(errno));

    return false;
  }

  Serial.println();
  Serial.print("Downloading: ");
  Serial.println(photo.name);

  Serial.print("Size: ");
  Serial.print((unsigned long long)photo.size);
  Serial.println(" bytes");

  usbTransferActive = true;
  abortUsbTransfer = false;

  uint64_t offset = 0;
  uint64_t sinceFlush = 0;
  int lastPercent = -1;
  bool success = true;

  while (offset < photo.size)
  {
    if (abortUsbTransfer)
    {
      Serial.println("Download aborted.");

      success = false;
      break;
    }

    size_t requestLength =
      (size_t)min(
        (uint64_t)DOWNLOAD_CHUNK_SIZE,
        photo.size - offset
      );

    size_t decodedLength = 0;

    if (
      !fetchDriveChunk(
        photo,
        offset,
        requestLength,
        decodedLength
      )
    )
    {
      Serial.println("Failed to receive chunk.");

      success = false;
      break;
    }

    if (decodedLength == 0)
    {
      Serial.println("Received empty chunk.");

      success = false;
      break;
    }

    size_t written =
      fwrite(
        downloadDecodeBuffer,
        1,
        decodedLength,
        file
      );

    if (written != decodedLength)
    {
      Serial.print("USB write failed. errno=");
      Serial.print(errno);
      Serial.print(" ");
      Serial.println(strerror(errno));

      success = false;
      break;
    }

    offset += decodedLength;
    sinceFlush += decodedLength;

    if (sinceFlush >= 1024ULL * 1024ULL)
    {
      fflush(file);
      sinceFlush = 0;
    }

    int percent =
      (int)((offset * 100ULL) / photo.size);

  if (percent != lastPercent && (percent % 5 == 0 || percent == 100))
{
  Serial.println();
  Serial.println("================================");
  Serial.print("       DOWNLOAD: ");
  Serial.print(percent);
  Serial.println("%");
  Serial.println("================================");
  Serial.println();

  lastPercent = percent;
}
  }

  if (success)
  {
    if (fflush(file) != 0)
    {
      success = false;
    }
  }

  if (fclose(file) != 0)
  {
    success = false;
  }

  usbTransferActive = false;

  if (!success)
  {
    remove(tempPath);

    return false;
  }

  struct stat tempStat;

  if (
    stat(tempPath, &tempStat) != 0 ||
    (uint64_t)tempStat.st_size != photo.size
  )
  {
    Serial.println("Downloaded byte count verification FAILED.");

    remove(tempPath);

    return false;
  }

  // Replace an old incomplete/mismatched destination if present.
  remove(finalPath.c_str());

  if (rename(tempPath, finalPath.c_str()) != 0)
  {
    Serial.print("Rename to final filename failed. errno=");
    Serial.print(errno);
    Serial.print(" ");
    Serial.println(strerror(errno));

    remove(tempPath);

    return false;
  }

  Serial.print("Saved: ");
  Serial.println(finalPath);

  return true;
}


// ------------------------------------------------------------------
// SYNC MISSING DRIVE PHOTOS
// ------------------------------------------------------------------

void syncDriveToUSB()
{
  if (!wifiSettingsConfigured())
  {
    Serial.println();
    Serial.println("Drive sync not configured yet.");
    Serial.println("Set WIFI_SSID, WIFI_PASSWORD, and APPS_SCRIPT_TOKEN.");

    return;
  }

  DrivePhoto *photos =
    new (std::nothrow) DrivePhoto[MAX_DRIVE_PHOTOS];

  if (!photos)
  {
    Serial.println("ERROR: Could not allocate Drive photo list.");
    return;
  }

  int photoCount = 0;

  if (!fetchDrivePhotoList(photos, photoCount))
  {
    delete[] photos;
    return;
  }

  Serial.println();
  Serial.println("========================");
  Serial.println("DRIVE -> USB SYNC");
  Serial.println("========================");

  int downloaded = 0;
  int skipped = 0;
  int failed = 0;

  for (int i = 0; i < photoCount; i++)
  {
    handlePowerButton();

    if (abortUsbTransfer)
    {
      break;
    }

    DrivePhoto &photo = photos[i];

    if (
      photo.size == 0 ||
      photo.size > MAX_PHOTO_SIZE ||
      (
        photo.mimeType != "image/jpeg" &&
        photo.mimeType != "image/png"
      )
    )
    {
      skipped++;
      continue;
    }

    String path = USB_MOUNT_POINT "/PHOTOS/";
    path += sanitizeFilename(photo.name);

    if (fileMatchesSize(path, photo.size))
    {
      skipped++;
      continue;
    }

    if (downloadPhotoToUSB(photo))
    {
      downloaded++;
    }
    else
    {
      failed++;
    }
  }

  Serial.println();
  Serial.println("========================");
  Serial.println("SYNC COMPLETE");
  Serial.println("========================");

  Serial.print("Downloaded: ");
  Serial.println(downloaded);

  Serial.print("Skipped: ");
  Serial.println(skipped);

  Serial.print("Failed: ");
  Serial.println(failed);

  delete[] photos;
}


bool mountUSBDrive(uint8_t address)
{
  Serial.println("\n========================");
  Serial.println("USB MASS STORAGE FOUND");
  Serial.println("========================");
  Serial.print("USB address: ");
  Serial.println(address);

  esp_err_t err = msc_host_install_device(address, &mscDevice);
  if (err != ESP_OK) {
    Serial.print("MSC install failed: ");
    Serial.println(esp_err_to_name(err));
    return false;
  }

  msc_host_device_info_t info;
  err = msc_host_get_device_info(mscDevice, &info);
  if (err == ESP_OK) {
    Serial.println("\nUSB DRIVE INFO");
    Serial.print("Sector size: "); Serial.println(info.sector_size);
    Serial.print("Sector count: "); Serial.println(info.sector_count);
    uint64_t capacity = (uint64_t)info.sector_size * (uint64_t)info.sector_count;
    Serial.print("Capacity: "); Serial.print(capacity / (1024ULL * 1024ULL)); Serial.println(" MiB");
  }

  const esp_vfs_fat_mount_config_t mountConfig = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 8192,
    .disk_status_check_enable = false,
    .use_one_fat = false
  };

  err = msc_host_vfs_register(mscDevice, USB_MOUNT_POINT, &mountConfig, &mscVfs);
  if (err != ESP_OK) {
    Serial.print("FAT mount failed: ");
    Serial.println(esp_err_to_name(err));
    msc_host_uninstall_device(mscDevice);
    mscDevice = NULL;
    return false;
  }

  usbDeviceConnected = true;
  usbStatusChanged = true;
  Serial.println("\n========================");
  Serial.println("USB DRIVE MOUNTED");
  Serial.println("========================");

listUSBFiles();

bool writeOK = writeUSBTestFile();
bool readOK = writeOK ? readUSBTestFile() : false;

Serial.println(
  writeOK && readOK
    ? "\nUSB STORAGE TEST PASSED"
    : "\nUSB STORAGE TEST FAILED"
);

listUSBFiles();

// Display the first JPEG already present on USB. showJpegVGA() stops VGA
// before touching PSRAM and restarts scanout only after the complete frame
// is ready.
bool usbPhotoDisplayed = showFirstUsbJpegVGA();

// NOW START GOOGLE DRIVE SYNC
Serial.println();
Serial.println(">>> STARTING GOOGLE DRIVE SYNC <<<");
Serial.flush();

syncDriveToUSB();

Serial.println(">>> GOOGLE DRIVE SYNC RETURNED <<<");
Serial.flush();

listUSBFiles();

// If there was no usable JPEG before synchronization, try again now in case
// Drive downloaded one. Do not redraw an already-active frame unnecessarily.
if (!usbPhotoDisplayed) {
  showFirstUsbJpegVGA();
}

return true;
}

void usbHostTask(void *parameter)
{
  while (true) {
    uint32_t eventFlags;
    usb_host_lib_handle_events(portMAX_DELAY, &eventFlags);
  }
}

void usbApplicationTask(void *parameter)
{
  UsbEvent msg;
  while (true) {
    if (xQueueReceive(usbEventQueue, &msg, portMAX_DELAY)) {
      if (msg.type == USB_DEVICE_CONNECTED) {
        mountUSBDrive(msg.address);
      } else {
        Serial.println("\nUSB DRIVE DISCONNECTED");
        if (mscVfs) { msc_host_vfs_unregister(mscVfs); mscVfs = NULL; }
        if (mscDevice) { msc_host_uninstall_device(mscDevice); mscDevice = NULL; }
        usbDeviceConnected = false;
        usbStatusChanged = true;
      }
    }
  }
}

bool setupUSBHost()
{
  Serial.println("\nStarting USB MSC Host...");
  usbEventQueue = xQueueCreate(4, sizeof(UsbEvent));
  if (!usbEventQueue) {
    Serial.println("USB queue creation failed");
    return false;
  }

  usb_host_config_t hostConfig = {};
  hostConfig.skip_phy_setup = false;
  hostConfig.intr_flags = ESP_INTR_FLAG_LEVEL1;
  esp_err_t err = usb_host_install(&hostConfig);
  if (err != ESP_OK) {
    Serial.print("USB Host install failed: ");
    Serial.println(esp_err_to_name(err));
    return false;
  }

  msc_host_driver_config_t mscConfig = {};
  mscConfig.create_backround_task = true;
  mscConfig.task_priority = 5;
  mscConfig.stack_size = 4096;
  mscConfig.callback = mscEventCallback;
  mscConfig.callback_arg = NULL;

  err = msc_host_install(&mscConfig);
  if (err != ESP_OK) {
    Serial.print("MSC Host install failed: ");
    Serial.println(esp_err_to_name(err));
    return false;
  }

  if (xTaskCreate(usbHostTask, "USB Host", 4096, NULL, 2, NULL) != pdPASS) {
    Serial.println("USB Host task creation FAILED");
    return false;
  }
  if (xTaskCreate(usbApplicationTask, "USB App", 16384, NULL, 1, NULL) != pdPASS) {
    Serial.println("USB App task creation FAILED");
    return false;
  }

  Serial.println("USB MSC Host ready");
  Serial.println("Waiting for FAT flash drive...");
  return true;
}


// ==================================================================
// GET DEEP-SLEEP WAKE PINS
// ==================================================================

uint64_t getWakePins()
{
  if (
    esp_sleep_get_wakeup_cause()
    == ESP_SLEEP_WAKEUP_EXT1
  )
  {
    return
      esp_sleep_get_ext1_wakeup_status();
  }


  return 0;
}


// ==================================================================
// SETUP
// ==================================================================

void setup()
{
  Serial.begin(
    115200
  );


  delay(400);


  // ---------------------------------------------------------------
  // Power button
  // ---------------------------------------------------------------

  pinMode(
    BUTTON_PIN,
    INPUT_PULLUP
  );


  // I2C is deliberately initialized later, after the power/relay
  // state has been restored, immediately before AMG/OLED setup.


  // ---------------------------------------------------------------
  // Determine wake reason
  // ---------------------------------------------------------------

  uint64_t wakePins =
    getWakePins();


  Serial.println();

  Serial.println(
    "========================"
  );

  Serial.println(
    "PHOTO FRAME TEST"
  );

  Serial.println(
    "========================"
  );


  // =================================================================
  // BUTTON WOKE MANUALLY-OFF SYSTEM
  // =================================================================

  if (
    !systemOn &&
    (
      wakePins &
      (1ULL << BUTTON_PIN)
    )
  )
  {
    systemOn =
      true;


    Serial.println(
      "POWER BUTTON WAKE"
    );
  }


  // =================================================================
  // RESTORE RELAY
  // =================================================================

  if (
    systemOn
  )
  {
    restoreRelay(
      true
    );


    setRelay(
      true
    );
  }
  else
  {
    restoreRelay(
      false
    );


    setRelay(
      false
    );
  }


  // =================================================================
  // SYSTEM MANUALLY OFF
  // =================================================================

  if (
    !systemOn
  )
  {
    Serial.println(
      "SYSTEM OFF"
    );


    if (
      digitalRead(BUTTON_PIN)
      == LOW
    )
    {
      waitForButtonRelease();
    }


    holdRelay();


    esp_sleep_disable_wakeup_source(
      ESP_SLEEP_WAKEUP_ALL
    );


    esp_sleep_enable_ext1_wakeup(
      (1ULL << BUTTON_PIN),
      ESP_EXT1_WAKEUP_ANY_LOW
    );


    Serial.println(
      "Press button to turn ON"
    );


    Serial.flush();


    esp_deep_sleep_start();
  }


  // =================================================================
  // SYSTEM ON
  // =================================================================


  // ---------------------------------------------------------------
  // Shared I2C bus recovery
  // ---------------------------------------------------------------

  if (!resetAndStartI2C())
  {
    while (true)
    {
      delay(1000);
    }
  }
scanI2CBus();

  // ---------------------------------------------------------------
  // AMG8833
  // ---------------------------------------------------------------

  if (
    !setupAMG()
  )
  {
    while (true)
    {
      delay(1000);
    }
  }


  // ---------------------------------------------------------------
  // OLED
  // ---------------------------------------------------------------

  if (
    !setupOLED()
  )
  {
    while (true)
    {
      delay(1000);
    }
  }


  // ---------------------------------------------------------------
  // VGA
  // ---------------------------------------------------------------
  // OLED and AMG are initialized first so VGA cannot disturb their
  // startup sequence. If VGA fails, the rest of the frame continues.
  setupVGA();


  // ---------------------------------------------------------------
  // USB HOST
  // ---------------------------------------------------------------

  if (
    !setupUSBHost()
  )
  {
    Serial.println(
      "USB Host failed to initialize"
    );
  }


  // =================================================================
  // SHOW WAKE SOURCE
  // =================================================================

  if (
    wakePins &
    (1ULL << AMG_INT_PIN)
  )
  {
    Serial.println();

    Serial.println(
      "THERMAL SENSOR WAKE"
    );
  }


  // =================================================================
  // DISPLAY AWAKE
  // =================================================================

  wakeDisplay();


  Serial.println();

  Serial.println(
    "SYSTEM AWAKE"
  );


  // Start fresh inactivity period

  lastPersonTime =
    millis();
}


// ==================================================================
// LOOP
// ==================================================================

void loop()
{
  // =================================================================
  // PRIORITY #1:
  // POWER BUTTON
  // =================================================================

  handlePowerButton();


  // =================================================================
  // USB DISPLAY STATUS
  // =================================================================

  if (
    usbStatusChanged
  )
  {
    usbStatusChanged =
      false;


    // Redraw fake photo so USB indicator changes.

    displayPhotos();
  }


  // =================================================================
  // PRESENCE
  // =================================================================

  if (
    detectPerson()
  )
  {
    lastPersonTime =
      millis();
  }


  // =================================================================
  // INACTIVITY
  // =================================================================

  if (
    !usbTransferActive &&
    millis() - lastPersonTime
    >= INACTIVITY_TIME
  )
  {
    Serial.println();

    Serial.println(
      "30 seconds without person"
    );


    sensorSleep();
  }


  delay(100);
}
extern "C" void app_main()
{
  initArduino();
  setup();
  while (true) { loop(); }
}