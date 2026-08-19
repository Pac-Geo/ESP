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

  THIS VERSION TESTS USB MASS STORAGE + FAT FILE ACCESS.

  It does NOT yet:

      - mount FAT32
      - list files
      - create files
      - download Google Drive photos

  Those are the next USB steps after enumeration succeeds.


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

#include "usb/msc_host.h"
#include "usb/msc_host_vfs.h"
#include "esp_vfs_fat.h"
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
  const char *path = USB_MOUNT_POINT "/FRAME_TEST.txt";
  Serial.println("\nCreating FRAME_TEST.txt...");
  FILE *file = fopen(path, "w");
  if (!file) {
    Serial.println("ERROR: Could not create FRAME_TEST.txt");
    return false;
  }
  fprintf(file, "ESP32-S3 PHOTO FRAME USB TEST\nUSB mass storage is working.\n");
  fclose(file);
  Serial.println("Write successful");
  return true;
}

bool readUSBTestFile()
{
  const char *path = USB_MOUNT_POINT "/FRAME_TEST.txt";
  Serial.println("\nReading FRAME_TEST.txt:");
  Serial.println("------------------------");
  FILE *file = fopen(path, "r");
  if (!file) {
    Serial.println("ERROR: Could not open FRAME_TEST.txt");
    return false;
  }
  char buffer[128];
  while (fgets(buffer, sizeof(buffer), file)) Serial.print(buffer);
  fclose(file);
  Serial.println("------------------------");
  return true;
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
  Serial.println(writeOK && readOK ? "\nUSB STORAGE TEST PASSED" : "\nUSB STORAGE TEST FAILED");
  listUSBFiles();
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
  if (xTaskCreate(usbApplicationTask, "USB App", 6144, NULL, 1, NULL) != pdPASS) {
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


  // ---------------------------------------------------------------
  // Start shared I2C
  // ---------------------------------------------------------------

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );


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