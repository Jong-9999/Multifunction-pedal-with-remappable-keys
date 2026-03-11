// ── HID Keycodes (USB HID Usage Tables, page 90) ─────────────
//https://usb.org/document-library/hid-usage-tables-17
// go to page 90 and check the desired UID
#define KEY_RIGHT  0x51
#define KEY_LEFT   0x52

/*
 * ============================================================
 *  BLE Page Turner for M5StampS3
 *
 *  Output:  BLE HID Keyboard
 *  Buttons: 2 momentary push buttons
 *  LED:     Onboard RGB (WS2812 on GPIO21) — status indicator
 *
 * ── LIBRARIES (install both via Library Manager) ─────────────
 *  1. "NimBLE-Arduino" by h2zero  — v2.x
 *     Sketch → Include Library → Manage Libraries → NimBLE-Arduino
 *
 *  2. "Adafruit NeoPixel" by Adafruit
 *     Sketch → Include Library → Manage Libraries → Adafruit NeoPixel
 *
 * ── BOARD SETTINGS (Arduino IDE) ─────────────────────────────
 *  Board:            ESP32S3/m5stamps3
 *  USB CDC On Boot:  Enabled   ← required for Serial over USB
 *  Partition Scheme: Default 4MB with spiffs
 *
 * ── WIRING ────────────────────────────────────────────────────
 *
 *        M5StampS3
 *        ┌─────────────────────────────┐
 *        │  [USB-C]         [RGB LED]  │
 *        │                             │
 *        │  G1 ──┤NEXT BTN├── GND      │
 *        │  G3 ──┤PREV BTN├── GND      │
 *        │                             │
 *        │  (Internal pull-ups used,   │
 *        │   no resistors needed)      │
 *        └─────────────────────────────┘
 *
 *  Any momentary push button — one leg to GPIO, other to GND.
 *
 * ── PAIRING ───────────────────────────────────────────────────
 *  1. Power on — RGB blinks blue = advertising
 *  2. Settings → Bluetooth on phone/tablet/Mac/PC
 *  3. Tap "PageTurner S3" to pair
 *  4. RGB solid green = connected and ready!
 *  5. Open your sheet music app and press the buttons!
 *
 * ── LED STATUS ────────────────────────────────────────────────
 *  Blinking blue  = waiting for connection / advertising
 *  Solid green    = connected
 *  Brief red      = boot sequence
 * ============================================================
 */

#include <Arduino.h>
#include <NimBLEDevice.h>      // NimBLE v2 — core
#include <NimBLEServer.h>      // NimBLE v2 — server
#include <NimBLEHIDDevice.h>   // NimBLE v2 — HID device
#include <Adafruit_NeoPixel.h> // Onboard RGB LED

// ── Pin Definitions ───────────────────────────────────────────
#define BTN_NEXT    1    // GPIO 1
#define BTN_PREV    3    // GPIO 3 
#define RGB_PIN     21   // Onboard RGB LED (M5StampS3)
#define RGB_COUNT   1

// ── Timing ────────────────────────────────────────────────────
#define DEBOUNCE_MS      30    // ms debounce window
#define HOLD_REPEAT_MS  70    // ms before hold-to-repeat fires

// ── HID Report Descriptor — Standard 8-byte keyboard ─────────
static const uint8_t hidReportDescriptor[] = {
  0x05, 0x01,  // Usage Page: Generic Desktop Controls
  0x09, 0x06,  // Usage: Keyboard
  0xA1, 0x01,  // Collection: Application
  0x85, 0x01,  //   Report ID: 1
  // Modifier keys (8 x 1-bit flags)
  0x75, 0x01,  //   Report Size: 1 bit
  0x95, 0x08,  //   Report Count: 8
  0x05, 0x07,  //   Usage Page: Keyboard/Keypad
  0x19, 0xE0,  //   Usage Minimum: Left Control
  0x29, 0xE7,  //   Usage Maximum: Right GUI
  0x15, 0x00,  //   Logical Minimum: 0
  0x25, 0x01,  //   Logical Maximum: 1
  0x81, 0x02,  //   Input: Data, Variable, Absolute
  // Reserved byte
  0x75, 0x08,  //   Report Size: 8 bits
  0x95, 0x01,  //   Report Count: 1
  0x81, 0x01,  //   Input: Constant
  // 6 simultaneous keycodes
  0x75, 0x08,  //   Report Size: 8 bits
  0x95, 0x06,  //   Report Count: 6
  0x15, 0x00,  //   Logical Minimum: 0
  0x25, 0x65,  //   Logical Maximum: 101
  0x05, 0x07,  //   Usage Page: Keyboard/Keypad
  0x19, 0x00,  //   Usage Minimum: 0
  0x29, 0x65,  //   Usage Maximum: 101
  0x81, 0x00,  //   Input: Data, Array, Absolute
  0xC0         // End Collection
};

// ── Globals ───────────────────────────────────────────────────
NimBLEServer*         pServer      = nullptr;
NimBLEHIDDevice*      pHID         = nullptr;
NimBLECharacteristic* pInput       = nullptr;
bool                  bleConnected = false;
Adafruit_NeoPixel     rgb(RGB_COUNT, RGB_PIN, NEO_GRB + NEO_KHZ800);

// ── Button state ──────────────────────────────────────────────
struct Button {
  uint8_t       pin;
  bool          lastRaw;
  bool          state;
  unsigned long lastDebounce;
  unsigned long holdStart;
};

Button btnNext = {BTN_NEXT, HIGH, HIGH, 0, 0};
Button btnPrev = {BTN_PREV, HIGH, HIGH, 0, 0};

// ── RGB helpers ───────────────────────────────────────────────
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  rgb.setPixelColor(0, rgb.Color(r, g, b));
  rgb.show();
}
void rgbOff()   { setRGB(  0,   0,   0); }
void rgbBlue()  { setRGB(  0,   0,  60); }
void rgbGreen() { setRGB(  0,  60,   0); }
void rgbRed()   { setRGB( 60,   0,   0); }

// ── Send one HID keypress (down + up) ─────────────────────────
void sendKey(uint8_t keycode) {
  if (!bleConnected || !pInput) return;

  // Key DOWN: [modifier, reserved, key1, key2..key6]
  uint8_t report[8] = {0x00, 0x00, keycode, 0x00, 0x00, 0x00, 0x00, 0x00};
  pInput->setValue(report, sizeof(report));
  pInput->notify();
  delay(15);

  // Key UP: all zeros
  memset(report, 0, sizeof(report));
  pInput->setValue(report, sizeof(report));
  pInput->notify();

}

// ── NimBLE v2 server callbacks ────────────────────────────────
// v2 signatures (confirmed from NimBLE-Arduino changelog):
//   onConnect    → (NimBLEServer*, NimBLEConnInfo&)
//   onDisconnect → (NimBLEServer*, NimBLEConnInfo&, int)
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pSvr, NimBLEConnInfo& connInfo) override {
    bleConnected = true;
    rgbGreen();
  }

  void onDisconnect(NimBLEServer* pSvr, NimBLEConnInfo& connInfo, int reason) override {
    bleConnected = false;
    rgbBlue();
    // pServer->advertiseOnDisconnect(true) set in setupBLE() handles restart
  }
};

// ── BLE + HID initialisation ──────────────────────────────────
void setupBLE() {
  NimBLEDevice::init("Justin's DTK pedal");

  // v2: setPower takes plain int8_t dBm — ESP_PWR_LVL_Pxx enum removed
  NimBLEDevice::setPower(9);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // v2: auto re-advertise when client disconnects
  pServer->advertiseOnDisconnect(true);

  pHID = new NimBLEHIDDevice(pServer);

  // v2 method names (all renamed from NimBLE-Arduino v1):
  //   setManufacturer(std::string)  was  manufacturer()->setValue()
  //   setPnp(...)                   was  pnp(...)
  //   setHidInfo(...)               was  hidInfo(...)
  //   setReportMap(...)             was  reportMap(...)
  //   getInputReport(id)            was  inputReport(id)
  //   getHidService()               was  hidService()
  pHID->setManufacturer(std::string("Justin made this"));
  pHID->setPnp(0x02, 0x05AC, 0x820A, 0x0210);  // Apple VID improves iOS pairing
  pHID->setHidInfo(0x00, 0x01);
  pHID->setReportMap((uint8_t*)hidReportDescriptor, sizeof(hidReportDescriptor));

  pInput = pHID->getInputReport(1);  // Report ID 1

  pHID->setBatteryLevel(85);  // Required by some OS/apps to complete HID profile
  pHID->startServices();

  // Advertising
  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->setAppearance(0x03C1);                            // HID Keyboard appearance
  pAdv->addServiceUUID(pHID->getHidService()->getUUID()); // v2: getHidService()
  pAdv->start();
}

// ── Button handler: debounce + press + hold-to-repeat ─────────
void handleButton(Button& btn, uint8_t keycode) {
  bool raw = digitalRead(btn.pin);

  if (raw != btn.lastRaw) {
    btn.lastDebounce = millis();
    btn.lastRaw = raw;
  }

  if ((millis() - btn.lastDebounce) < DEBOUNCE_MS) return;

  if (raw != btn.state) {
    btn.state = raw;
    if (btn.state == LOW) {
      sendKey(keycode);
      btn.holdStart = millis();
    }
  }

  // Hold-to-repeat
  if (btn.state == LOW && (millis() - btn.holdStart >= HOLD_REPEAT_MS)) {
    sendKey(keycode);
    btn.holdStart = millis();
  }
}

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(800);

  rgb.begin();
  rgb.setBrightness(80);
  rgbRed();
  delay(300);
  rgbOff();

  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_PREV, INPUT_PULLUP);

  setupBLE();
  rgbBlue();
}

// ─────────────────────────────────────────────────────────────
void loop() {
  if (!bleConnected) {
    unsigned long t = millis();
    if ((t / 600) % 2 == 0) rgbBlue();
    else                     rgbOff();
  }

  handleButton(btnNext, KEY_RIGHT);
  handleButton(btnPrev, KEY_LEFT);
  // No need to modify this only modify the define key left/right at the very top.
  delay(5);
}
