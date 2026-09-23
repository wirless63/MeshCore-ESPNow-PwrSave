#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#include "MyMesh.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(board, display);
#endif

#ifdef ETHERNET_ENABLED
  #define ETHERNET_CLI_BANNER "MeshCore Repeater CLI"
  #include <helpers/nrf52/EthernetCLI.h>
#endif

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

static char command[160];
#ifdef ETHERNET_ENABLED
static char ethernet_command[160];
#endif

// For power saving
//unsigned long POWERSAVING_FIRSTSLEEP_SECS = 120; // The first sleep (if enabled) from boot

// Define Sleep variables and needed files
#ifdef WITH_ESPNOW_BRIDGE
  unsigned long previousMillis = 0;   
  unsigned int sleepDisabled = 0;    
  uint8_t ltslp_en;
  uint8_t dpslp_en;  
  uint16_t sleeptime;
  uint16_t awakeTime;
  uint8_t startHr;
  uint8_t startMin;  
  uint16_t duration;
  #include <helpers/CommonCLI.h>  
  #include "Melopero_RV3028.h"
  Melopero_RV3028 rtc;
#endif

#if defined(PIN_USER_BTN) // && defined(_SEEED_SENSECAP_SOLAR_H_)
static unsigned long userBtnDownAt = 0;
#define USER_BTN_HOLD_OFF_MILLIS 1500
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#ifdef HAS_EXTERNAL_WATCHDOG
  external_watchdog.begin();
#endif

#if defined(MESH_DEBUG) && defined(NRF52_PLATFORM)
  // give some extra time for serial to settle so
  // boot debug messages can be seen on terminal
  delay(5000);
#endif

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.setCursor(0, 0);
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) {
    MESH_DEBUG_PRINTLN("Radio init failed!");
    halt();
  }

  fast_rng.begin(radio_driver.getRngSeed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Repeater ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;
#ifdef ETHERNET_ENABLED
  ethernet_command[0] = 0;
#endif

  sensors.begin();

  the_mesh.begin(fs);

#if WITH_ESPNOW_BRIDGE == 1
	// Initialize RTC timing for deep sleep mode 6 Sep 2026
  Wire.begin();
  rtc.initI2C();
  rtc.set24HourMode();

// Get prefs from memory to enable/disable sleep modes and define sleep settings
  ltslp_en = the_mesh.getNodePrefs()->bridge_ltslp_enabled;    //boolean
  sleeptime = the_mesh.getNodePrefs()->bridge_ltslp_slptime;   //seconds
  awakeTime = the_mesh.getNodePrefs()->bridge_ltslp_awake;     //milliseconds

  dpslp_en = the_mesh.getNodePrefs()->bridge_dpslp_enabled;    //boolean
  startHr = the_mesh.getNodePrefs()->bridge_dpslp_starthr;     //UTC hrs
  startMin = the_mesh.getNodePrefs()->bridge_dpslp_startmin;   //minutes
  duration = the_mesh.getNodePrefs()->bridge_dpslp_duration;   //seconds
  esp_sleep_enable_timer_wakeup(sleepTime * 1000000ULL);
#endif
	
#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

#ifdef ETHERNET_ENABLED
  ethernet_start_task();
#endif

  // send out initial zero hop Advertisement to the mesh
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif

  board.onBootComplete();
}

void loop() {
  // Handle Serial CLI
  int len = strlen(command);
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
      Serial.print(c);
    }
    if (c == '\r') break;
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    Serial.print('\n');
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    reply[0] = 0;
#ifdef ETHERNET_ENABLED
    if (!ethernet_handle_command(command, reply)) {
      the_mesh.handleCommand(0, command, reply);
    }
#else
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
#endif
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }

#ifdef ETHERNET_ENABLED
  ethernet_loop_maintain();
  if (ethernet_read_line(ethernet_command, sizeof(ethernet_command))) {
    char reply[160];
    reply[0] = 0;
    if (!ethernet_handle_command(ethernet_command, reply)) {
      the_mesh.handleCommand(0, ethernet_command, reply);
    }
    ethernet_send_reply(reply);
    ethernet_command[0] = 0;
  }
#endif

#ifdef (WITH_ESPNOW_BRIDGE) && defined(PIN_USER_BTN)
int btnState = digitalRead(PIN_USER_BTN);
  if (btnState == LOW) {
    if (userBtnDownAt == 0) {
      userBtnDownAt = millis();
    } else if ((unsigned long)(millis() - userBtnDownAt) >= USER_BTN_HOLD_OFF_MILLIS) && (ltslp_en == 1 || dpslp_en == 1){
      sleepDisabled = 1;            //Flag to turn off sleep modes. Reboot to re-enable sleep mode(s)
      digitalWrite(35, HIGH);       //Flash the LED to show the sleep modes have been temporarily disabled
	    delay(50);
      digitalWrite(35, LOW);
	    delay(50);
      digitalWrite(35, HIGH);
	    delay(50);
      digitalWrite(35, LOW);
      }
  } else {
    userBtnDownAt = 0;
  }
#endif

#if defined(PIN_USER_BTN) && defined(_SEEED_SENSECAP_SOLAR_H_) && !defined(DISPLAY_CLASS)
  // Hold the user button to power off the SenseCAP Solar repeater.
  int btnState = digitalRead(PIN_USER_BTN);
  if (btnState == LOW) {
    if (userBtnDownAt == 0) {
      userBtnDownAt = millis();
    } else if ((unsigned long)(millis() - userBtnDownAt) >= USER_BTN_HOLD_OFF_MILLIS) {
      Serial.println("Powering off...");
      board.powerOff();  // does not return
    }
  } else {
    userBtnDownAt = 0;
  }
#endif  

  the_mesh.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();

#ifdef HAS_EXTERNAL_WATCHDOG
  external_watchdog.loop();
//#endif
//  if (the_mesh.getNodePrefs()->powersaving_enabled && !the_mesh.hasPendingWork()) {
//#if defined(NRF52_PLATFORM)
//    board.sleep(0); // nrf ignores seconds param, sleeps whenever possible
//#else
//    if (the_mesh.millisHasNowPassed(POWERSAVING_FIRSTSLEEP_SECS * 1000)) { // To check if it is time to sleep
//      board.sleep(30); // Sleep. Wake up after a while or when receiving a LoRa packet
//    }
//#endif
  }
if (WITH_ESPNOW_BRIDGE == 1 && sleepDisabled == 0){ 
    if (ltslp_en == 1){
    // Establish a continuous light sleep/awake cycle, if enabled. Hold if ESPNow or LoRa is busy
    unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= (awakeTime * 1000) && espnow_recving == 0 && espnow_sending == 0) { 
        previousMillis = currentMillis;
        board.sleep(sleepTime);   // Go to light sleep as defined in helpers/ESP32Board.h which addresses Lora busy hold
      }
    }
  // Establish a deep sleep starting with the node prefs UTC start hr/min, if enabled.  Hold if ESPNow is busy
    if (dpslp_en == 1 && espnow_recving == 0 && espnow_sending == 0){  
      if ((rtc.getHour() == startHr) && (rtc.getMinute() == startMin)) {
        board.enterDeepSleep(duration);    //Go into deep sleep as defined in helpers/ESP32Board.cpp
      }
    }
  }
}
