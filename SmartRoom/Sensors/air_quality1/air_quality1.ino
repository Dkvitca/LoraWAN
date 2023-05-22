
#include <GP2YDustSensor.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// The following lines define the necessary keys and identifiers for LoRaWAN communication.
// Please replace the placeholders with the actual values assigned by the TTN console.
////////////////////////////////////////////
static const u1_t PROGMEM APPEUI[8] = { 0x86, 0x41, 0x50, 0x02, 0xc8, 0x09, 0xb7, 0xd7 };
void os_getArtEui(u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}
static const u1_t PROGMEM DEVEUI[8] = { 0x86, 0x41, 0x50, 0x02, 0xc8, 0x09, 0xb7, 0xd7 };
void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}
static const u1_t PROGMEM APPKEY[16] = { 0x79, 0xf8 , 0x17 , 0xfc , 0x04 , 0x82 , 0x14 , 0x65 , 0xc7 , 0x19 , 0x41 , 0xa3 , 0xab , 0x5d , 0x60 , 0x2f };
void os_getDevKey(u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}
/////////////////////////////////////////////////////
int pirPin = 36;
int mq135Pin = 34;
int mq7Pin = 39;
const uint8_t SHARP_LED_PIN = 32;   // Sharp Dust/particle sensor Led Pin
const uint8_t SHARP_VO_PIN = 35;  
GP2YDustSensor dustSensor(GP2YDustSensorType::GP2Y1010AU0F, SHARP_LED_PIN, SHARP_VO_PIN);

// This structure represents a job to be executed by the LMIC library.
static osjob_t sendjob;
// This flag indicates whether the device should go into deep sleep after transmission.
bool GOTO_DEEPSLEEP = false;
// This array contains the names of the different sensor readings.
const char* sensor_names[] = { "movment", "air quality", "Co&Smoke", "dust" };

               //define the analog input pin for wind direction
///////////////////////////////////////////////////////
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).

// Define the desired time interval in minutes
#define TX_INTERVAL_MINUTES 5

#ifndef TX_INTERVAL_MINUTES
  #error "TX_INTERVAL_MINUTES is not defined. Please specify the desired time interval."
#endif

// Calculate the corresponding TX_INTERVAL in microseconds
#if TX_INTERVAL_MINUTES == 1
  #define TX_INTERVAL (1 * 60 * 1000000UL)   // 1 minute
#elif TX_INTERVAL_MINUTES == 10
  #define TX_INTERVAL (10 * 60 * 1000000UL)  // 10 minutes
#elif TX_INTERVAL_MINUTES == 60
  #define TX_INTERVAL (60 * 60 * 1000000UL)  // 60 minutes (1 hour)
#elif TX_INTERVAL_MINUTES == 5
  #define TX_INTERVAL (5 * 60 * 1000000UL) // 240 minutes (4 hours)
#elif TX_INTERVAL_MINUTES == 720
  #define TX_INTERVAL (720 * 60 * 1000000UL) // 720 minutes (12 hours)
#elif TX_INTERVAL_MINUTES == 1440
  #define TX_INTERVAL (1440 * 60 * 1000000UL) // 1440 minutes (24 hours)
#else
  #error "Invalid TX_INTERVAL_MINUTES value. Please choose a valid interval."
#endif
///////////////////////////////////////////////////////

// Saves the LMIC structure during DeepSleep
RTC_DATA_ATTR lmic_t RTC_LMIC;

const lmic_pinmap lmic_pins = {
  .nss = 5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 17,
  .dio = { 2, 14, LMIC_UNUSED_PIN },
};

// Function to print a hexadecimal value as two digits
void printHex2(unsigned v) {
  v &= 0xff; // Mask the value to keep only the lowest 8 bits
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

// Function to handle LoRaWAN events
void onEvent(ev_t ev) {
  Serial.print(os_getTime());// Print the current time
  Serial.print(": ");
  
  // Handle different LoRaWAN events
  switch (ev) {
    case EV_SCAN_TIMEOUT:// Timeout while scanning for networks
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:// Beacon found during scanning
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:// Missed beacon during scanning
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:// Beacon tracked during scanning
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:// Joining network
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:// Successfully joined network
      Serial.println(F("EV_JOINED"));
      {
         // Retrieve session keys
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
         // Print network information
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(nwkKey[i]);
        }
        Serial.println();
      }
      // Disable link check validation (automatically enabled during join, but not used in this example)
      LMIC_setLinkCheckMode(0);
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));// Joining network failed
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));// Rejoining network failed
      break;
    case EV_TXCOMPLETE:// Transmission complete
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }

      GOTO_DEEPSLEEP = true;// Set flag to go to deep sleep
      break;

    case EV_LOST_TSYNC:// Lost synchronization with network time
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET: // Reset event
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:// Data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD: // Link with network is dead
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:// Link with network is alive
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART: // Transmission started
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:// Transmission canceled
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:// Join transmission complete without receiving JoinAccept
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;

    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {// Current job running, skip sending
    Serial.println(F("OP_TXRXPEND, not sending"));

  } else {
    /////////////////////////////////////////////////////
    dustSensor.begin();
    int pirValue = digitalRead(pirPin);
    int mq135Value = analogRead(mq135Pin);
    int mq7Value = analogRead(mq7Pin);
    int dust = dustSensor.getDustDensity();

    /////////////////////////////////////////////////////

    // int -> byte array
    byte payload[8];
    payload[0] = lowByte(pirValue); 
    payload[1] = highByte(pirValue);
    payload[2] = lowByte(mq135Value);
    payload[3] = highByte(mq135Value);
    payload[4] = lowByte(mq7Value);
    payload[5] = highByte(mq7Value);
    payload[6] = lowByte(dust);
    payload[7] = highByte(dust);

    LMIC_setTxData2(10, payload, sizeof(payload), 0);
    Serial.print(F("Packet queued: "));
      // Print sensor values
    Serial.print("pirValue: ");
    Serial.print(pirValue);
    Serial.print(", mq135Value: ");
    Serial.print(mq135Value);
    Serial.print(", mq7Value: ");
    Serial.print(mq7Value);
    Serial.print(", dust: ");
    Serial.println(dust);
  }
}
// Next TX is scheduled after TX_COMPLETE event.


int averageAnalogRead(int pinToRead) {
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for (int x = 0; x < numberOfReadings; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);
}

// Map the input value from the input range to the output range using linear interpolation
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void SaveLMICToRTC(int deepsleep_sec) {
  Serial.println(F("Save LMIC to RTC"));

  // Save the LMIC state to the RTC_LMIC variable
  RTC_LMIC = LMIC;

  // ESP32 can't track millis during DeepSleep and no option to advance millis after DeepSleep.
  // Therefore reset DutyCycles

  // Get the current time in milliseconds
  unsigned long now = millis();

  // EU Like Bands
#if defined(CFG_LMIC_EU_like)
  Serial.println(F("Reset CFG_LMIC_EU_like band avail"));

  // Iterate over the bands and adjust the availability time by subtracting the deep sleep duration
  for (int i = 0; i < MAX_BANDS; i++) {
    ostime_t correctedAvail = RTC_LMIC.bands[i].avail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
    // Ensure the corrected availability time is not negative
    if (correctedAvail < 0) {
      correctedAvail = 0;
    }
    // Update the band availability time
    RTC_LMIC.bands[i].avail = correctedAvail;
  }

  // Adjust the global duty cycle availability time by subtracting the deep sleep duration
  RTC_LMIC.globalDutyAvail = RTC_LMIC.globalDutyAvail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
  if (RTC_LMIC.globalDutyAvail < 0) {
    RTC_LMIC.globalDutyAvail = 0;
  }
#else
  Serial.println(F("No DutyCycle recalculation function!"));
#endif
}


void LoadLMICFromRTC() {
  Serial.println(F("Load LMIC from RTC"));
  LMIC = RTC_LMIC;
}

void GoDeepSleep() {
  Serial.println(F("Go DeepSleep"));
  Serial.flush();
  esp_sleep_enable_timer_wakeup(TX_INTERVAL);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting"));
  pinMode(pirPin, INPUT);
  pinMode(mq135Pin, INPUT);
  pinMode(mq7Pin, INPUT);
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Load LMIC state from RTC memory
  LoadLMICFromRTC();

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
    os_runloop_once();

  if (GOTO_DEEPSLEEP && !(LMIC.opmode & OP_TXRXPEND)) {
    // Save LMIC state to RTC memory
    SaveLMICToRTC(TX_INTERVAL);
    // Go to deep sleep
    GoDeepSleep();
  }

}