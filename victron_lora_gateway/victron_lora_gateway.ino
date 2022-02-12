/*


    LoRaWAN Gateway for the Victron VE.Direct-Protocol based on an Adafruit Feather M0 LoRa

    The Victron device will be connected through additional Serial2 of the Feather M0.
    A level shifter has to be used, because the Victron Solar Chargers use 5V level, the Feather M0 3.3V.

    ----------------------------------------
    The code for parsing the VE.Direct-Protocol is from https://github.com/physee/Victron.Arduino-ESP8266

    Victron.Arduino-ESP8266
    A:Pim Rutgers
    E:pim@physee.eu

    Code to grab data from the VE.Direct-Protocol on Arduino / ESP8266.
    Tested on NodeMCU v1.0

    The fields of the serial commands are configured in "config.h"

*/
#include "config_victron.h"
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

char receivedChars[buffsize];                       // an array to store the received data
char tempChars[buffsize];                           // an array to manipulate the received data
char recv_label[num_keywords][label_bytes]  = {0};  // {0} tells the compiler to initalize it with 0.
char recv_value[num_keywords][value_bytes]  = {0};  // That does not mean it is filled with 0's
char value[num_keywords][value_bytes]       = {0};  // The array that holds the verified data
static byte blockindex = 0;
bool new_data = false;
bool blockend = false;

Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

long previousMillis = 0;
long interval = 60000;  //send interval when charge controller active (CS > 0)
long interval2 = 300000;  //send interval when charge controller inactive (CS == 0)

// Pin mapping
#define LMIC_NSS    8
#define LMIC_RXTX   LMIC_UNUSED_PIN
#define LMIC_RST    4
#define LMIC_DIO0   3
#define LMIC_DIO1   6

const lmic_pinmap lmic_pins = {
  .nss = LMIC_NSS,
  .rxtx = LMIC_RXTX,
  .rst = LMIC_RST,
  .dio = {LMIC_DIO0, LMIC_DIO1},
  .rxtx_rx_active = 0,
  .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
  .spi_freq = 8000000,
};

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static osjob_t sendjob;

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      //      {
      //        u4_t netid = 0;
      //        devaddr_t devaddr = 0;
      //        u1_t nwkKey[16];
      //        u1_t artKey[16];
      //        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
      //        Serial.print("netid: ");
      //        Serial.println(netid, DEC);
      //        Serial.print("devaddr: ");
      //        Serial.println(devaddr, HEX);
      //        Serial.print("AppSKey: ");
      //        for (size_t i=0; i<sizeof(artKey); ++i) {
      //          if (i != 0)
      //            Serial.print(" - ");
      //          printHex2(artKey[i]);
      //        }
      //        Serial.println("");
      //        Serial.print("NwkSKey: ");
      //        for (size_t i=0; i<sizeof(nwkKey); ++i) {
      //          if (i != 0)
      //            Serial.print(" - ");
      //          printHex2(nwkKey[i]);
      //        }
      //        Serial.println();
      //      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     Serial.println(F("EV_RFU1"));
      ||     break;
    */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
      }
      // Schedule next transmission
      //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE : no JoinAccept"));
      break;

    default:
      Serial.print(F("Unknown event : "));
      Serial.println((unsigned) ev);
      break;
  }
}

void do_send(osjob_t* j) {

  unsigned char mydata[14];

  //Convert Victron Charer Values
  int Vbatt = atoi(value[V]) / 10;
  int Ibatt = atoi(value[I]) / 10;
  int Iload = atoi(value[IL]) / 100;
  int Ppanel = atoi(value[PPV]);
  int StateOperation = atoi(value[CS]);
  int yieldday = atoi(value[H20]);
  int yieldtotal = atoi(value[H19]);

  int LoadState = 0;
  String LoadStateS;
  LoadStateS = value[LOAD];
  if (LoadStateS == "OFF") {
    LoadState = 0;
  } else {
    LoadState = 1;
  }

  mydata[0] = Vbatt >> 8;
  mydata[1] = Vbatt & 0xFF;
  mydata[2] = Ibatt >> 8;
  mydata[3] = Ibatt & 0xFF;
  mydata[4] = Iload >> 8;
  mydata[5] = Iload & 0xFF;
  mydata[6] = Ppanel;
  mydata[7] = StateOperation;
  mydata[8] = LoadState;
  mydata[9] = yieldtotal >> 8;
  mydata[10] = yieldtotal & 0xFF;
  mydata[11] = yieldday;

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    //Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, sizeof(mydata) , 0);
    //previousMillis = millis();
    //Serial.println(F("Packet queued"));
  }
  //Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  //Serial1.begin(19200);

  Serial2.begin(19200);

  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  //LMIC specific parameters
  LMIC_setAdrMode(0);
  LMIC_setLinkCheckMode(0);
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  //Join the Network
  previousMillis = millis();
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();

  // Receive information on Serial from MPPT
  RecvWithEndMarker();
  HandleNewData();

  SendLoRa();
}

// Serial Handling
// ---
// This block handles the serial reception of the data in a
// non blocking way. It checks the Serial line for characters and
// parses them in fields. If a block of data is send, which always ends
// with "Checksum" field, the whole block is checked and if deemed correct
// copied to the 'value' array.

void RecvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial2.available() > 0 && new_data == false) {
    rc = Serial2.read();
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= buffsize) {
        ndx = buffsize - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      new_data = true;
    }
  }
}

void HandleNewData() {
  // We have gotten a field of data
  if (new_data == true) {
    //Copy it to the temp array because parseData will alter it.
    strcpy(tempChars, receivedChars);
    ParseData();
    new_data = false;
  }
}

void ParseData() {
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(tempChars, "\t");     // get the first part - the label
  // The last field of a block is always the Checksum
  if (strcmp(strtokIndx, "Checksum") == 0) {
    blockend = true;
  }
  strcpy(recv_label[blockindex], strtokIndx); // copy it to label

  // Now get the value
  strtokIndx = strtok(NULL, "\r");    // This continues where the previous call left off until '/r'.
  if (strtokIndx != NULL) {           // We need to check here if we don't receive NULL.
    strcpy(recv_value[blockindex], strtokIndx);
  }
  blockindex++;

  if (blockend) {
    // We got a whole block into the received data.
    // Check if the data received is not corrupted.
    // Sum off all received bytes should be 0;
    byte checksum = 0;
    for (int x = 0; x < blockindex; x++) {
      // Loop over the labels and value gotten and add them.
      // Using a byte so the the % 256 is integrated.
      char *v = recv_value[x];
      char *l = recv_label[x];
      while (*v) {
        checksum += *v;
        v++;
      }
      while (*l) {
        checksum += *l;
        l++;
      }
      // Because we strip the new line(10), the carriage return(13) and
      // the horizontal tab(9) we add them here again.
      checksum += 32;
    }
    // Checksum should be 0, so if !0 we have correct data.
    if (!checksum) {
      // Since we are getting blocks that are part of a
      // keyword chain, but are not certain where it starts
      // we look for the corresponding label. This loop has a trick
      // that will start searching for the next label at the start of the last
      // hit, which should optimize it.
      int start = 0;
      for (int i = 0; i < blockindex; i++) {
        for (int j = start; (j - start) < num_keywords; j++) {
          if (strcmp(recv_label[i], keywords[j % num_keywords]) == 0) {
            // found the label, copy it to the value array
            strcpy(value[j], recv_value[i]);
            start = (j + 1) % num_keywords; // start searching the next one at this hit +1
            break;
          }
        }
      }
    } else {
    }
    // Reset the block index, and make sure we clear blockend.
    blockindex = 0;
    blockend = false;
  }
}

void PrintEverySecond() {
  static unsigned long prev_millis;
  if (millis() - prev_millis > 1000) {
    PrintValues();
    prev_millis = millis();
  }
}


void PrintValues() {
  for (int i = 0; i < num_keywords; i++) {
    Serial.print(keywords[i]);
    Serial.print(",");
    Serial.println(value[i]);
  }
}

//LoRa Sending
void SendLoRa() {
  if ((atoi(value[CS]) > 0) && (millis() - previousMillis > interval)) {
    do_send(&sendjob);
    previousMillis = millis();
  } else if (millis() - previousMillis > interval2) {
    do_send(&sendjob);
    previousMillis = millis();
  }
}

//Delay withput delay
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {} while (millis() - start < ms);
}
