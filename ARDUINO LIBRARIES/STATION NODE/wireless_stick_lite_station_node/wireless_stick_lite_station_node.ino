/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// For this Sketch
#include "DHT.h"
 
// DHT Sensor
#define DHT1PIN 21
#define DHT2PIN 23

#define DHTTYPE1 DHT11
#define DHTTYPE2 DHT11

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0xe3, 0xe5, 0x7a, 0x04, 0x2f, 0xc6, 0x96, 0x21, 0xc8, 0x0b, 0x2b, 0x36, 0xf7, 0xd0, 0x26, 0xdf};

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x8e, 0x4c, 0xf1, 0x1b, 0xe6, 0x12, 0x69, 0x7f, 0x3a, 0xbf, 0x98, 0xfe, 0x7b, 0x9c, 0x2f, 0x20 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x0034e3ad ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t payload[13];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 35, 34},
};

//Init. DHT
DHT dht1(DHT1PIN, DHTTYPE1);
DHT dht2(DHT2PIN, DHTTYPE2);

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
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
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // read the temperature from the DHT22
        float temperature1 = dht1.readTemperature();
        Serial.print("Temperature: "); Serial.print(temperature1);
        Serial.println(" *C");
        // adjust for the f2sflt16 range (-1 to 1)
        temperature1 = temperature1 / 100; 
        

        // read the humidity from the DHT22
        float rHumidity1 = dht1.readHumidity();
        Serial.print("%RH ");
        Serial.println(rHumidity1);
        // adjust for the f2sflt16 range (-1 to 1)
        rHumidity1 = rHumidity1 / 100;

        // float -> int
        // note: this uses the sflt16 datum (https://github.com/mcci-catena/arduino-lmic#sflt16)
        uint16_t payloadTemp1 = LMIC_f2sflt16(temperature1);
        // int -> bytes
        byte temp1Low = lowByte(payloadTemp1);
        byte temp1High = highByte(payloadTemp1);
        // place the bytes into the payload
        payload[0] = temp1Low;
        payload[1] = temp1High;

        // float -> int
        uint16_t payloadHumid1 = LMIC_f2sflt16(rHumidity1);
        // int -> bytes
        byte humid1Low = lowByte(payloadHumid1);
        byte humid1High = highByte(payloadHumid1);
        payload[2] = humid1Low;
        payload[3] = humid1High;


        float temperature2 = dht2.readTemperature();
        Serial.print("Temperature2: "); Serial.print(temperature2);
        Serial.println(" *C");
        // adjust for the f2sflt16 range (-1 to 1)
        temperature2 = temperature2 / 100; 

        // read the humidity from the DHT22
        float rHumidity2 = dht2.readHumidity();
        Serial.print("%RH2 ");
        Serial.println(rHumidity2);
        // adjust for the f2sflt16 range (-1 to 1)
        rHumidity2 = rHumidity2 / 100;

        
                // float -> int
        // note: this uses the sflt16 datum (https://github.com/mcci-catena/arduino-lmic#sflt16)
        uint16_t payloadTemp2 = LMIC_f2sflt16(temperature2);
        // int -> bytes
        byte temp2Low = lowByte(payloadTemp2);
        byte temp2High = highByte(payloadTemp2);
        // place the bytes into the payload
        payload[4] = temp2Low;
        payload[5] = temp2High;

        // float -> int
        uint16_t payloadHumid2 = LMIC_f2sflt16(rHumidity2);
        // int -> bytes
        byte humid2Low = lowByte(payloadHumid2);
        byte humid2High = highByte(payloadHumid2);
        payload[6] = humid2Low;
        payload[7] = humid2High;
        
        float davisValue = analogRead(36);
        //Serial.print("davisVal: "); Serial.print(davisValue);
        float davisvoltage = (davisValue / 4096) * 3.3;
        float radiation = mapfloat(davisvoltage, 0, 3, 0, 1800);
        //Serial.print("radiation: "); Serial.print(radiation);
        // adjust for the f2sflt16 range (-1 to 1)
        radiation = radiation / 10000;
        // float -> int
        uint16_t payloadRadiation = LMIC_f2sflt16(radiation);
        // int -> bytes
        byte radiationLow = lowByte(payloadRadiation);
        byte radiationHigh = highByte(payloadRadiation);
        payload[8] = radiationLow;
        payload[9] = radiationHigh;
        
        float anemometerValue = analogRead(37);
        float anemometervoltage = (anemometerValue / 4096) * 3.3;
        float wind_speed = mapfloatws(anemometervoltage, 0.4, 2, 0, 32.4);
        //float speed_mph = (( wind_speed *3600)/1609.344);
        // adjust for the f2sflt16 range (-1 to 1)
        wind_speed = wind_speed / 100;
        // float -> int
        uint16_t payloadWind = LMIC_f2sflt16(wind_speed);
        // int -> bytes
        byte windLow = lowByte(payloadWind);
        byte windHigh = highByte(payloadWind);
        payload[10] = windLow;
        payload[11] = windHigh;

        
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
        //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    SPI.begin(5, 19, 27);
    delay(2000);
    Serial.begin(115200);
    Serial.println(F("Starting"));
    dht1.begin();
    dht2.begin();
    

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min)/(in_max - in_min) + out_min;
}

float mapfloatws(float xws, float in_minws, float in_maxws, float out_minws, float out_maxws)
{
  return (xws - in_minws) * (out_maxws - out_minws)/(in_maxws - in_minws) + out_minws;
}
