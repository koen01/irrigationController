/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * Version 1.4 - koen01 
 *
 * DESCRIPTION
 * Example sketch showing how to control physical relays.
 * This example will remember relay state after power failure.
 * http://www.mysensors.org/build/relay
 * v. 1.4: updated with pulse watermeter sensor and soil moisture readings
 */

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Enable repeater functionality for this node
//#define MY_REPEATER_FEATURE

#include <SPI.h>
#include <MySensor.h>

#define RELAY_1  3  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define NUMBER_OF_RELAYS 2 // Total number of attached relays
#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay
#define CHILD_ID_MOISTURE 0
#define CHILD_ID_WATER 99                       // Id of the sensor child
#define ANALOG_INPUT_SOIL_SENSOR A0
#define DIGITAL_INPUT_SENSOR 2                  // The digital input you attached your sensor.  (Only 2 and 3 generates interrupt!)
#define PULSE_FACTOR 4500                       // Nummber of blinks per m3 of your meter (One rotation/liter)
#define SLEEP_MODE false                        // flowvalue can only be reported when sleep mode is false.
#define MAX_FLOW 60                             // Max flow (l/min) value to report. This filters outliers.
#define RF24_PA_LEVEL      RF24_PA_MAX
unsigned long SEND_FREQUENCY = 30000;           // Minimum time between send (in milliseconds). We don't want to spam the gateway.

double ppl = ((double)PULSE_FACTOR) / 1000;      // Pulses per liter

volatile unsigned long pulseCount = 0;
volatile unsigned long lastBlink = 0;
volatile double flow = 0;
boolean pcReceived = false;
unsigned long oldPulseCount = 0;
unsigned long newBlink = 0;
double oldflow = 0;
double volume = 0;
double oldvolume = 0;
unsigned long lastSend = 0;
unsigned long lastPulse = 0;

int lastSoilValue = -1;

MyMessage msgSoil(CHILD_ID_MOISTURE, V_WATT);
MyMessage flowMsg(CHILD_ID_WATER, V_FLOW);
MyMessage volumeMsg(CHILD_ID_WATER, V_VOLUME);
MyMessage lastCounterMsg(CHILD_ID_WATER, V_VAR1);

void before() {
  for (int sensor = 1, pin = RELAY_1; sensor <= NUMBER_OF_RELAYS; sensor++, pin++) {
    // Then set relay pins in output mode
    pinMode(pin, OUTPUT);
    // Set relay to last known state (using eeprom storage)
    digitalWrite(pin, loadState(sensor) ? RELAY_ON : RELAY_OFF);
  }
}

void setup() {
  pinMode(ANALOG_INPUT_SOIL_SENSOR, INPUT);
  pinMode(DIGITAL_INPUT_SENSOR, INPUT_PULLUP);

  pulseCount = oldPulseCount = 0;

  // Fetch last known pulse count value from gw
  request(CHILD_ID_WATER, V_VAR1);

  lastSend = lastPulse = millis();

  attachInterrupt(digitalPinToInterrupt(DIGITAL_INPUT_SENSOR), onPulse, FALLING);
}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Irrigation Controller", "1.4");

  for (int sensor = 1, pin = RELAY_1; sensor <= NUMBER_OF_RELAYS; sensor++, pin++) {
    // Register all sensors to gw (they will be created as child devices)
    present(sensor, S_LIGHT, "Valve", 1);
    present(CHILD_ID_MOISTURE, S_POWER, "Soil Moisture");
    present(CHILD_ID_WATER, S_WATER);
  }
}



void loop()
{
  boolean process();

  unsigned long currentTime = millis();

  // Only send values at a maximum frequency
  if (currentTime - lastSend > SEND_FREQUENCY)
  {
    lastSend = currentTime;

    if (!pcReceived) {
      //Last Pulsecount not yet received from controller, request it again
      request(CHILD_ID_WATER, V_VAR1);
      return;
    }

    if (flow != oldflow) {
      oldflow = flow;

      Serial.print("l/min:");
      Serial.println(flow);

      // Check that we dont get unresonable large flow value.
      // could happen when long wraps or false interrupt triggered
      if (flow < ((unsigned long)MAX_FLOW)) {
        send(flowMsg.set(flow, 2));                   // Send flow value to gw
      }
    }

    // No Pulse count received in 2min
    if (currentTime - lastPulse > 120000) {
      flow = 0;
    }

    // Pulse count has changed
    if (pulseCount != oldPulseCount)  {
      oldPulseCount = pulseCount;

      Serial.print("pulsecount:");
      Serial.println(pulseCount);

      send(lastCounterMsg.set(pulseCount));                  // Send  pulsecount value to gw in VAR1

      double volume = ((double)pulseCount / ((double)PULSE_FACTOR));
      if (volume != oldvolume) {
        oldvolume = volume;

        Serial.print("volume:");
        Serial.println(volume, 3);

        send(volumeMsg.set(volume, 3));               // Send volume value to gw
      }
    }

    // Read analog soil value
    int soilValue = analogRead(ANALOG_INPUT_SOIL_SENSOR);

    if (soilValue != lastSoilValue) {
      Serial.print("Soil:");
      Serial.println(soilValue);
      send(msgSoil.set(constrain(map(soilValue, 1023, 0, 0, 100), 0, 100)));
      lastSoilValue = soilValue;
    }
  }

}
void receive(const MyMessage & message) {
  // We only expect one type of message from controller. But we better check anyway.
  if (message.type == V_LIGHT) {
    // Change relay state
    digitalWrite(message.sensor - 1 + RELAY_1, message.getBool() ? RELAY_ON : RELAY_OFF);
    // Store state in eeprom
    saveState(message.sensor, message.getBool());
    // Write some debug info
    Serial.print("Incoming change for sensor:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getBool());
  }
  if (message.type == V_VAR1) {
    unsigned long gwPulseCount = message.getULong();
    pulseCount += gwPulseCount;
    flow = oldflow = 0;
    Serial.print("Received last pulse count from gw:");
    Serial.println(pulseCount);
    pcReceived = true;
  }
}

void onPulse()
{
  if (!SLEEP_MODE)
  {
    unsigned long newBlink = micros();
    unsigned long interval = newBlink - lastBlink;

    if (interval != 0)
    {
      lastPulse = millis();
      if (interval < 500000L) {
        // Sometimes we get interrupt on RISING,  500000 = 0.5sek debounce ( max 120 l/min)
        return;
      }
      flow = (60000000.0 / interval) / ppl;
    }
    lastBlink = newBlink;
  }
  pulseCount++;
}

