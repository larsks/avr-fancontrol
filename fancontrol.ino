/**
 * \file fancontrol.ino
 */

#include <OneWire.h>
#include <DallasTemperature.h>

/** The output LED is used to indicate diagnostic or error conditions. */
#define PIN_LED 0

/** Tach signal from fan 1 */
#define PIN_TACH1 1

/** Tach signal from fan 2 */
#define PIN_TACH2 2

/** DS1820B temperature sensor */
#define PIN_TEMP 3

/** Relay to control the heating lamp */
#define PIN_RELAY 4

/** A fan will be considered "stopped" if the RPM is below this value. */
#define MIN_RPM 10

/** A temperature >= this value will trigger an over-temperature condition. */
#define MAX_TEMP 65.0

/**
 * A 4-pin fan emits two pulses per cycle, so given the interval between pulses
 * the rpm is `(ONE_MINUTE / (duration * 2))`.
 */
#define DURATION_TO_RPM(d) ( (30000000/int(d)) )

volatile int last_tach1_state,  /**< Last known state of the tach1 pin */
             last_tach2_state;  /**< Last known state of the tach2 pin */

volatile int tach1_valid = 0,       /**< True if tach 1 duration can be trusted */
             tach1_duration = 0,    /**< Duration of the most recent pulse interval for fan 1 */
             last_tach1_duration,   /**< Duration of the previous pulse interval for fan 1 */

             tach2_valid = 0,       /**< True if tach 2 duration can be trusted */
             tach2_duration = 0,    /**< Duration of the most recent pulse interval for fan 2 */
             last_tach2_duration;   /**< Duration of the previous pulse interval for fan 2 */


OneWire ow(PIN_TEMP);
DallasTemperature sensors(&ow);

/** Address of the temperature sensor */
DeviceAddress tempAddress;

/**
 * Enable pin-change interrupts for the fan tach signal inputs
 */
void setupInterrupts() {
    cli();
    GIMSK |= bit(PCIE);
    PCMSK |= bit(digitalPinToPCMSKbit(PIN_TACH1));
    PCMSK |= bit(digitalPinToPCMSKbit(PIN_TACH2));
    sei();
}

/**
 * Configure input and output pins.
 */
void setupPins() {
    pinMode(PIN_TACH1, INPUT_PULLUP);
    pinMode(PIN_TACH2, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_RELAY, OUTPUT);

    // Ensure the lamp is initially off. Note that the relay board
    // is active-low.
    digitalWrite(PIN_RELAY, 1);
}

/**
 * Discover temperature sensor.
 *
 * This routine will loop until it is able to acquire the address of the 
 * DS1820B sensor at index 0.
 */
void discoverTemperatureSensor() {
    digitalWrite(PIN_LED, 1);

    while (1) {
        int numDevices = 0;
        numDevices = sensors.getDeviceCount();
        if (numDevices == 0) goto end;
        if (sensors.getAddress(tempAddress, 0))
            break;

        end:
        delay(500);
    }

    digitalWrite(PIN_LED, 0);
}

/**
 * Called when microcontroller first boots, before calling `loop()`.
 */
void setup() {
    setupPins();
    setupInterrupts();
    sensors.begin();

    discoverTemperatureSensor();
}

/**
 * Turn off the heating lamp.
 */
void turnLampOff() {
     digitalWrite(PIN_RELAY, 1);
}

/**
 * Turn on the heating lamp.
 */
void turnLampOn() {
     digitalWrite(PIN_RELAY, 0);
}

/**
 * Called repeatedly when the microcontroller is running.
 */
void loop() {
    int tach1_rpm,
        tach2_rpm;
    float temp;

    digitalWrite(PIN_LED, digitalRead(PIN_LED) ^1);

    sensors.requestTemperatures();
    temp = sensors.getTempC(tempAddress);

    if (tach1_valid) {
        tach1_rpm = DURATION_TO_RPM(tach1_duration);
    } else {
        tach1_rpm = 0;
    }

    if (tach2_valid) {
        tach2_rpm = DURATION_TO_RPM(tach2_duration);
    } else {
        tach2_rpm = 0;
    }

    tach1_valid = 0;
    tach2_valid = 0;

    byte both_fans_stopped = ( (tach1_rpm < MIN_RPM) && (tach2_rpm < MIN_RPM) );
    byte temp_invalid = (temp == DEVICE_DISCONNECTED_C);
    byte temp_high = (temp >= MAX_TEMP);

    if (both_fans_stopped || temp_invalid || temp_high) {
        turnLampOff();
    } else {
        turnLampOn();
    }

    delay(1000);
}

/**
 * Calculate the time between two pulses.
 */
void calc_interval (volatile int *this_interval,
        volatile int *last_interval,
        volatile int *valid) {
    int now = micros();

    *this_interval = now - *last_interval;
    if (*this_interval > 0) {
        *valid = 1;
    }
    *last_interval = now;
}

/**
 * Interrupt service routine for any pin change interrupt.
 *
 * This handles interrupts from the fan tach signals. It maintains the last pin
 * state for each device and uses that to figure out which pin generated the
 * current interrupt.
 */
ISR(PCINT0_vect) {
    byte tach1, tach2;

    tach1 = (PINB & bit(PIN_TACH1));
    tach2 = (PINB & bit(PIN_TACH2));

    if ((! tach1) && (last_tach1_state)) {
        calc_interval(&tach1_duration, &last_tach1_duration, &tach1_valid);
    }

    if ((! tach2) && (last_tach2_state)) {
        calc_interval(&tach2_duration, &last_tach2_duration, &tach2_valid);
    }

    last_tach1_state = tach1;
    last_tach2_state = tach2;
}
