#include <OneWire.h>
#include <DallasTemperature.h>

#define PIN_LED 0
#define PIN_TACH1 1
#define PIN_TACH2 2
#define PIN_TEMP 3
#define PIN_RELAY 4

#define MIN_RPM 10
#define MAX_TEMP 65.0

#define DURATION_TO_RPM(d) ( (30000000/int(d)) )

OneWire ow(PIN_TEMP);
DallasTemperature sensors(&ow);
DeviceAddress tempAddress;

volatile int last_tach1_state, last_tach2_state;
volatile int tach1_valid = 0,
             tach1_duration = 0,
             last_tach1_time,

             tach2_valid = 0,
             tach2_duration = 0,
             last_tach2_time;


void setupInterrupts() {
    // Enable pin-change interrupts for the fan tach signal inputs
    cli();
    GIMSK |= bit(PCIE);
    PCMSK |= bit(digitalPinToPCMSKbit(PIN_TACH1));
    PCMSK |= bit(digitalPinToPCMSKbit(PIN_TACH2));
    sei();
}

void setupPins() {
    pinMode(PIN_TACH1, INPUT_PULLUP);
    pinMode(PIN_TACH2, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);
}

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

void setup() {
    setupPins();
    setupInterrupts();
    sensors.begin();

    discoverTemperatureSensor();
}

void turnLightOff() {
}

void turnLightOn() {
}

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
        turnLightOff();
    } else {
        turnLightOn();
    }

    delay(1000);
}

void calc_interval (int *this_interval, int *last_interval, int *valid) {
    int now = micros();

    *this_interval = now - *last_interval;
    if (*this_interval > 0) {
        *valid = 1;
    }
    *last_interval = now;
}

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
