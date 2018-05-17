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

void isr_tach1 () {
    int now = micros();

    tach1_duration = now - last_tach1_time;
    if (tach1_duration > 0) {
        tach1_valid = 1;
    }
    last_tach1_time = now;
}

void isr_tach2 () {
    int now = micros();

    tach2_duration = now - last_tach2_time;
    if (tach2_duration > 0) {
        tach2_valid = 1;
    }
    last_tach2_time = now;
}

ISR(PCINT0_vect) {
    byte tach1, tach2;

    tach1 = (PINB & bit(PIN_TACH1));
    tach2 = (PINB & bit(PIN_TACH2));

    if ((! tach1) && (last_tach1_state)) {
        isr_tach1();
    }

    if ((! tach2) && (last_tach2_state)) {
        isr_tach2();
    }

    last_tach1_state = tach1;
    last_tach2_state = tach2;
}
