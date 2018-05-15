#include <OneWire.h>
#include <DallasTemperature.h>

#define PIN_LED 0
#define PIN_DEBUG 4
#define PIN_TACH1 1
#define PIN_TACH2 2
#define PIN_TACH1_INT PCINT1
#define PIN_TACH2_INT PCINT2
#define PIN_TEMP 4
#define PIN_RELAY 5 

OneWire ow(PIN_TEMP);
DallasTemperature ds(&ow);
DeviceAddress temp;

int temp_valid = 0;


void setupPins() {
	cli();
	pinMode(PIN_TACH1, INPUT_PULLUP);
	pinMode(PIN_TACH2, INPUT_PULLUP);
	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_DEBUG, OUTPUT);

	GIMSK |= _BV(PCIE);
	PCMSK |= _BV(PIN_TACH1_INT);
	PCMSK |= _BV(PIN_TACH2_INT);
	sei();
}

void setup() {
	setupPins();

	digitalWrite(PIN_LED, 1);
	digitalWrite(PIN_DEBUG, 0);

	ds.begin();

	if (ds.getAddress(temp, 0)) {
		temp_valid = 1;
	}

}

void loop() {
	digitalWrite(PIN_LED, 1);
	delay(2000);    
}

ISR(PCINT0_vect) {
	byte tach1, tach2;

	digitalWrite(PIN_DEBUG, 1);

	tach1 = (PINB & _BV(PIN_TACH1_INT));
	tach2 = (PINB & _BV(PIN_TACH2_INT));

	if (! tach1) {
		digitalWrite(PIN_LED, 0);
	}

	if (! tach2) {
		digitalWrite(PIN_LED, 0);
	}

	digitalWrite(PIN_DEBUG, 0);
}
