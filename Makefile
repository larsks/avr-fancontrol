ARDUINO_DIR = $(HOME)/lib/arduino
ARDUINO_PACKAGE_DIR = $(HOME)/.arduino15/packages
ARDUINO_USER_DIR = $(HOME)/Arduino

ARDUINO_BUILDER = $(ARDUINO_DIR)/arduino-builder
ARDUINO_TOOLS = -tools $(ARDUINO_DIR)/hardware/tools \
		-tools $(ARDUINO_DIR)/hardware/tools/avr \
		-tools $(ARDUINO_DIR)/tools-builder

ARDUINO_HARDWARE = -hardware $(ARDUINO_DIR)/hardware \
		   -hardware $(ARDUINO_USER_DIR)/hardware \
		   -hardware $(ARDUINO_PACKAGE_DIR)

ARDUINO_LIBRARIES = -built-in-libraries $(ARDUINO_DIR)/libraries \
		    -libraries $(ARDUINO_USER_DIR)/libraries

ARDUINO_VERSION = 10804
ARDUINO_WARNINGS = -warnings=default

FQBN = ATTinyCore:avr:attinyx5:chip=85,clock=8internal,LTO=disable

MONITOR_PORT = /dev/ttyACM0
ISP_PORT = /dev/ttyACM0

SKETCHBOOK = $(wildcard *.ino)

build/%.ino.hex: $(SKETCHBOOK)
	mkdir -p build
	$(ARDUINO_BUILDER) \
		$(ARDUINO_TOOLS) \
		$(ARDUINO_HARDWARE) \
		$(ARDUINO_LIBRARIES) \
		$(ARDUION_WARNINGS) \
		-ide-version=$(ARDUINO_VERSION) \
		-verbose \
		-build-path $(PWD)/build \
		-fqbn=$(FQBN) \
		$<

all: build/$(SKETCHBOOK).hex

clean:
	rm -rf build
