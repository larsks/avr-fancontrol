ARDUINO_QUIET = 1

ARDMK_DIR = $(HOME)/src/Arduino-Makefile
ARDUINO_DIR = $(HOME)/lib/arduino
ARDUINO_PACKAGE_DIR = $(HOME)/.arduino15/packages
MONITOR_PORT = /dev/ttyACM0

ALTERNATE_CORE_PATH = $(ARDUINO_PACKAGE_DIR)/ATTinyCore/hardware/avr/1.1.5
ALTERNATE_CORE = ATTinyCore
BOARD_TAG = attinyx5
BOARD_SUB = 85
F_CPU = 8000000L
ISP_PORT = /dev/ttyACM0

LDFLAGS = -Wl,--undefined=_mmcu,--section-start=.mmcu=0x910000
CPPFLAGS = -I$(HOME)/src/simavr/simavr/sim/avr

LOCAL_OBJS = metadata.c.o

include $(ARDMK_DIR)/Arduino.mk
