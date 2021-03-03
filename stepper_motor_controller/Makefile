# Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile

ARDUINO_DIR = /usr/share/arduino
ARDMK_DIR = /usr/share/arduino
AVR_TOOLS_DIR = /usr
ARDUINO_SKETCHBOOK = $(realpath ~)/sketchbook

# ARDUINO_QUIET = 0

BOARD_TAG = uno
MONITOR_PORT = /dev/ttyACM0

USER_LIB_PATH := $(realpath ../libraries)

AVRDUDE_OPTS = -v
# BOOTLOADER_FILE = optiboot/optiboot_atmega328.hex

ISP_PORT = /dev/ttyACM0

#AVRDUDE_ARD_BAUDRATE = 57600

BOARDS_TXT = $(ARDUINO_DIR)/hardware/arduino/boards.txt

include /usr/share/arduino/Arduino.mk
