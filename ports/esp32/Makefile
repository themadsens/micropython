# Makefile for MicroPython on ESP32.
#
# This is a simple, convenience wrapper around idf.py (which uses cmake).

ifneq (,$(wildcard ./my-settings.mk))
include my-settings.mk
endif
#
# Select the board to build for, defaulting to GENERIC.
BOARD ?= GENERIC

# If the build directory is not given, make it reflect the board name.
BUILD ?= build-$(BOARD)

# Device serial settings.
PORT ?= /dev/ttyUSB0
BAUD ?= 460800

PYTHON ?= python3

GIT_SUBMODULES = lib/berkeley-db-1.xx

.PHONY: all clean deploy erase submodules FORCE

CMAKE_ARGS =

ifdef USER_C_MODULES
	CMAKE_ARGS += -DUSER_C_MODULES=${USER_C_MODULES}
endif

IDFPY_FLAGS += -D MICROPY_BOARD=$(BOARD) -B $(BUILD) $(CMAKE_ARGS)

ifdef FROZEN_MANIFEST
       IDFPY_FLAGS += -D MICROPY_FROZEN_MANIFEST=$(FROZEN_MANIFEST)
endif

all:
	idf.py $(IDFPY_FLAGS) build
	$(PYTHON) makeimg.py \
		$(BUILD)/sdkconfig \
		$(BUILD)/bootloader/bootloader.bin \
		$(BUILD)/partition_table/partition-table.bin \
		$(BUILD)/micropython.bin \
		$(BUILD)/firmware.bin \
		$(BUILD)/micropython.uf2

$(BUILD)/bootloader/bootloader.bin $(BUILD)/partition_table/partition-table.bin $(BUILD)/micropython.bin: FORCE

clean:
	idf.py $(IDFPY_FLAGS) fullclean

deploy:
	idf.py $(IDFPY_FLAGS) --port $(PORT) --baud $(BAUD) flash

flash erase bootloader-flash erase_flash erase_otadata size size-components size-files:
	idf.py $(IDFPY_FLAGS) --port $(PORT) --baud $(BAUD) $@

PART_CSV = $(shell awk  -F '"' '/^CONFIG_PARTITION_TABLE_FILENAME=/ {print $$2}' $(BUILD)/sdkconfig)
VFS_SIZE = $(shell awk -F , '/^vfs/ {print $$5}' $(PART_CSV))
VFS_OFFSET = $(shell awk -F , '/^vfs/ {print $$4}' $(PART_CSV))
flashfs:
	# Partition table hardcoded for now (partitions-custom.csv)
	mklittlefs -s $(VFS_SIZE) -c lfs-root $(BUILD)/lfs.bin
	esptool.py --port $(PORT) --baud $(BAUD) write_flash $(VFS_OFFSET) $(BUILD)/lfs.bin

monitor:
	idf_monitor.py --port $(PORT) --eol CRLF $(BUILD)/micropython.elf

mon:
	python -m serial.tools.miniterm --rts 0 --dtr 0 --raw $(PORT) 115200

submodules:
	git submodule update --init $(addprefix ../../,$(GIT_SUBMODULES))
