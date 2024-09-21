SHELL=/usr/bin/env bash

PROJECT_NAME=buddy-beacon

# Build targets
TRANSMITTER_DIR:=src/transmitter
RECEIVER_DIR:=src/receiver

# ESP32 environment
ESP_VENV ?= $(HOME)/.espressif/python_env/idf5.3_py3.10_env
ESP32_TOOLCHAIN_DIR:=$(HOME)/.espressif/tools/xtensa-esp-elf/esp-13.2.0_20240530/xtensa-esp-elf
IDF_PATH ?= $(HOME)/esp/v5.3.1/esp-idf
IDF_PYTHON_ENV_PATH ?= $(ESP_VENV)

# PATH
export IDF_PYTHON_ENV_PATH := $(IDF_PYTHON_ENV_PATH)
export IDF_PATH := $(IDF_PATH)
export PATH := $(ESP32_TOOLCHAIN_DIR)/bin:$(IDF_PATH)/tools/:$(PATH)


# Build all
all: transmitter receiver

# Clean all
clean: clean_transmitter clean_receiver

# Setup ESP32 environment
menuconfig:
	@echo "Running menuconfig..."
	@cd $(TRANSMITTER_DIR) && source $(ESP_VENV)/bin/activate && idf.py menuconfig
	@cd $(RECEIVER_DIR) && source $(ESP_VENV)/bin/activate && idf.py menuconfig

# Build transmitter
transmitter:
	@echo "Building transmitter..."
	@cd $(TRANSMITTER_DIR) && source $(ESP_VENV)/bin/activate && idf.py build

# Build receiver
receiver:
	@echo "Building receiver..."
	@cd $(RECEIVER_DIR) && source $(ESP_VENV)/bin/activate && idf.py build

# Flash transmitter
flash_transmitter:
	@echo "Flashing transmitter..."
	@cd $(TRANSMITTER_DIR) && source $(ESP_VENV)/bin/activate && idf.py -p /dev/ttyUSB0 flash

# Flash receiver
flash_receiver:
	@echo "Flashing receiver..."
	@cd $(RECEIVER_DIR) && source $(ESP_VENV)/bin/activate && idf.py -p /dev/ttyUSB0 flash

# Clean transmitter
clean_transmitter:
	@echo "Cleaning transmitter..."
	@cd $(TRANSMITTER_DIR) && source $(ESP_VENV)/bin/activate && idf.py fullclean

# Clean receiver
clean_receiver:
	@echo "Cleaning receiver..."
	@cd $(RECEIVER_DIR) && source $(ESP_VENV)/bin/activate && idf.py fullclean

# Monitor transmitter
monitor_transmitter:
	@echo "Monitoring ESP32..."
	@cd $(TRANSMITTER_DIR) && source $(ESP_VENV)/bin/activate && idf.py -p /dev/ttyUSB0 monitor

# Monitor receiver
monitor_receiver:
	@echo "Monitoring ESP32..."
	@cd $(RECEIVER_DIR) && source $(ESP_VENV)/bin/activate && idf.py -p /dev/ttyUSB0 monitor

