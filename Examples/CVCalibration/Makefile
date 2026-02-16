# Project Name
TARGET = CVCalibration

USE_FATFS = 1

# Sources
CPP_SOURCES = CVCalibration.cpp ../../Planter/planter.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy/
DAISYSP_DIR = ../../DaisySP/
PLANTER_DIR = ../../Planter/

# Include paths
C_INCLUDES += -I$(PLANTER_DIR)

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core

# Set boot type for custom hardware
APP_TYPE = BOOT_SRAM

# Use external USB logger
CFLAGS += -DLOGGER_EXTERNAL=1
LDFLAGS += -u _printf_float
include $(SYSTEM_FILES_DIR)/Makefile
