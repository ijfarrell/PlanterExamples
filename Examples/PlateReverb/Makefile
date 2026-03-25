# Project Name
TARGET = PhysPlateReverb

USE_FATFS = 1

# Sources
CPP_SOURCES = PhysPlateReverb.cpp ../../Planter/planter.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy/
DAISYSP_DIR = ../../DaisySP/
DAISYSP_LGPL_DIR = ../../DaisySP/DaisySP-LGPL/
PLANTER_DIR = ../../Planter/

# Include paths
C_INCLUDES += -I$(PLANTER_DIR)
C_INCLUDES += -I$(DAISYSP_LGPL_DIR)/Source

# Select calibration unit (change per hardware unit)
CPPFLAGS += -DPLANTER_SERIAL=1

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core

# Set boot type for custom hardware
APP_TYPE = BOOT_SRAM


include $(SYSTEM_FILES_DIR)/Makefile
