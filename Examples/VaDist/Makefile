# Project Name
TARGET = VaDist

USE_FATFS = 1

# Sources
CPP_SOURCES = VaDist.cpp ../../Planter/planter.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy/
DAISYSP_DIR = ../../DaisySP/
PLANTER_DIR = ../../Planter/

# Include paths
C_INCLUDES += -I$(PLANTER_DIR)

# Select calibration unit (change per hardware unit)
CPPFLAGS += -DPLANTER_SERIAL=1

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core

# Set boot type for custom hardware
APP_TYPE = BOOT_SRAM


include $(SYSTEM_FILES_DIR)/Makefile
