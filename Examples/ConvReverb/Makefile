# Project Name
TARGET = ConvReverb

USE_FATFS = 1

# Sources
CPP_SOURCES = ConvReverb.cpp ../../Planter/planter.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy/
DAISYSP_DIR = ../../DaisySP/
DAISYSP_LGPL_DIR = ../../DaisySP/DaisySP-LGPL/
PLANTER_DIR = ../../Planter/

# CMSIS-DSP source directory
CMSIS_DSP_DIR = $(LIBDAISY_DIR)/Drivers/CMSIS-DSP

# Include paths
C_INCLUDES += -I$(PLANTER_DIR)
C_INCLUDES += -I$(DAISYSP_LGPL_DIR)/Source
C_INCLUDES += -I$(CMSIS_DSP_DIR)/Include
C_INCLUDES += -I$(LIBDAISY_DIR)/src

# CMSIS-DSP Transform / Table sources needed for arm_rfft_fast_f32
# NOTE: CommonTables.c already #includes arm_common_tables.c + arm_const_structs.c
#       so we must NOT list those individually to avoid multiple-definition errors.
C_SOURCES += \
	$(CMSIS_DSP_DIR)/Source/TransformFunctions/arm_rfft_fast_f32.c \
	$(CMSIS_DSP_DIR)/Source/TransformFunctions/arm_rfft_fast_init_f32.c \
	$(CMSIS_DSP_DIR)/Source/TransformFunctions/arm_cfft_f32.c \
	$(CMSIS_DSP_DIR)/Source/TransformFunctions/arm_cfft_init_f32.c \
	$(CMSIS_DSP_DIR)/Source/TransformFunctions/arm_cfft_radix8_f32.c \
	$(CMSIS_DSP_DIR)/Source/TransformFunctions/arm_bitreversal2.c \
	$(CMSIS_DSP_DIR)/Source/CommonTables/CommonTables.c

# Enable ARM DSP intrinsics for FIR / FFT
CPPFLAGS += -DUSE_ARM_DSP

# Select calibration unit (change per hardware unit)
CPPFLAGS += -DPLANTER_SERIAL=1

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core

# Set boot type for custom hardware
APP_TYPE = BOOT_SRAM


include $(SYSTEM_FILES_DIR)/Makefile
