# STM32F3-Discovery Makefile
#
# Part of the stm32f3-discovery project
#
#######################################

# TARGET: name of the output file
TARGET = main

# MCU: part number to build for
MCU = STM32F3XX

# OUTDIR: directory to use for output
OUTDIR = build
MAINFILE = $(OUTDIR)/$(TARGET).bin

# STM32_PATH: path to STM32 Firmware folder
#STM32_PATH = $(HOME)/opt/STM32F3-Discovery_FW_V1.1.0
CMSIS_PATH = cmsis

PROJ := <REPLACEME>

# SOURCES: list of input source sources
SOURCEDIR = src
SOURCES = $(wildcard $(SOURCEDIR)/*.c)
ASM_SOURCES += $(SOURCEDIR)/startup_stm32f30x.S

# INCLUDES: list of includes, by default, use Includes directory
INCLUDES = -Iinc
#INCLUDES += -I$(STM32_PATH)/Utilities/STM32F3_Discovery
INCLUDES += -ISPL/inc
# INCLUDES += -I$(STM32_PATH)/Libraries/STM32_USB-FS-Device_Driver/inc
# INCLUDES += -I$(STM32_PATH)/Project/Demonstration
# INCLUDES += -I$(CMSIS_PATH)/Device/ST/STM32F30x/Include
INCLUDES += -I$(CMSIS_PATH)
#INCLUDES += -include$(STM32_PATH)/Project/Demonstration/stm32f30x_conf.h

# LIBRARIES
SPL_LIBDIR	= SPL/src
LIB_SOURCES	+= $(shell find $(SPL_LIBDIR) -name '*.c')
LIB_ASM_SRC	+= $(shell find $(SPL_LIBDIR) -name '*.S')
# USB_LIBDIR	= $(STM32_PATH)/Libraries/STM32_USB-FS-Device_Driver/src
# USB_LIBDIR	+= $(STM32_PATH)/Utilities/STM32F3_Discovery
# LIB_SOURCES	+= $(shell find $(USB_LIBDIR) -name '*.c')
LIB_OUTDIR	= lib/spl_build
STM32_LIB	= lib/libstm32_f3.a

# LD_SCRIPT: linker script
LD_SCRIPT = stm32f302r8_flash.ld
#LD_SCRIPT += stm32f302r8_sram.ld

# define flags
# CFLAGS = -g -mthumb -mthumb-interwork -mcpu=cortex-m4
# CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
# CFLAGS += -Os -MD -std=c99 -Wall -Wextra #-pedantic
# # All constants are assumed to be float (32 bit) and not
# # double (32 bit) by default and warn if a float value is implicit promoted
# # to double. Doubles are emulated in software while floats can use the FPU.
# CFLAGS += -fsingle-precision-constant -Wdouble-promotion
# # Enable the linker to discard unused functions
# CFLAGS += -ffunction-sections -fdata-sections
# CFLAGS += -D$(MCU) -DF_CPU=72000000 $(INCLUDES) -c

# ASFLAGS = -x assembler-with-cpp -fmessage-length=0 -mcpu=cortex-m4 -mthumb -gdwarf-2

# LDFLAGS = -mcpu=cortex-m4 -mthumb -T $(LD_SCRIPT) -L.
# LDFLAGS += -Wl,--relax -Wl,--gc-sections

CFLAGS = -g3 -O0 -Wall -Wextra -fdata-sections -ffunction-sections -nostartfiles
CFLAGS += -D$(MCU) $(INCLUDES) -std=c99 -mcpu=cortex-m4 -mfloat-abi=soft
CFLAGS += -fno-strict-aliasing -DSTM32F302R8 -DSTM32F30X -DUSE_STDPERIPH_DRIVER -c

ASFLAGS = -Wa,--gdwarf-2 -mcpu=cortex-m4 -mthumb

LDFLAGS = --specs=nosys.specs -mcpu=cortex-m4 -mthumb -T $(LD_SCRIPT) -L. -Wl,--gc-sections


#######################################
# output configs
#######################################
#ifeq ($(NOCOLOR),1)
	CYAN            = ""
	NORMAL          = ""
# else
# 	CYAN        = `tput setaf 6`
# 	NORMAL      = `tput sgr0`
# endif
#######################################
# binaries
#######################################
CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
AR = arm-none-eabi-ar
OBJCOPY = arm-none-eabi-objcopy
OPENOCD = openocd
FLASH	= st-flash
RM      = rm -rf
MKDIR	= mkdir -p
#######################################

# list of object files, placed in the build directory regardless of source path
OBJECTS = $(addprefix $(OUTDIR)/,$(notdir $(SOURCES:.c=.o)))
ASM_OBJECTS = $(addprefix $(OUTDIR)/,$(notdir $(ASM_SOURCES:.S=.o)))
LIB_OBJECTS = $(addprefix $(LIB_OUTDIR)/,$(notdir $(LIB_SOURCES:.c=.o)))

# default: build bin
all: $(STM32_LIB) $(OUTDIR)/$(TARGET).bin

$(OBJECTS): $(SOURCES) | $(OUTDIR)
	@echo -e "Compiling\t"$(CYAN)$(filter %$(subst .o,.c,$(@F)), $(SOURCES))$(NORMAL)
	@$(CC) $(CFLAGS) -o $@ $(filter %$(subst .o,.c,$(@F)), $(SOURCES))

$(ASM_OBJECTS): $(ASM_SOURCES) | $(OUTDIR)
	@echo -e "Assembling\t"$(CYAN)$(filter %$(subst .o,.S,$(@F)), $(ASM_SOURCES))$(NORMAL)
	@$(CC) $(ASFLAGS) -c $(filter %$(subst .o,.S,$(@F)), $(ASM_SOURCES)) -o $@

$(LIB_OBJECTS): $(LIB_SOURCES) | $(LIB_OUTDIR)
	@echo -e "Compiling\t"$(CYAN)$(filter %$(subst .o,.c,$(@F)), $(LIB_SOURCES))$(NORMAL)
	@echo -e "Outputting to file $@"
	@$(CC) $(CFLAGS) -o $@ $(filter %$(subst .o,.c,$(@F)), $(LIB_SOURCES))

$(OUTDIR)/$(TARGET): $(OBJECTS) $(ASM_OBJECTS) $(STM32_LIB)
	@echo -e "Linking\t\t"$(CYAN)$^$(NORMAL)
	@$(LD) $(LDFLAGS) -o $@ $^

$(MAINFILE): $(OUTDIR)/$(TARGET)
	@$(OBJCOPY) -O binary $< $@

$(STM32_LIB): $(LIB_OBJECTS)
	@echo -e "Making library \t"$(CYAN)$@$(NORMAL)
	@$(AR) rs $(STM32_LIB) $(LIB_OBJECTS)

# create the output directory
$(OUTDIR):
	$(MKDIR) $(OUTDIR)

$(LIB_OUTDIR):
	$(MKDIR) $(LIB_OUTDIR)

program: $(MAINFILE) 
	$(OPENOCD) -f openocd.cfg

flash: $(MAINFILE)
	$(FLASH) --reset write $(MAINFILE) 0x08000000

debug: flash
	./debug/nemiver.sh $(TARGET)

cleanall:
	-$(RM) $(OUTDIR)
	-$(RM) $(LIB_OUTDIR)
	-$(RM) $(STM32_LIB)

clean:
	-$(RM) $(OUTDIR)/*

.PHONY: all clean
