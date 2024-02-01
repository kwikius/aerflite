
# A typical invocation in ArduPlane would be
# make QUAN_TARGET_VEHICLE=QUAN_APM_ARDUPLANE QUANTRACKER_ROOT_DIR=/home/andy/ap_lib/quantracker/ quan
#QUANTRACKER_ROOT_DIR = /home/andy/cpp/projects/quantracker/

ifeq ($(QUANTRACKER_ROOT_DIR), )
# See http://github.com/kwikius/quantracker
$(error QUANTRACKER_ROOT_DIR must be defined to the path to the quantracker root directory.)
endif

TARGET_PROCESSOR = STM32F4

# dont need the extra flash
LINKER_SCRIPT = $(QUANTRACKER_ROOT_DIR)air/osd/stm32f4_aerflite.ld

HAVE_DEPENDENCIES_FILE := $(shell if test -f $(QUANTRACKER_ROOT_DIR)Dependencies.mk; then echo "True"; fi)

ifeq ($(HAVE_DEPENDENCIES_FILE), )
  quantracker-make-help:
	@echo ' '
	@echo '   ########## HELP - Quantracker OSD firmware build needs more info ############'
	@echo '   #                                                                           #'
	@echo '   #                Hi. Welcome to quantracker / air / OSD.                    #'
	@echo '   #                                                                           #'
	@echo '   #                To build the OSD firmware, you need to                     #'
	@echo '   #                create a Dependencies.mk file.                             #'
	@echo '   #                                                                           #'
	@echo '   #                Please read "Sample-Dependencies.mk" .                     #'
	@echo '   #                in this directory for further Details.                     #                                                          #'
	@echo '   #                                                                           #'
	@echo '   #############################################################################'
	@echo ' '
#$(error "stopping the build")
endif

# You will need a custom Dependencies.mk
include $(QUANTRACKER_ROOT_DIR)Dependencies.mk

###############################################################
ifeq ($(TOOLCHAIN_PREFIX), )
$(error "TOOLCHAIN_PREFIX must be defined to the path to the gcc-arm compiler - see README.")
endif

ifeq ($(QUAN_INCLUDE_PATH), )
$(error "QUAN_INCLUDE_PATH must be defined to the path to the quan library - see README.")
endif

ifeq ($(FREE_RTOS_DIR), )
$(error "FREE_RTOS_DIR must be defined to the path to the FreeRTOS library - see README.")
endif

ifeq ($(STM32_STD_PERIPH_LIB_DIR), )
$(error "STM32_STD_PERIPH_LIB_DIR must be defined to the path to the STM32 Std peripherals library - see README.")
endif

ifeq  ($(MIXER_LANG_PATH), )
$(error "MIXER_LANG_PATH - must be defined to path to mixer_lang - see README.")
endif

TOOLCHAIN = QUAN_ARM

STM32_INCLUDES = $(STM32_STD_PERIPH_LIB_DIR)CMSIS/Include \
$(STM32_STD_PERIPH_LIB_DIR)CMSIS/Device/ST/STM32F4xx/Include \
$(STM32_STD_PERIPH_LIB_DIR)STM32F4xx_StdPeriph_Driver/inc

RTOS_INCLUDES = \
$(FREE_RTOS_DIR)Source/include \
$(FREE_RTOS_DIR)Source/portable/GCC/ARM_CM4F \
$(QUANTRACKER_ROOT_DIR)air/osd

ifeq ($(OPTIMISATION_LEVEL), )
OPTIMISATION_LEVEL = O3
endif

ifeq ( $(CFLAG_EXTRAS), )
CFLAG_EXTRAS = -fno-math-errno
endif

#required for Ubuntu 12.x placid as system headers have been put in strange places
# these have beeen defined to thos in my bash .profile
CPLUS_INCLUDE_PATH=
C_INCLUDE_PATH=
LIBRARY_PATH=

include $(MK_DIR)/find_tools.mk

#ifeq ($(QUAN_TARGET_VEHICLE),)
#$(error "QUAN_TARGET_VEHICLE not defined")
#endif

ifeq ($(QUAN_TARGET_VEHICLE),QUAN_APM_ARDUPLANE)
TELEMETRY_DIRECTION = QUAN_OSD_TELEM_TRANSMITTER
endif

ifeq ($(QUAN_TARGET_VEHICLE),QUAN_APM_ANTENNATRACKER)
TELEMETRY_DIRECTION = QUAN_OSD_TELEM_RECEIVER
endif

# specific flags for stm32f4
QUAN_DEFINES = QUAN_STM32F4 QUAN_FREERTOS $(TELEMETRY_DIRECTION) STM32F40_41xxx \
QUAN_OSD_SOFTWARE_SYNCSEP HSE_VALUE=8000000 QUAN_OSD_BOARD_TYPE=4 QUAN_CUSTOM_AP_PARAMS
ifeq ($(AERFLITE),True)
QUAN_DEFINES += QUAN_AERFLITE_BOARD
endif

ifeq ($(MIXER_DISCO),True)
   QUAN_DEFINES += QUAN_MIXER_DISCO
else
ifeq ($(MIXER_FALCON),True)
   QUAN_DEFINES += QUAN_MIXER_FALCON
else
ifeq ($(MIXER_TRANQUILITY),True)
   QUAN_DEFINES += QUAN_MIXER_TRANQUILITY
else
   $(warning "No MIXER defined.")
endif
endif
endif

MIXER_LANG_INCLUDES = $(MIXER_LANG_PATH)/include

QUAN_INCLUDES = $(STM32_INCLUDES) $(QUAN_INCLUDE_PATH) $(QUANTRACKER_ROOT_DIR)include \
$(RTOS_INCLUDES) $(MIXER_LANG_INCLUDES)

QUAN_PROCESSOR_FLAGS = -march=armv7e-m -mtune=cortex-m4 -mhard-float -mthumb \
-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mthumb -mfloat-abi=hard

QUAN_CFLAGS  = -Wall -Werror -Wdouble-promotion -std=gnu++11 -fno-rtti -fno-exceptions -c -g \
-$(OPTIMISATION_LEVEL) $(QUAN_PROCESSOR_FLAGS) \
 $(CFLAG_EXTRAS) -fno-math-errno -Wl,-u,vsprintf -lm -fdata-sections -ffunction-sections\
-Wno-unused-local-typedefs

QUAN_LINKER_FLAGS  = -T$(LINKER_SCRIPT) -$(OPTIMISATION_LEVEL)  -nodefaultlibs \
 $(QUAN_PROCESSOR_FLAGS) --specs=nano.specs $(CFLAG_EXTRAS) -Wl,--gc-sections 

#------------------------------------------- ardupilot stuff --------

#VERBOSE = True
#
# Tool options
#
#DEFINES         =   -DF_CPU=$(F_CPU)
DEFINES         = 
DEFINES        +=   -DSKETCH=\"$(SKETCH)\" -DSKETCHNAME="\"$(SKETCH)\"" \
    -DSKETCHBOOK="\"$(SKETCHBOOK)\"" -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)
DEFINES        +=   $(EXTRAFLAGS)
DEFINES        +=   -DCONFIG_HAL_BOARD=$(HAL_BOARD) 
DEFINES        +=   $(patsubst %,-D%,$(QUAN_DEFINES))
ifneq ($(QUAN_CUSTOM_DEFINES),)
DEFINES        +=   $(patsubst %,-D%,$(QUAN_CUSTOM_DEFINES))
endif

WARNFLAGS       =   -Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align \
-Wno-unused-parameter -Wno-missing-field-initializers

WARNFLAGS      +=   -Wwrite-strings -Wformat=2
WARNFLAGSCXX    = -Wno-reorder \
	-Werror=unused-but-set-variable \
	-Werror=format-security \
	-Werror=array-bounds \
	-Wfatal-errors \
	-Werror=unused-but-set-variable \
	-Werror=uninitialized \
	-Werror=init-self \
	-Wno-missing-field-initializers \
   -Wno-psabi

DEPFLAGS        =   -MD -MT $@

CXXOPTS         =   $(QUAN_CFLAGS) -fsigned-char
COPTS           =   -ffunction-sections -fdata-sections -fsigned-char

ASOPTS          =   -x assembler-with-cpp 

# disable as this breaks distcc
#ifneq ($(SYSTYPE),Darwin)
#LISTOPTS        =   -adhlns=$(@:.o=.lst)
#endif

CPUFLAGS     = -D_GNU_SOURCE
CPULDFLAGS   = -g
OPTFLAGS     ?= -O3 -g

CXXFLAGS        =   -g $(CPUFLAGS) $(DEFINES) $(OPTFLAGS)
CXXFLAGS       +=   -std=gnu++11 $(WARNFLAGS) $(WARNFLAGSCXX) $(DEPFLAGS) $(CXXOPTS)
CFLAGS          =   -g $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(OPTFLAGS)
CFLAGS         +=   $(WARNFLAGS) $(DEPFLAGS) $(COPTS)
ASFLAGS         =   -g $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(DEPFLAGS)
ASFLAGS        +=   $(ASOPTS)
LDFLAGS         =   -g $(CPUFLAGS) $(OPTFLAGS) $(WARNFLAGS) $(QUAN_LINKER_FLAGS) 

ifneq ($(SYSTYPE),Darwin)
LDFLAGS        +=   -Wl,--gc-sections -Wl,-Map -Wl,$(SKETCHMAP)
endif

INIT_LIB_PREFIX = $(TOOLCHAIN_PREFIX)/lib/gcc/arm-none-eabi/$(TOOLCHAIN_GCC_VERSION)/armv7e-m/fpu/
INIT_LIBS = $(INIT_LIB_PREFIX)crti.o $(INIT_LIB_PREFIX)crtn.o 

#ifneq ($(findstring CYGWIN, $(SYSTYPE)),)
#LIBS += -lwinmm -lstdc++
#endif

LIBS = -Wl,--undefined=_sbrk  

ifneq ($(AERFLITE),True)
ifeq ($(QUAN_TARGET_VEHICLE),QUAN_APM_ANTENNATRACKER)
LIBS += $(QUANTRACKER_ROOT_DIR)lib/osd/quantracker_air_osd_rx.a 
endif

ifeq ($(QUAN_TARGET_VEHICLE),QUAN_APM_ARDUPLANE)
LIBS += $(QUANTRACKER_ROOT_DIR)lib/osd/quantracker_air_osd_tx.a
endif

LIBS += $(QUANTRACKER_ROOT_DIR)lib/osd/quantracker_air_system.a  \
$(QUANTRACKER_ROOT_DIR)lib/osd/quantracker_air_graphics_api.a

else

ifeq ($(QUAN_TARGET_VEHICLE),QUAN_APM_ANTENNATRACKER)
error( na for aerflite)
endif

ifeq ($(QUAN_TARGET_VEHICLE),QUAN_APM_ARDUPLANE)
LIBS += $(QUANTRACKER_ROOT_DIR)lib/osd/aerflite_osd_tx.a
endif
LIBS +=  $(QUANTRACKER_ROOT_DIR)lib/osd/aerflite_system.a  \
          $(QUANTRACKER_ROOT_DIR)lib/osd/aerflite_graphics_api.a 
          # $(MIXER_LANG_PATH)/build/stm32f405/mixer_lang.a

endif


ifeq ($(VERBOSE),)
v = @
else
v =
endif

# Library object files
LIBOBJS			:=	$(SKETCHLIBOBJS)

#QUAN_OBJS  := fonts.o

################################################################################
# Built products
#

# The ELF file
SKETCHELF		=	$(BUILDROOT)/$(SKETCH).elf
BUILDELF                =       $(notdir $(SKETCHELF))

# HEX file
SKETCHHEX		=	$(BUILDROOT)/$(SKETCH).hex

# EEP file
SKETCHEEP		=	$(BUILDROOT)/$(SKETCH).eep

# Map file
SKETCHMAP		=	$(BUILDROOT)/$(SKETCH).map

QUANSKETCHBIN =	$(BUILDROOT)/$(SKETCH).bin

QUANSKETCHLST   = $(BUILDROOT)/$(SKETCH).lst

# All of the objects that may be built
ALLOBJS			=	$(SKETCHOBJS) $(LIBOBJS)

# All of the dependency files that may be generated
ALLDEPS			=	$(ALLOBJS:%.o=%.d)

################################################################################
# Targets
#

all: $(SKETCHELF)

print-%:
	echo "$*=$($*)"

################################################################################
# Rules
#

# fetch dependency info from a previous build if any of it exists
-include $(ALLDEPS)

# Link the final object
#$(SKETCHELF): $(SKETCHOBJS) $(LIBOBJS) $(BUILDROOT)/fonts.o
#$(v)$(LD) $(LDFLAGS) -o $@   $(SKETCHOBJS) $(LIBOBJS) $(LIBS) $(BUILDROOT)/fonts.o
$(SKETCHELF): $(SKETCHOBJS) $(LIBOBJS) 
	@echo "Building $(SKETCHELF)"
	$(RULEHDR)
	$(v)$(LD) $(LDFLAGS) -o $@   $(SKETCHOBJS) $(LIBOBJS) $(LIBS)
	$(v)cp $(SKETCHELF) .
	@echo "Firmware is in $(BUILDELF)"
	$(QUAN_ARM_OBJCOPY) -Obinary $(SKETCHELF) $(QUANSKETCHBIN)
	$(v)cp $(QUANSKETCHBIN) .
	$(QUAN_ARM_OBJDUMP) -d $(SKETCHELF) > $(QUANSKETCHLST)
	$(v)cp $(QUANSKETCHLST) .
	$(QUAN_ARM_SIZE)    -A $(SKETCHELF)

SKETCH_INCLUDES	=	$(SKETCHLIBINCLUDES) $(patsubst %,-I%,$(QUAN_INCLUDES))
SLIB_INCLUDES	=	-I$(dir $<)utility $(SKETCHLIBINCLUDES) $(patsubst %,-I%,$(QUAN_INCLUDES))

include $(MK_DIR)/build_rules.mk

#$(BUILDROOT)/fonts.o : $(QUANTRACKER_ROOT_DIR)/examples/osd_example1/board/fonts.cpp
#	$(CXX) $(CXXFLAGS) -c -o $@ $< $(SLIB_INCLUDES)
