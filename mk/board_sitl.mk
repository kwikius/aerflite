TOOLCHAIN = NATIVE

include $(MK_DIR)/find_tools.mk


#ifeq ($(QUANTRACKER_ROOT_DIR), )
# See http://github.com/kwikius/quantracker
#$(error QUANTRACKER_ROOT_DIR must be defined to the path to the quantracker root directory.)
#endif

#include $(QUANTRACKER_ROOT_DIR)Dependencies.mk

QUAN_INCLUDE_PATH = /home/andy/cpp/projects/quan-trunk

MIXER_LANG_INCLUDES = $(MIXER_LANG_PATH)/include

#
# Tool options
#
DEFINES         =   -DF_CPU=$(F_CPU)
DEFINES        +=   -DSKETCH=\"$(SKETCH)\" -DSKETCHNAME="\"$(SKETCH)\"" -DSKETCHBOOK="\"$(SKETCHBOOK)\"" -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)
DEFINES        +=   $(EXTRAFLAGS)
DEFINES        +=   -DCONFIG_HAL_BOARD=$(HAL_BOARD) -DCONFIG_HAL_BOARD_SUBTYPE=$(HAL_BOARD_SUBTYPE) -DQUAN_NO_EXCEPTIONS
WARNFLAGS       =   -Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align -Wno-unused-parameter -Wno-missing-field-initializers
WARNFLAGS      +=   -Wwrite-strings -Wformat=2
WARNFLAGSCXX    = -Wno-reorder \
	-Werror=format-security \
	-Werror=array-bounds \
	-Wfatal-errors \
	-Werror=unused-but-set-variable \
	-Werror=uninitialized \
	-Werror=init-self \
	-Wno-missing-field-initializers
DEPFLAGS        =   -MD -MP -MT $@

CXXOPTS         =   -ffunction-sections -fdata-sections -fno-exceptions -fsigned-char
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
CXXFLAGS       +=   -std=gnu++2a -fconcepts $(WARNFLAGS) $(WARNFLAGSCXX) $(DEPFLAGS) $(CXXOPTS)
CFLAGS          =   -g $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(OPTFLAGS)
CFLAGS         +=   $(WARNFLAGS) $(DEPFLAGS) $(COPTS)
ASFLAGS         =   -g $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(DEPFLAGS)
ASFLAGS        +=   $(ASOPTS)
LDFLAGS         =   -g $(CPUFLAGS) $(OPTFLAGS) $(WARNFLAGS)

ifneq ($(SYSTYPE),Darwin)
LDFLAGS        +=   -Wl,--gc-sections -Wl,-Map -Wl,$(SKETCHMAP)
endif

LIBS ?= -lm -lpthread
ifneq ($(findstring CYGWIN, $(SYSTYPE)),)
LIBS += -lwinmm
endif

#VERBOSE=True
ifeq ($(VERBOSE),)
v = @
else
v =
endif

# Library object files
LIBOBJS			:=	$(SKETCHLIBOBJS)


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
$(SKETCHELF): $(SKETCHOBJS) $(LIBOBJS)
	@echo "Building $(SKETCHELF)"
	$(RULEHDR)
	$(v)$(LD) $(LDFLAGS) -o $@ $(SKETCHOBJS) $(LIBOBJS) $(LIBS) 
	$(v)cp $(SKETCHELF) .
	@echo "Firmware is in $(BUILDELF)"

AERFLITE_INCLUDES = -I$(QUAN_INCLUDE_PATH) 
SKETCH_INCLUDES	=	$(SKETCHLIBINCLUDES) $(AERFLITE_INCLUDES)
SLIB_INCLUDES	=	-I$(dir $<)/utility $(SKETCHLIBINCLUDES) $(AERFLITE_INCLUDES)

include $(MK_DIR)/build_rules.mk

