################################################################################
# Sketch
#

SRCSUFFIXES = *.cpp *.c *.S

MAKE_INC=$(wildcard $(SRCROOT)/make.inc)
ifeq (,$(MAKE_INC))
$(error You must have a make.inc file to list library dependencies)
endif

SRC_EXCLUDES = $(wildcard $(SRCROOT)/exlude.inc)

# Sketch source files
SKETCHSRCS     := $(wildcard $(addprefix $(SRCROOT)/,$(SRCSUFFIXES)))
SKETCHCPP      := $(SRCROOT)/$(SKETCH).cpp

# Sketch object files
SKETCHOBJS := $(subst $(SRCROOT),$(BUILDROOT),$(SKETCHSRCS))
SKETCHOBJS := $(addsuffix .o,$(basename $(SKETCHOBJS)))

# get list of libraries from make.inc
include $(MAKE_INC)
LIBTOKENS := $(LIBRARIES)

# HAL and board specific libraries are included here.
LIBTOKENS += \
	AP_HAL \
	AP_HAL_Empty

ifeq ($(HAL_BOARD),HAL_BOARD_APM1)
LIBTOKENS += \
	AP_HAL_APM
endif

ifeq ($(HAL_BOARD),HAL_BOARD_APM2)
LIBTOKENS += \
	AP_HAL_APM
endif

ifeq ($(HAL_BOARD),HAL_BOARD_SITL)
LIBTOKENS += \
	AP_HAL_SITL \
	SITL
endif

ifeq ($(HAL_BOARD),HAL_BOARD_QUAN)
LIBTOKENS += \
   AP_HAL_Quan \
   AP_OSD
endif

#
# Find sketchbook libraries referenced by the sketch.
#
# Include paths for sketch libraries 
#
SKETCHLIBS		:=	$(wildcard $(addprefix $(SKETCHBOOK)/libraries/,$(LIBTOKENS)))
SKETCHLIBNAMES		:=	$(notdir $(SKETCHLIBS))
SKETCHLIBSRCDIRS	:=	$(SKETCHLIBS) $(addsuffix /utility,$(SKETCHLIBS))
SKETCHLIBSRCS		:=	$(wildcard $(foreach suffix,$(SRCSUFFIXES),$(addsuffix /$(suffix),$(SKETCHLIBSRCDIRS))))
SKETCHLIBOBJS		:=	$(addsuffix .o,$(basename $(subst $(SKETCHBOOK),$(BUILDROOT),$(SKETCHLIBSRCS))))
SKETCHLIBINCLUDES	:=	-I$(SKETCHBOOK)/libraries/
SKETCHLIBSRCSRELATIVE	:=	$(subst $(SKETCHBOOK)/,,$(SKETCHLIBSRCS))

ifeq ($(VERBOSE),)
v = @
else
v =
endif

FORCE: $(SKETCHELF)


$(BUILDROOT)/make.flags: FORCE
	@mkdir -p $(BUILDROOT)
	@echo "// BUILDROOT=$(BUILDROOT) HAL_BOARD=$(HAL_BOARD) HAL_BOARD_SUBTYPE=$(HAL_BOARD_SUBTYPE) TOOLCHAIN=$(TOOLCHAIN) EXTRAFLAGS=$(EXTRAFLAGS)" > $(BUILDROOT)/make.flags.new
	@cmp $(BUILDROOT)/make.flags $(BUILDROOT)/make.flags.new > /dev/null 2>&1 || rm -f $(SRCROOT)/*.o
	@cmp $(BUILDROOT)/make.flags $(BUILDROOT)/make.flags.new > /dev/null 2>&1 || mv $(BUILDROOT)/make.flags.new $(BUILDROOT)/make.flags
	@rm -f $(BUILDROOT)/make.flags.new
	@cat $(BUILDROOT)/make.flags

# common header for rules, prints what is being built
define RULEHDR
	@echo %% $(subst $(BUILDROOT)/,,$@)
	@mkdir -p $(dir $@)
endef
