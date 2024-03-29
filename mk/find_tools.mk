################################################################################
# Tools
#

#
# Decide where we are going to look for tools
#
ifeq ($(SYSTYPE),Darwin)
  # use the tools that come with Arduino
  # use BWK awk
  AWK =  awk
  FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1),$(TOOLPATH))))
endif
ifeq ($(SYSTYPE),Linux)
  # expect that tools are on the path
  TOOLPATH :=  $(subst :, ,$(PATH))
  FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1),$(TOOLPATH))))
endif
ifeq ($(findstring CYGWIN, $(SYSTYPE)),CYGWIN)
  TOOLPATH :=  $(subst :, ,$(PATH))
  FIND_TOOL = $(firstword $(wildcard $(addsuffix /$(1).exe,$(TOOLPATH))))
endif
ifeq ($(findstring MINGW, $(SYSTYPE)),MINGW)
  # expect that tools are on the path
  TOOLPATH :=  $(subst :, ,$(PATH))
  FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1).exe,$(TOOLPATH))))
endif

NATIVE_GCC_VERSION = -9
NATIVE_CXX     :=  g++$(NATIVE_GCC_VERSION)
NATIVE_CC      :=  gcc$(NATIVE_GCC_VERSION)
NATIVE_AS      :=  gcc$(NATIVE_GCC_VERSION)
NATIVE_AR      :=  ar
NATIVE_LD      :=  g++$(NATIVE_GCC_VERSION)
NATIVE_GDB     :=  gdb
NATIVE_OBJCOPY :=  objcopy

AVARICE      :=  $(call FIND_TOOL,avarice)

# Tools for Maple/Flymaple
# Toolchain is expected to be on the PATH
ARM_CXX     :=  $(call FIND_TOOL,arm-none-eabi-g++)
ARM_CC      :=  $(call FIND_TOOL,arm-none-eabi-gcc)
ARM_AS      :=  $(call FIND_TOOL,arm-none-eabi-gcc)
ARM_AR      :=  $(call FIND_TOOL,arm-none-eabi-ar)
ARM_LD      :=  $(call FIND_TOOL,arm-none-eabi-g++)
ARM_GDB     :=  $(call FIND_TOOL,arm-none-eabi-gdb)
ARM_OBJCOPY :=  $(call FIND_TOOL,arm-none-eabi-objcopy)


#$(TOOLCHAIN_PREFIX) from <root-quantracker-directory>/Dependencies.mk
QUAN_ARM_CXX     :=  $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-g++
QUAN_ARM_CC      :=  $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-gcc
QUAN_ARM_AS      :=  $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-gcc
QUAN_ARM_AR      :=  $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-ar
QUAN_ARM_LD      :=  $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-g++
QUAN_ARM_GDB     :=  $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-gdb
QUAN_ARM_OBJCOPY :=  $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-objcopy
QUAN_ARM_SIZE    :=  $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-size
QUAN_ARM_OBJDUMP :=  $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-objdump

# toolchains for beagleboneblack
BBONE_CXX     :=  arm-linux-gnueabihf-g++
BBONE_CC      :=  arm-linux-gnueabihf-gcc
BBONE_AS      :=  arm-linux-gnueabihf-gcc
BBONE_AR      :=  ar
BBONE_LD      :=  arm-linux-gnueabihf-g++
BBONE_GDB     :=  gdb
BBONE_OBJCOPY :=  objcopy

# toolchains for Raspberry Pi
RPI_CXX     :=  arm-linux-gnueabihf-g++
RPI_CC      :=  arm-linux-gnueabihf-gcc
RPI_AS      :=  arm-linux-gnueabihf-gcc
RPI_AR      :=  arm-linux-gnueabihf-ar
RPI_LD      :=  arm-linux-gnueabihf-g++
RPI_GDB     :=  arm-linux-gnueabihf-gdb
RPI_OBJCOPY :=  arm-linux-gnueabihf-obj

# toolchains for zynq
ZYNQ_CXX     :=  arm-xilinx-linux-gnueabi-g++
ZYNQ_CC      :=  arm-xilinx-linux-gnueabi-gcc
ZYNQ_AS      :=  arm-xilinx-linux-gnueabi-gcc
ZYNQ_AR      :=  arm-xilinx-linux-gnueabi-ar
ZYNQ_LD      :=  arm-xilinx-linux-gnueabi-g++
ZYNQ_GDB     :=  arm-xilinx-linux-gnueabi-gdb
ZYNQ_OBJCOPY :=  arm-xilinx-linux-gnueabi-objcopy

# enable ccache if installed
CCACHE :=  $(call FIND_TOOL,ccache)
export CCACHE

CXX = $(CCACHE) $($(TOOLCHAIN)_CXX)
CC = $(CCACHE) $($(TOOLCHAIN)_CC)
AS = $($(TOOLCHAIN)_AS)
AR = $($(TOOLCHAIN)_AR)
LD = $($(TOOLCHAIN)_LD)
GDB = $($(TOOLCHAIN)_GDB)
OBJCOPY = $($(TOOLCHAIN)_OBJCOPY)

ifeq ($(CXX),)
$(error ERROR: cannot find the compiler tools for $(TOOLCHAIN) anywhere on the path $(TOOLPATH))
endif

# Find awk
AWK			?=	gawk
ifeq ($(shell which $(AWK)),)
$(error ERROR: cannot find $(AWK) - you may need to install GNU awk)
endif
