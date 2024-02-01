# find the mk/ directory, which is where this makefile fragment
# lives. (patsubst strips the trailing slash.)
SYSTYPE			:=	$(shell uname)

ifneq ($(findstring CYGWIN, $(SYSTYPE)),) 
  MK_DIR := $(shell cygpath -m ../mk)
else
  MK_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
endif

include $(MK_DIR)/environ.mk

# short-circuit build for the configure target
ifeq ($(MAKECMDGOALS),configure)
include $(MK_DIR)/configure.mk

else

# short-circuit build for the help target
include $(MK_DIR)/help.mk

# common makefile components
include $(MK_DIR)/targets.mk
include $(MK_DIR)/sketch_sources.mk

ifneq ($(MAKECMDGOALS),clean)

# board specific includes
ifeq ($(BOARD_NAME),)
$(error, board name error)
endif

include $(MK_DIR)/board_$(BOARD_NAME).mk
endif

endif
