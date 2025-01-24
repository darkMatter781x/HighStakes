################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include
EXTRA_INCDIR=$(ROOT)/include/packages

WARNFLAGS+=
EXTRA_C_BOTH_FLAGS=-D_PROS_INCLUDE_LIBLVGL_LLEMU_H -D_PROS_INCLUDE_LIBLVGL_LLEMU_HPP
EXTRA_C_BOTH_FLAGS+=-Wno-deprecated-enum-enum-conversion 
# EXTRA_C_BOTH_FLAGS+=-DENABLE_PROFILING

ifndef SIM

$(info COMPILING FOR V5 BRAIN)

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1
BINDIR=$(ROOT)/bin/bot
else
$(info COMPILING FOR SIM)

USE_PACKAGE:=0
# TODO: Measure performance impact of -Ofast vs -O3
# TODO: Figure out potential safety concerns of -Ofast
# TODO: Add flag for all targets
EXTRA_C_BOTH_FLAGS+=-Ofast -lto

EXTRA_C_BOTH_FLAGS+=-DVEXIDE_SIMULATION -DENABLE_PROFILING
BINDIR=$(ROOT)/bin/sim
endif

# ifndef TUNING
# enable pid tuning
EXTRA_C_BOTH_FLAGS+=-DTUNING
# endif

EXTRA_CFLAGS+=$(EXTRA_C_BOTH_FLAGS)
EXTRA_CXXFLAGS+=$(EXTRA_C_BOTH_FLAGS)

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS! 
# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

.DEFAULT_GOAL=my-quick

my-quick: quick
	$(shell cp $(BINDIR)/*.bin $(BINDIR)/..)

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
