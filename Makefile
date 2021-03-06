# Build and Install the EbbRT Toolchain & EbbLib (Linux) Library
#
# This Makefile provides a convenient way build and install the EbbRT "native"
# toolchain and "hosted" Linux libraries. 
#
# example: 
# 	$ make -f ~/EbbRT/Makefile -j=12 VERBOSE=1
#
# Options:
# 	DEBUG=1 						# build without optimisation
# 	CLEANUP=1     			# remove build state when finished
# 	PREFIX=<path> 			# install directory [=$PWD] 
# 	BUILD_ROOT					# build directory [=$PREFIX/build]
# 	EBBRT_SRCDIR=<path> # EbbRT source repository [=$HOME/EbbRT]
# 	VERBOSE=1   				# verbose build 
#
# Targets: 
# 	hosted native ebbrt-only ebbrt-libs clean 

-include config.mk # Local config (optional)

MYDIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
MKROOT ?= $(abspath $(CURDIR))
MYHOME ?= $(abspath $(HOME))
THISFILE := $(MYDIR)/Makefile
CD ?= cd
CMAKE ?= cmake
MAKE ?= time make
MKDIR ?= mkdir
RM ?= -rm
TEST ?= test

EBBRT_SRCDIR ?= $(MYDIR)
ifeq "$(wildcard $(EBBRT_SRCDIR) )" ""
  $(error Unable to locate source EBBRT_SRCDIR=$(EBBRT_SRCDIR))
endif
EBBRT_SRC ?= $(EBBRT_SRCDIR)/src
EBBRT_LIBS ?= $(EBBRT_SRCDIR)/libs
EBBRT_MAKEFILE ?= $(EBBRT_SRCDIR)/toolchain/Makefile

PREFIX ?= $(abspath $(CURDIR))
INSTALL_ROOT ?= $(PREFIX)
BUILD_ROOT ?= $(PREFIX)/build
NATIVE ?= $(INSTALL_ROOT)/sysroot
NATIVE_TOOLCHAIN_FILE ?= $(NATIVE)/usr/misc/ebbrt.cmake
HOSTED ?= $(INSTALL_ROOT)/hosted
HOSTED_BUILD_DIR ?= $(BUILD_ROOT)/hosted
NATIVE_BUILD_DIR ?= $(BUILD_ROOT)/native

ifdef CLEANUP
CLEANUP ?= $(RM) -rf  
else
CLEANUP ?= $(TEST)
endif

ifdef DEBUG
CMAKE_BUILD_OPT ?= -DCMAKE_BUILD_TYPE=Debug
MAKE_BUILD_OPT ?= DEBUG=1
else
CMAKE_BUILD_OPT ?= -DCMAKE_BUILD_TYPE=Release
MAKE_BUILD_OPT ?= 
endif

ifdef VERBOSE
MAKE_VERBOSE_OPT ?= VERBOSE=1
CMAKE_VERBOSE_OPT ?= -DCMAKE_VERBOSE_MAKEFILE=On
else
MAKE_VERBOSE_OPT ?= 
CMAKE_VERBOSE_OPT ?= 
endif

all: hosted native ebbrt-libs

clean:
	$(MAKE) -C $(HOSTED_BUILD_DIR) clean
	$(MAKE) -C $(NATIVE_BUILD_DIR) clean

hosted: | $(EBBRT_SRC)
	$(MKDIR) -p $(HOSTED_BUILD_DIR) && $(CD) $(HOSTED_BUILD_DIR) && \
	$(CMAKE) -DCMAKE_INSTALL_PREFIX=$(HOSTED)  \
	$(CMAKE_BUILD_OPT) $(CMAKE_VERBOSE_OPT) $(EBBRT_SRC) && \
	$(MAKE) $(MAKE_OPT) install && \
	$(CD) - && \
	$(CLEANUP) $(HOSTED_BUILD_DIR)
	
native: | $(EBBRT_MAKEFILE)
	$(MKDIR) -p $(NATIVE_BUILD_DIR) && $(CD) $(NATIVE_BUILD_DIR) && \
	$(MAKE) $(MAKE_OPT) -f $(EBBRT_MAKEFILE) SYSROOT=$(NATIVE) \
	$(MAKE_BUILD_OPT) $(MAKE_VERBOSE_OPT) && $(CD) - && \
	$(CLEANUP) $(NATIVE_BUILD_DIR)

ebbrt-libs: ebbrt-native-libs ebbrt-hosted-libs

ebbrt-hosted-libs: hosted 
	$(CD) $(HOSTED_BUILD_DIR) && $(MKDIR) -p libs && $(CD) libs && \
	$(CMAKE) -DCMAKE_INSTALL_PREFIX=$(HOSTED)  \
	$(CMAKE_BUILD_OPT) $(CMAKE_VERBOSE_OPT) $(EBBRT_LIBS) && \
	$(MAKE) $(MAKE_OPT) install && $(CD) - 
	
ebbrt-native-libs: ebbrt-only 
	$(CD) $(NATIVE_BUILD_DIR) && $(MKDIR) -p libs && $(CD) libs && \
	EBBRT_SYSROOT=$(NATIVE) $(CMAKE) -DCMAKE_INSTALL_PREFIX=$(NATIVE) \
	-DCMAKE_TOOLCHAIN_FILE=$(NATIVE_TOOLCHAIN_FILE) \
	$(CMAKE_BUILD_OPT) $(CMAKE_VERBOSE_OPT) $(EBBRT_LIBS) && \
	$(MAKE) $(MAKE_OPT) install && $(CD) - 

ebbrt-only: | $(EBBRT_MAKEFILE)
	$(MKDIR) -p $(NATIVE_BUILD_DIR) && $(CD) $(NATIVE_BUILD_DIR) && \
	$(MAKE) -f $(EBBRT_MAKEFILE) SYSROOT=$(NATIVE) \
	$(MAKE_BUILD_OPT) $(MAKE_VERBOSE_OPT) ebbrt-only && $(CD) - 

.PHONY: all clean hosted native ebbrt-only 
