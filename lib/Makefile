# Target specific macros
TARGET = lib

TARGET_CPP_SOURCES = 
	
TOPPERS_OSEK_OIL_SOURCE =

BUILD_MODE = 

# GCC optimisation level
#-Os optimisation for size
#-O1/2/3 optimisation for speed
C_OPTIMISATION_FLAGS = -Os

# Don't modify below part
O_PATH ?= build

# makefile for C++(.cpp) build
include ../../nxtOSEK/ecrobot/ecrobot++.mak

#--------------------------------------------------------------------------------------
# 'make doc' will use Doxygen to create documentation for the project. 'make libdoc'
# will do the same for the subdirectories which include ME405 library files.

.PHONY: doc
doc:
	doxygen doxygen.conf
