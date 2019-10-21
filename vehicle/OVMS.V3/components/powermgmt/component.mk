#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the
# src/ directory, compile them and link them into lib(subdirectory_name).a
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#

COMPONENT_SRCDIRS := src
COMPONENT_ADD_INCLUDEDIRS := src
COMPONENT_ADD_LDFLAGS = -Wl,--whole-archive -l$(COMPONENT_NAME) -Wl,--no-whole-archive

#
# ULP support additions to component makefile.
#
# 1. ULP_APP_NAME must be unique (if multiple components use ULP)
#    Default value, override if necessary:
ULP_APP_NAME ?= ulp_$(COMPONENT_NAME)
#
# 2. Specify all assembly source files here.
#    Files should be placed into a separate directory (in this case, ulp/),
#    which should not be added to COMPONENT_SRCDIRS.
ULP_S_SOURCES = $(addprefix $(COMPONENT_PATH)/ulp/, \
	adc.S \
	)
#
# 3. List all the component object files which include automatically
#    generated ULP export file, $(ULP_APP_NAME).h:
ULP_EXP_DEP_OBJECTS := ulp_powermgmt.o
#
# 4. Include build rules for ULP program 
include $(IDF_PATH)/components/ulp/component_ulp_common.mk
#
# End of ULP support additions to component makefile.
#
