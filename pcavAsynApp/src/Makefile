TOP=../..

include $(TOP)/configure/CONFIG

USR_CXXFLAGS += -DHAVE_YAML

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#========================================

INCLUDES += $(addprefix -I,$(YAML_INCLUDE))
INCLUDES += $(addprefix -I,$(BOOST_INCLUDE))

## this part is just for yaml loader driver
INCLUDES += $(addprefix -I,$(CPSW_FRAMEWORK_INCLUDE))

INCLUDES += $(addprefix -I,$(PCAVLIB_INCLUDE))

INC += pcavAsyn.h

DBD += pcavAsynDriver.dbd

LIBRARY_IOC += pcavAsyn

pcavAsyn_DBD  += pcavAsynDriver.dbd
pcavAsyn_SRCS += pcavAsyn.cpp


cpsw_DIR = $(CPSW_FRAMEWORK_LIB)
pcavLib_DIR = $(PCAVLIB_LIB)

pcavAsyn_LIBS += cpsw
pcavAsyn_LIBS += pcavLib
pcavAsyn_LIBS += asyn
pcavAsyn_LIBS += yamlLoader
pcavAsyn_LIBS += $(EPICS_BASE_IOC_LIBS)


#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
#

