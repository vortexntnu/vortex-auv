# Makefile for StreamCommon

ifndef VERBOSE
.SILENT :
endif
.SUFFIXES :

ifdef CONFIG
ifneq ($(CONFIG),Debug)
ifneq ($(CONFIG),InternalRelease)
ifneq ($(CONFIG),Release)
Error: unknown configuration.
endif
endif
endif
else
CONFIG=Debug
endif

INCLUDEPATHS=../../../thirdparty/Boost/boost-1.58.0-dynamic-linux-x64/installed/include . .. 
INCLUDEPATHS_Debug=Debug 
INCLUDEPATHS_InternalRelease=InternalRelease 
INCLUDEPATHS_Release=Release 
LIBRARYPATHS=../../../thirdparty/Boost/boost-1.58.0-dynamic-linux-x64/installed/lib 
LIBRARYPATHS_Debug=../../../lib/Debug ../../../bin/Debug
LIBRARYPATHS_InternalRelease=../../../lib/InternalRelease ../../../bin/InternalRelease
LIBRARYPATHS_Release=../../../lib/Release ../../../bin/Release
DEPENDENCIES=
LIBRARIES=
LIBRARIES_Debug=boost_wserialization-mt-d boost_wave-mt-d boost_unit_test_framework-mt-d boost_timer-mt-d boost_thread-mt-d boost_system-mt-d boost_signals-mt-d boost_serialization-mt-d boost_regex-mt-d boost_random-mt-d boost_python-mt-d boost_program_options-mt-d boost_prg_exec_monitor-mt-d boost_math_tr1l-mt-d boost_math_tr1f-mt-d boost_math_tr1-mt-d boost_math_c99l-mt-d boost_math_c99f-mt-d boost_math_c99-mt-d boost_log_setup-mt-d boost_log-mt-d boost_locale-mt-d boost_iostreams-mt-d boost_graph-mt-d boost_filesystem-mt-d boost_date_time-mt-d boost_coroutine-mt-d boost_context-mt-d boost_container-mt-d boost_chrono-mt-d boost_atomic-mt-d 
LIBRARIES_InternalRelease=boost_wserialization-mt boost_wave-mt boost_unit_test_framework-mt boost_timer-mt boost_thread-mt boost_system-mt boost_signals-mt boost_serialization-mt boost_regex-mt boost_random-mt boost_python-mt boost_program_options-mt boost_prg_exec_monitor-mt boost_math_tr1l-mt boost_math_tr1f-mt boost_math_tr1-mt boost_math_c99l-mt boost_math_c99f-mt boost_math_c99-mt boost_log_setup-mt boost_log-mt boost_locale-mt boost_iostreams-mt boost_graph-mt boost_filesystem-mt boost_date_time-mt boost_coroutine-mt boost_context-mt boost_container-mt boost_chrono-mt boost_atomic-mt 
LIBRARIES_Release=boost_wserialization-mt boost_wave-mt boost_unit_test_framework-mt boost_timer-mt boost_thread-mt boost_system-mt boost_signals-mt boost_serialization-mt boost_regex-mt boost_random-mt boost_python-mt boost_program_options-mt boost_prg_exec_monitor-mt boost_math_tr1l-mt boost_math_tr1f-mt boost_math_tr1-mt boost_math_c99l-mt boost_math_c99f-mt boost_math_c99-mt boost_log_setup-mt boost_log-mt boost_locale-mt boost_iostreams-mt boost_graph-mt boost_filesystem-mt boost_date_time-mt boost_coroutine-mt boost_context-mt boost_container-mt boost_chrono-mt boost_atomic-mt 
DEFINES=TCM_LINUX TCM_UNIX PROJECT_SOURCE_PATH=\"/home/swtest/workspace/DatastreamSDK_Linux_64_Branch_Release/Binary/bin/Release/deployment/Vicon/CrossMarket/StreamCommon\" 
DEFINES_Debug=_DEBUG 
DEFINES_InternalRelease=VICON_INTERNAL_RELEASE NDEBUG 
DEFINES_Release=NDEBUG TCM_OFF_SITE 
ENV_CPU=x64

SOURCEDIRECTORY=../../..
PROJECTPATH=.
BINARYDIRECTORY=../../..
INTERMEDIATEDIRECTORY=.
LIBRARYDIRECTORY=../../../lib
OUTPUTDIRECTORY=../../../bin

include $(BINARYDIRECTORY)/gcc.mk

HIDE_BOOST_SCRIPT=hide_boost_version_script
ifneq ($(HIDE_BOOST),)
    HIDE_BOOST_LD_PARAM= -Wl,--version-script=$(HIDE_BOOST_SCRIPT)
    HIDE_BOOST_LD_PREREQ=$(HIDE_BOOST_SCRIPT)
endif
all: all_$(CONFIG)

all_Debug: $(LIBRARYDIRECTORY)/$(CONFIG)/libStreamCommon.a
all_InternalRelease: $(LIBRARYDIRECTORY)/$(CONFIG)/libStreamCommon.a
all_Release: $(LIBRARYDIRECTORY)/$(CONFIG)/libStreamCommon.a

OBJECTS=$(CONFIG)/CGStreamAsyncReaderWriter.o 

CXXFLAGS=$(CXXFLAGS_$(CONFIG)) $(L_INCLUDEPATHS) $(L_INCLUDEPATHS_$(CONFIG)) $(L_INCLUDES) $(L_INCLUDES_$(CONFIG)) $(L_DEFINES) $(L_DEFINES_$(CONFIG))
CCFLAGS=$(CCFLAGS_$(CONFIG)) $(L_INCLUDEPATHS) $(L_INCLUDEPATHS_$(CONFIG)) $(L_INCLUDES) $(L_INCLUDES_$(CONFIG)) $(L_DEFINES) $(L_DEFINES_$(CONFIG))
LDFLAGS=$(LDFLAGS_$(CONFIG)) $(L_LIBRARYPATHS) $(L_LIBRARYPATHS_$(CONFIG))
CXXFLAGS+=-Wno-unused-local-typedefs
CXXFLAGS+=-Wno-delete-non-virtual-dtor -std=c++14
#Android toolchain does not include librt but integrates some of its functionality into Android libc.
ifndef ANDROID_TARGET_ARCH
LDFLAGS+=-lrt
endif


$(LIBRARYDIRECTORY)/Debug/libStreamCommon.a: makefile $(OBJECTS)
	@echo \[1\;35mLinking archive library $@\[0m
	@mkdir -p $(@D)
	$(AR) rcs $@ $(OBJECTS)

$(LIBRARYDIRECTORY)/InternalRelease/libStreamCommon.a: makefile $(OBJECTS)
	@echo \[1\;35mLinking archive library $@\[0m
	@mkdir -p $(@D)
	$(AR) rcs $@ $(OBJECTS)

$(LIBRARYDIRECTORY)/Release/libStreamCommon.a: makefile $(OBJECTS)
	@echo \[1\;35mLinking archive library $@\[0m
	@mkdir -p $(@D)
	$(AR) rcs $@ $(OBJECTS)

# Source Files
$(CONFIG)/CGStreamAsyncReaderWriter.o: makefile $(SOURCEDIRECTORY)/Vicon/CrossMarket/StreamCommon/CGStreamAsyncReaderWriter.cpp
	@echo \[1\;34mCompiling CGStreamAsyncReaderWriter.cpp\[0m
	@mkdir -p $(@D)
	find $(CONFIG) -name *.gch -exec cp '{}' . \;
	$(CXX) -fPIC -MMD -MP -I$(CONFIG)/ $(CXXFLAGS)  -o $@ -c $(SOURCEDIRECTORY)/Vicon/CrossMarket/StreamCommon/CGStreamAsyncReaderWriter.cpp

-include $(CONFIG)/CGStreamAsyncReaderWriter.d

# Header Files
# Other Files

clean:
	@echo \[1\;31mCleaning $(CONFIG) build\[0m
	find . -path '*/$(CONFIG)/*' \( -name '*.[od]' -o -name '*.gch' \) -exec rm -f {} ';' 
	rm -f moc_*.cxx

$(HIDE_BOOST_SCRIPT): makefile
	echo -n >$@
	echo "{" >>$@
	echo "  local: *N5boost*; *NK5boost*;" >>$@
	echo "};" >>$@
